#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>



// Constants for engine control pins

constexpr int ALL_ENGINE_BRAKE_PIN = 45;
constexpr int ALL_ENGINE_SPEED_CONTROL_PIN = 35;
constexpr int ALL_ENGINE_DIRECTION_CONTROL_PIN = 37;

constexpr int ROTATION_SPEED_LEFT_PIN_F = 4;
constexpr int ROTATION_SPEED_RIGHT_PIN_F = 6;

constexpr int ROTATION_SPEED_LEFT_PIN_B = 15;
constexpr int ROTATION_SPEED_RIGHT_PIN_B = 8;



// Speed constants

int acceleration_motor_step = 5;
int acceleration_rotation_step = 5;
int DEADZONE_MOTOR_SPEED = 10;
int DEADZONE_ROTATION_SPEED = 10;

constexpr int STOP_MOTOR_SPEED = 0;
constexpr int MAX_MOTOR_SPEED = 100;
constexpr int BACKWARD_MAX_MOTOR_SPEED = - MAX_MOTOR_SPEED;

constexpr int STOP_ROTATION_SPEED = 0;
constexpr int MAX_ROTATION_SPEED = 100;
constexpr int BACKWARD_MAX_ROTATION_SPEED = - MAX_ROTATION_SPEED;

// ELRS setup

#define PIN_RX 18
#define PIN_TX 17

HardwareSerial controllerSerial(2);
AlfredoCRSF controller;


// ELRS channels

constexpr int ROTATION_CHANNEL = 1;
constexpr int SPEED_CHANNEL = 3;
constexpr int ROTATION_TYPE_CHANNEL = 5;
constexpr int DIRECTION_CHANNEL = 6;
constexpr int BRAKE_CHANNEL = 7;


// Channel values

constexpr int MAX_CHANNEL_VALUE = 2010;
constexpr int MIN_CHANNEL_VALUE = 990;
constexpr int MIDDLE_CHANNEL_VALUE = (MAX_CHANNEL_VALUE + MIN_CHANNEL_VALUE) / 2;


int forward_reversed_data = MIN_CHANNEL_VALUE;
int rotation_data = MIDDLE_CHANNEL_VALUE;
int direction_data = MIN_CHANNEL_VALUE;



// Rotation sensor

int rotation_angle_b = 0;
int rotation_angle_f = 0;
int deadzone_rotation_angle = 5;



int max_angle_difference = 10;
bool rotation_angle_is_too_high;

constexpr int MAX_ROTATION_ANGLE = 50;



// Time variables

int animation_delay = 30;
int restarting_animation_delay = 300;
int input_delay = 10;
int waiting_delay = 100;
int animation_timer = 0;


int current_num_turning_pixels = 0;



// Car states

bool braking;
bool motor_spinning_forward;
bool inverse_wheel_rotation;
int direction = 1;
int turn = 1;



// Speed variables

int current_motor_speed = 0;
int target_motor_speed = 0;
int current_rotation_speed = 0;
int target_rotation_speed = 0;



// LED strip pins and variables

#define PIN_LED_STRIP_F 20
#define PIN_LED_STRIP_B 19
#define NUM_PIXELS 20

constexpr uint32_t NUM_PIXELS_HALF = NUM_PIXELS / 2;


Adafruit_NeoPixel strip_f(NUM_PIXELS, PIN_LED_STRIP_F, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_b(NUM_PIXELS, PIN_LED_STRIP_B, NEO_GRB + NEO_KHZ800);



uint32_t LED_RED = strip_f.Color(255, 0, 0);
uint32_t LED_LIGHT_RED = strip_f.Color(50, 0, 0);
uint32_t LED_ORANGE = strip_f.Color(255, 50, 0);
uint32_t LED_WHITE = strip_f.Color(255, 255, 255);
uint32_t LED_NO_COLOR = strip_f.Color(0, 0, 0);




void setup() {
  pinMode(ALL_ENGINE_SPEED_CONTROL_PIN, OUTPUT);
  pinMode(ALL_ENGINE_BRAKE_PIN, OUTPUT);
  pinMode(ALL_ENGINE_DIRECTION_CONTROL_PIN, OUTPUT);

  pinMode(ROTATION_SPEED_LEFT_PIN_F, OUTPUT);
  pinMode(ROTATION_SPEED_RIGHT_PIN_F, OUTPUT);

  pinMode(ROTATION_SPEED_LEFT_PIN_B, OUTPUT);
  pinMode(ROTATION_SPEED_RIGHT_PIN_B, OUTPUT);


  strip_f.begin();
  strip_b.begin();
  no_led();
  all_leds_show();
  reset_wheels_angle();

  Serial.begin(115200); // Initialize serial communication

  controllerSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!controllerSerial) while (1) Serial.println("Invalid controllerSerial configuration");

  controller.begin(controllerSerial);
  Serial.println("Initialization complete");

  Serial.printf("code number: 1\n");
}



void loop() {
  controller.update();
  // printChannels();
  Serial.printf(".");


  if (no_controller_connection()) {
    delay(waiting_delay);
    return;
  }

  direction_data = controller.getChannel(DIRECTION_CHANNEL);
  update_direction(direction_data);


  target_motor_speed = get_target_motor_speed();
  braking = is_braking(direction, current_motor_speed, target_motor_speed);
  if (current_motor_speed == 0 && target_motor_speed > 0) {
    int backwards_speed_at_start = 30;
    set_motor_speed(backwards_speed_at_start);
  }
  current_motor_speed = updated_value(current_motor_speed, target_motor_speed, acceleration_motor_step);
  set_motor_speed(current_motor_speed);

  inverse_wheel_rotation = get_wheel_rotation_type();


  turn = 1;
  target_rotation_speed = get_target_rotation_speed();
  if (rotation_is_changed(current_rotation_speed, target_rotation_speed)) {
    led_no_turn();
  }
  current_rotation_speed = target_rotation_speed;
  if (direction != 0) {
    set_rotation(current_rotation_speed);
  }

  // target_rotation_speed = get_target_rotation_speed();
  // if (rotation_is_changed(current_rotation_speed, target_rotation_speed)) {
  //   led_no_turn();
  // }
  // current_rotation_speed = target_rotation_speed;
  // set_rotation(current_rotation_speed);


  Serial.printf("target_motor_speed: %d\n2\n", target_motor_speed);
  Serial.printf("current_motor_speed: %d\n", current_motor_speed);

  Serial.printf("target_rotation_speed: %d\n", target_rotation_speed);
  Serial.printf("current_rotation_speed: %d\n", current_rotation_speed);


  update_led();


  delay(input_delay);
}


void reset_wheels_angle() {
  reset_wheels_angle_f();
  reset_wheels_angle_b();
}


void reset_wheels_angle_f() {
  if (rotation_angle_f > deadzone_rotation_angle) {
    turn_left_f();
    // while (rotation_angle_f > deadzone_rotation_angle) {};
  } else {
    turn_right_f();
    // while (rotation_angle_f < -deadzone_rotation_angle) {};
  }
  no_turn_f();
}

void reset_wheels_angle_b() {
  // while (rotation_angle_b > deadzone_rotation_angle) {
  //   turn_left_b();
  // }
  // while (rotation_angle_b < -deadzone_rotation_angle) {
  //   turn_right_b();
  // }
  no_turn_b();
}

int apply_deadzone(int value, int deadzone, int replacing_value){
  if (abs(value) < deadzone) {
    return replacing_value;
  }
  return value;
}


int get_wheel_rotation_type() {
  return controller.getChannel(ROTATION_TYPE_CHANNEL) <= MIDDLE_CHANNEL_VALUE - 100;
}


int get_target_motor_speed() {
  forward_reversed_data = constrain(controller.getChannel(SPEED_CHANNEL), MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE);
  forward_reversed_data = map(forward_reversed_data, MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE, 0, MAX_MOTOR_SPEED) * direction;
  return apply_deadzone(forward_reversed_data, DEADZONE_MOTOR_SPEED, STOP_MOTOR_SPEED);
}


int get_target_rotation_speed() {
  rotation_data = constrain(controller.getChannel(ROTATION_CHANNEL), MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE);
  rotation_data = map(rotation_data, MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE, BACKWARD_MAX_ROTATION_SPEED, MAX_ROTATION_SPEED) * turn;
  return apply_deadzone(rotation_data, DEADZONE_ROTATION_SPEED, STOP_ROTATION_SPEED);
}


bool no_controller_connection() {
  return controller.getChannel(SPEED_CHANNEL) < MIN_CHANNEL_VALUE - 200;
}



void update_direction(int direction_data) {
  if (direction_data <= MIN_CHANNEL_VALUE + 100) {
    direction = 1;
    turn = 1;
  } else if (direction_data >= MAX_CHANNEL_VALUE - 100) {
    direction = -1;
    turn = 1;
  } else {
    direction = 0;
    turn = 0;
  }
}

bool is_moving(int direction) {
  return direction != 0;
}

bool is_braking(int direction, int motor_speed, int target_motor_speed) {
  return direction == -1 || motor_speed > target_motor_speed + 3;
}

int updated_value(int value, int target_value, int increment_step) {
  if (value < target_value) {
    value = min(value + increment_step, target_value);
  }
  else if (value > target_value) {
    value = max(value - increment_step, target_value);
  }
  return value;
}

void set_motor_speed(int motor_speed) {
  motor_spinning_forward = motor_speed > 0;
  digitalWrite(ALL_ENGINE_DIRECTION_CONTROL_PIN, motor_spinning_forward);
  analogWrite(ALL_ENGINE_SPEED_CONTROL_PIN, abs(motor_speed));
}

bool rotation_is_changed(int value, int target_value) {
  return (value > 0) && (target_value < 0) || (value < 0) && (target_value > 0);
}

void set_rotation(int rotation_speed) {
  bool rotation_is_calibrated = abs(rotation_angle_f - rotation_angle_b) < max_angle_difference;

  if (rotation_is_calibrated) {
    rotation_angle_is_too_high = max(rotation_angle_f, rotation_angle_b) >= MAX_ROTATION_ANGLE;
    if (rotation_speed == 0 || rotation_angle_is_too_high) {
      no_turn();
    }
    else if (rotation_speed > 0) {
      turn_right();
    }
    else {
      turn_left();
    }
    return;
  }
  calibrate_rotation(rotation_speed);
}

void calibrate_rotation(int rotation_speed) {
  if (!inverse_wheel_rotation) {
    rotation_angle_b = -rotation_angle_b;
  }
  if (rotation_speed > 0) {
    if (rotation_angle_f > rotation_angle_b) {
      turn_right_b();
      return;
    }
    turn_right_f();
    return;
  }
  if (rotation_angle_f < rotation_angle_b) {
    turn_left_b();
    return;
  }
  turn_left_f();
}


void update_led() {
  led_moving();
  if (braking) {
    led_braking();
  }
  if (current_rotation_speed > 0) {
    led_turn_right();
  } else if (current_rotation_speed < 0) {
    led_turn_left();
  }
  all_leds_show();
}


void all_leds_show() {
    strip_f.show();
    strip_b.show();
}


void led_moving() {
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
    strip_f.setPixelColor(pixel, LED_WHITE);
    strip_b.setPixelColor(pixel, LED_LIGHT_RED);
  }
}


void led_braking() {
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
    strip_b.setPixelColor(pixel, LED_RED);
  }
}

void led_no_turn() {
  current_num_turning_pixels = 0;
  animation_timer = 0;
}

void update_animation() {
  if (animation_timer <= 0) {
    animation_timer = animation_delay;
    ++current_num_turning_pixels;


    if (current_num_turning_pixels == NUM_PIXELS_HALF) {
      animation_timer = restarting_animation_delay;
    }
    else if (current_num_turning_pixels > NUM_PIXELS_HALF) {
      current_num_turning_pixels = 0;
      animation_timer = animation_delay;
    }
  }

  animation_timer -= input_delay;
}

void led_turn_left() {
  update_animation();
  for (int pixel = NUM_PIXELS_HALF; pixel < NUM_PIXELS_HALF + current_num_turning_pixels; pixel++) {
    strip_f.setPixelColor(pixel, LED_ORANGE);
    strip_b.setPixelColor(pixel, LED_ORANGE);
  }
  for (int pixel = NUM_PIXELS_HALF + current_num_turning_pixels; pixel < NUM_PIXELS; pixel++) {
    strip_f.setPixelColor(pixel, LED_NO_COLOR);
    strip_b.setPixelColor(pixel, LED_NO_COLOR);
  }
}


void led_turn_right() {
  update_animation();
  for (int pixel = 0; pixel < current_num_turning_pixels; pixel++) {
    strip_f.setPixelColor(pixel, LED_ORANGE);
    strip_b.setPixelColor(pixel, LED_ORANGE);
  }
  for (int pixel = current_num_turning_pixels; pixel < NUM_PIXELS_HALF; pixel++) {
    strip_f.setPixelColor(pixel, LED_NO_COLOR);
    strip_b.setPixelColor(pixel, LED_NO_COLOR);
  }
}


void no_led() {
    strip_f.clear();
    strip_b.clear();
}


void no_turn_b() {
  digitalWrite(ROTATION_SPEED_LEFT_PIN_B, 0);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_B, 0);
}

void no_turn_f() {
  digitalWrite(ROTATION_SPEED_LEFT_PIN_F, 0);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_F, 0);
}


void no_turn(){
  no_turn_f();
  no_turn_b();
}


void turn_left_b() {
  digitalWrite(ROTATION_SPEED_LEFT_PIN_B, 1);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_B, 0);
}

void turn_right_b() {
  digitalWrite(ROTATION_SPEED_LEFT_PIN_B, 0);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_B, 1);
}

void turn_left_f() {
  digitalWrite(ROTATION_SPEED_LEFT_PIN_F, 1);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_F, 0);
}

void turn_right_f() {
  digitalWrite(ROTATION_SPEED_LEFT_PIN_F, 0);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_F, 1);
}

void turn_left(){
  turn_left_f();

  if (inverse_wheel_rotation) {
    turn_right_b();
  } else {
    turn_left_b();
  }

}


void turn_right(){
  turn_right_f();

  if (inverse_wheel_rotation) {
    turn_left_b();
  } else {
    turn_right_b();
  }
}


void block_wheels() {
  digitalWrite(ALL_ENGINE_BRAKE_PIN, HIGH);
}


//Use controller.getChannel(x) to get us channel values (1-16).
void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 12; ChannelNum++)
  {
    Serial.printf("%d: %d", ChannelNum, controller.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println("");
}
