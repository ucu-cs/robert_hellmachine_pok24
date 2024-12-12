#include "config.h"
#include "LED_module.h"
#include "motors_controllers_module.h"

#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "AS5600.h"


// Define custom TwoWire instances
TwoWire Wire_f(0);  // First I2C bus
TwoWire Wire_b(1);  // Second I2C bus
AS5600 sensor_f(&Wire_f);   //  use default Wire
AS5600 sensor_b(&Wire_b);   //  use default Wire

// Speed constants
int acceleration_motor_step = 10;
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
HardwareSerial controllerSerial(2);
AlfredoCRSF controller;


// Channel values
constexpr int MAX_CHANNEL_VALUE = 2010;
constexpr int MIN_CHANNEL_VALUE = 990;
constexpr int MIDDLE_CHANNEL_VALUE = (MAX_CHANNEL_VALUE + MIN_CHANNEL_VALUE) / 2;


int forward_reversed_data = MIN_CHANNEL_VALUE;
int rotation_data = MIDDLE_CHANNEL_VALUE;
int direction_data = MIN_CHANNEL_VALUE;


// Rotation sensor
int max_rotation_angle = 1700;
int starting_angle_f = 2136;
int starting_angle_b = 2150;
int rotation_angle_b = 0;
int rotation_angle_f = 0;
int deadzone_rotation_angle = 50;


int max_angle_difference = 100;
// bool rotation_angle_is_too_high;

constexpr int MAX_ROTATION_ANGLE = 50;

int input_delay = 10;
int waiting_delay = 500;

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
  // all_leds_show();
  // reset_wheels_angle();

  Serial.begin(115200); // Initialize serial communication

  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  bool sensor_b_connected = Wire_b.begin(PIN_SENSOR_SDA_B, PIN_SENSOR_SCL_B);
  bool sensor_f_connected = Wire_f.begin(PIN_SENSOR_SDA_F, PIN_SENSOR_SCL_F);

  sensor_b.begin(PIN_SENSOR_DIR_B);
  sensor_f.begin(PIN_SENSOR_DIR_F);
  
  sensor_b.setDirection(AS5600_CLOCK_WISE);
  sensor_f.setDirection(AS5600_CLOCK_WISE);

  int b = sensor_b.isConnected();
  int f = sensor_f.isConnected();

  Serial.print("Connect_sensor_b: ");
  Serial.println(b);
  Serial.print("Connect_sensor_f: ");
  Serial.println(f);

  rotation_angle_b = get_angle_b();
  rotation_angle_f = get_angle_f();
  // reset_wheels_angle();

  controllerSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!controllerSerial) while (1) Serial.println("Invalid controllerSerial configuration");

  controller.begin(controllerSerial);
  Serial.println("Initialization complete");

  Serial.printf("code number: 4\n");
  delay(100);
}


void loop() {
  controller.update();
  // printChannels();


  Serial.printf("sensor_f angle4: %d\n", get_angle_f());
  // Serial.printf("sensor_b angle: %d\n", get_angle_b());


  if (no_controller_connection()) {
    delay(waiting_delay);
    return;
  }

  if (controller.getChannel(RESET_CHANNEL) < MIN_CHANNEL_VALUE + 100) {
    reset_wheels_angle();
  }
  else if (controller.getChannel(RESET_CHANNEL) > MAX_CHANNEL_VALUE - 100) {
    max_rotation_angle = 5000;
  }
  else {
    max_rotation_angle = 1500;
  }



  direction_data = controller.getChannel(DIRECTION_CHANNEL);
  update_direction(direction_data);


  target_motor_speed = get_target_motor_speed();
  braking = is_braking(direction, current_motor_speed, target_motor_speed);
  if (current_motor_speed == 0 && target_motor_speed > 0) {
    int backwards_speed_at_start = 50;
    set_motor_speed(backwards_speed_at_start);
  }
  current_motor_speed = updated_value(current_motor_speed, target_motor_speed, acceleration_motor_step);
  set_motor_speed(current_motor_speed);

  inverse_wheel_rotation = get_wheel_rotation_type();


  turn = 1;
  rotation_angle_f = get_angle_f();
  rotation_angle_b = get_angle_b();
  target_rotation_speed = get_target_rotation_speed();
  if (rotation_is_changed(current_rotation_speed, target_rotation_speed)) {
    led_no_turn();
  }
  current_rotation_speed = target_rotation_speed;
  if (direction != 0) {
    set_rotation(current_rotation_speed);
  }
  rotation_angle_f = get_angle_f();
  rotation_angle_b = get_angle_b();
  

  // target_rotation_speed = get_target_rotation_speed();
  // if (rotation_is_changed(current_rotation_speed, target_rotation_speed)) {
  //   led_no_turn();
  // }
  // current_rotation_speed = target_rotation_speed;
  // set_rotation(current_rotation_speed);


  // Serial.printf("target_motor_speed: %d\n2\n", target_motor_speed);
  // Serial.printf("current_motor_speed: %d\n", current_motor_speed);

  // Serial.printf("target_rotation_speed: %d\n", target_rotation_speed);
  // Serial.printf("current_rotation_speed: %d\n", current_rotation_speed);


  update_led();


  delay(input_delay);
}

int get_angle_f() {
  return sensor_f.rawAngle() - starting_angle_f;
}

int get_angle_b() {
  return sensor_b.rawAngle() - starting_angle_b;
}

void reset_wheels_angle() {
  rotation_angle_b = get_angle_b();
  rotation_angle_f = get_angle_f();

  Serial.println("centering forward");
  reset_wheels_angle_f();
  Serial.println("centering backward");
  reset_wheels_angle_b();
}


void reset_wheels_angle_f() {
  Serial.printf("rotation_angle_f: %d\n", rotation_angle_f);
  if (rotation_angle_f > deadzone_rotation_angle + 10) {
    Serial.println("to left");
    turn_left_f();
    while (rotation_angle_f > deadzone_rotation_angle) {
      Serial.printf("angle_f: %d\n", get_angle_f());
        rotation_angle_f = get_angle_f();
    }
  } else if (rotation_angle_f < -deadzone_rotation_angle - 10) {
    Serial.println("to right");
    turn_right_f();
    while (rotation_angle_f < -deadzone_rotation_angle) {
      Serial.printf("angle_f: %d\n", get_angle_f());
        rotation_angle_f = get_angle_f();
    }
  }
  no_turn_f();
}

void reset_wheels_angle_b() {
  Serial.printf("rotation_angle_b: %d\n", rotation_angle_b);
  if (rotation_angle_b > deadzone_rotation_angle + 10) {
    Serial.println("to left");
    turn_left_b();
    while (rotation_angle_b > deadzone_rotation_angle) {
        Serial.printf("angle_b: %d\n", get_angle_b());
        rotation_angle_b = get_angle_b();
    }
  } else if (rotation_angle_b < -deadzone_rotation_angle - 10) {
    Serial.println("to right");
    turn_right_b();
    while (rotation_angle_b < -deadzone_rotation_angle) {
        Serial.printf("angle_b: %d\n", get_angle_b());
        rotation_angle_b = get_angle_b();
    }
  }
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
  return (value > 0) && (target_value <= 0) || (value < 0) && (target_value >= 0);
}

void set_rotation(int rotation_speed) {
  if (inverse_wheel_rotation) {
    rotation_angle_b = -rotation_angle_b;
  }

  // bool rotation_is_calibrated = abs(rotation_angle_f - rotation_angle_b) < max_angle_difference;
  bool rotation_is_calibrated = true;

  if (rotation_is_calibrated) {
    if (rotation_speed == 0 || rotation_angle_is_too_high(rotation_speed, rotation_angle_f, rotation_angle_b)) {
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

bool rotation_angle_is_too_high(int rotation_speed, int rotation_angle_1, int rotation_angle_2) {
  if (rotation_speed > 0) {
    return max(rotation_angle_1, rotation_angle_2) >= max_rotation_angle;
  }
  if (rotation_speed < 0) {
    return min(rotation_angle_1, rotation_angle_2) <= -max_rotation_angle;
  }
  return false;
}

void calibrate_rotation(int rotation_speed) {
  if (rotation_speed > 0) {
    if (rotation_angle_f > rotation_angle_b) {
      if (inverse_wheel_rotation) {
        turn_left_b();
        return;
      }
      turn_right_b();
      return;
    }
    turn_right_f();
    return;
  }
  if (rotation_angle_f < rotation_angle_b) {
    if (inverse_wheel_rotation) {
      turn_right_b();
      return;
    }
    turn_left_b();
    return;
  }
  turn_left_f();
}

void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 12; ChannelNum++)
  {
    Serial.printf("%d: %d", ChannelNum, controller.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println("");
}