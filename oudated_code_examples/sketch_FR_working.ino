#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

#define PIN_RX 18
#define PIN_TX 17

#define PIN_NEO_PIXEL 20
#define NUM_PIXELS 10

Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

// Set up a new Serial object
HardwareSerial crsfSerial(2);
AlfredoCRSF crsf;


// Constants for engine control pins
constexpr int ALL_ENGINE_BRAKE_PIN = 45;
constexpr int ALL_ENGINE_SPEED_CONTROL_PIN = 35;
constexpr int ALL_ENGINE_DIRECTION_CONTROL_PIN = 37;

constexpr int SWITCH_LEFT_PIN = 11;
constexpr int SWITCH_RIGHT_PIN = 12;

constexpr int ROTATION_SPEED_LEFT_PIN_F = 4;
constexpr int ROTATION_SPEED_RIGHT_PIN_F = 6;

constexpr int ROTATION_SPEED_LEFT_PIN_B = 15;
constexpr int ROTATION_SPEED_RIGHT_PIN_B = 8;

constexpr int ROTATION_CHANNEL = 1;
constexpr int SPEED_CHANNEL = 3;
constexpr int ROTATION_TYPE_CHANNEL = 5;
constexpr int DIRECTION_CHANNEL = 6;
constexpr int BRAKE_CHANNEL = 7;

// Speed constants
int acceleration_step = 5;
int input_delay = 20;

int DEADZONE_MOTOR_SPEED = acceleration_step + 3;
int DEADZONE_ROTATION_SPEED = acceleration_step + 7;

constexpr int STOP_MOTOR_SPEED = 0;
constexpr int MAX_MOTOR_SPEED = 100;
constexpr int BACKWARD_MAX_MOTOR_SPEED = - MAX_MOTOR_SPEED;
constexpr int STOP_ROTATION_SPEED = 0;
constexpr int MAX_ROTATION_SPEED = 100;
constexpr int BACKWARD_MAX_ROTATION_SPEED = - MAX_ROTATION_SPEED;
constexpr int ROTATION_OFFSET = 120;

int current_motor_speed = 0;
int target_motor_speed = 0;
int current_rotation_speed = 0;
int target_rotation_speed = 0;

int direction = 1;
int turn = 1;
bool inverse_wheel_rotation = true;
int forward_reversed_data = 1500;
int rotation_data = 1500;

int direction_data = 990;

void setup() {
  pinMode(ALL_ENGINE_SPEED_CONTROL_PIN, OUTPUT);
  pinMode(ALL_ENGINE_BRAKE_PIN, OUTPUT);
  pinMode(ALL_ENGINE_DIRECTION_CONTROL_PIN, OUTPUT);

  pinMode(SWITCH_LEFT_PIN, INPUT);
  pinMode(SWITCH_RIGHT_PIN, INPUT);

  pinMode(ROTATION_SPEED_LEFT_PIN_F, OUTPUT);
  pinMode(ROTATION_SPEED_RIGHT_PIN_F, OUTPUT);

  pinMode(ROTATION_SPEED_LEFT_PIN_B, OUTPUT);
  pinMode(ROTATION_SPEED_RIGHT_PIN_B, OUTPUT);

  NeoPixel.begin(); 

  Serial.begin(115200); // Initialize serial communication

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(crsfSerial);
  Serial.println("Initialization complete");
}

void loop() {

  NeoPixel.clear();

  crsf.update();
  printChannels();

  Serial.printf("code number: %d\n", 4);
  
  if (crsf.getChannel(SPEED_CHANNEL) < 800) {
    delay(input_delay);
    return;
  }

  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
    NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 255, 0));
    NeoPixel.show();
  }

  forward_reversed_data = min(max(crsf.getChannel(SPEED_CHANNEL), 990), 2010);

  direction_data = crsf.getChannel(DIRECTION_CHANNEL);
  // direction_data = 990;

  if (direction_data <= 1100) {
    // Serial.printf("direction: %d", direction);
    direction = 1;
    turn = 1;
  } else if (direction_data >= 1900) {
    // Serial.printf("direction: %d", direction);
    direction = -1;
    turn = 1;
  } else {
    // Serial.printf("direction: %d", direction);
    direction = 0;
    turn = 0;
  }
  
  target_motor_speed = (forward_reversed_data - 990) / 10 * direction;

  // Serial.printf("target_motor_speed: %d, direction: %d\n", target_motor_speed, direction);

  reaching_target_motor_speed();

  Serial.printf("target_motor_speed: %d\n2\n", target_motor_speed);
  Serial.printf("current_motor_speed: %d\n", current_motor_speed);



  rotation_data = min(max(crsf.getChannel(ROTATION_CHANNEL), 990), 2010);
  target_rotation_speed = (rotation_data - 1500) / 5 * turn;

  inverse_wheel_rotation = crsf.getChannel(ROTATION_TYPE_CHANNEL) <= 1400;

  reaching_target_rotation_speed();

  Serial.printf("target_rotation_speed: %d\n", target_rotation_speed);
  Serial.printf("current_rotation_speed: %d\n", current_rotation_speed);


  delay(input_delay);
}


void reaching_target_motor_speed() {
  if (abs(target_motor_speed) < DEADZONE_MOTOR_SPEED) {
    target_motor_speed = STOP_MOTOR_SPEED;
  }
  else if (current_motor_speed < target_motor_speed) {
    current_motor_speed = min(current_motor_speed + acceleration_step, MAX_MOTOR_SPEED);
  }
  else {
    current_motor_speed = max(current_motor_speed - acceleration_step, BACKWARD_MAX_MOTOR_SPEED);
  }
  if (current_motor_speed < 0) {
    digitalWrite(ALL_ENGINE_DIRECTION_CONTROL_PIN, LOW);
  }
  else {
    digitalWrite(ALL_ENGINE_DIRECTION_CONTROL_PIN, HIGH);
  }
  analogWrite(ALL_ENGINE_SPEED_CONTROL_PIN, abs(current_motor_speed));
}


void reaching_target_rotation_speed() {
  if (abs(target_rotation_speed) < DEADZONE_ROTATION_SPEED) {
    current_rotation_speed = STOP_ROTATION_SPEED;
  }
  else if (current_rotation_speed < target_rotation_speed) {
    current_rotation_speed = min(current_rotation_speed + acceleration_step, MAX_ROTATION_SPEED);
  }
  else {
    current_rotation_speed = max(current_rotation_speed - acceleration_step, BACKWARD_MAX_ROTATION_SPEED);
  }
  if (current_rotation_speed == 0) {
    no_turn();
  }
  else if (current_rotation_speed > 0) {
    turn_right(ROTATION_OFFSET + current_rotation_speed);
  }
  else {
    turn_left(ROTATION_OFFSET + abs(current_rotation_speed));
  }
}


// void decelerating(int speed_start, int speed_end, int step) {

//   for (int speed = speed_start; speed >= speed_end; speed -= step) {
//     analogWrite(ALL_ENGINE_SPEED_CONTROL_PIN, speed);
//     delay(acceleration_delay);
//     Serial.printf("Deceletaring: %d\n", speed);
//   }

// }

// void accelerating(int speed_start, int speed_end, int step) {

//   for (int speed = speed_start; speed <= speed_end; speed += step) {
//     analogWrite(ALL_ENGINE_SPEED_CONTROL_PIN, speed);
//     delay(acceleration_delay);
//     Serial.printf("Acceletaring: %d\n", speed);
//   }

// }

// void move_forward() {

//   digitalWrite(ALL_ENGINE_DIRECTION_CONTROL_PIN, HIGH);
//   accelerating(starting_motor_speed, moving_motor_speed, acceleration_step);
//   Serial.println("Moving forward");

// }

// void move_backward() {
  
//   digitalWrite(ALL_ENGINE_DIRECTION_CONTROL_PIN, LOW);
//   accelerating(starting_motor_speed, moving_motor_speed, acceleration_step);
//   Serial.println("Moving backward");

// }


void no_turn(){
  // analogWrite(ROTATION_SPEED_LEFT_PIN_B, STOP_ROTATION_SPEED);
  // analogWrite(ROTATION_SPEED_RIGHT_PIN_B, STOP_ROTATION_SPEED);

  // analogWrite(ROTATION_SPEED_LEFT_PIN_F, STOP_ROTATION_SPEED);
  // analogWrite(ROTATION_SPEED_RIGHT_PIN_F, STOP_ROTATION_SPEED);


  digitalWrite(ROTATION_SPEED_LEFT_PIN_B, 0);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_B, 0);

  digitalWrite(ROTATION_SPEED_LEFT_PIN_F, 0);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_F, 0);
}


void turn_left(const int rotation_speed){
  // Serial.println("Left turn");
  // analogWrite(ROTATION_SPEED_LEFT_PIN_F, rotation_speed);
  // analogWrite(ROTATION_SPEED_RIGHT_PIN_F, STOP_ROTATION_SPEED);

  // analogWrite(ROTATION_SPEED_LEFT_PIN_B, STOP_ROTATION_SPEED);
  // analogWrite(ROTATION_SPEED_RIGHT_PIN_B, rotation_speed);

  // analogWrite(ROTATION_SPEED_LEFT_PIN_F, ROTATION_SPEED);
  // digitalWrite(ROTATION_SPEED_RIGHT_PIN_F, 0);

  // digitalWrite(ROTATION_SPEED_LEFT_PIN_B, 0);
  // analogWrite(ROTATION_SPEED_RIGHT_PIN_B, ROTATION_SPEED);

  digitalWrite(ROTATION_SPEED_LEFT_PIN_F, 1);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_F, 0);

  if (inverse_wheel_rotation) {
    digitalWrite(ROTATION_SPEED_LEFT_PIN_B, 0);
    digitalWrite(ROTATION_SPEED_RIGHT_PIN_B, 1);
  } else {
    digitalWrite(ROTATION_SPEED_LEFT_PIN_B, 1);
    digitalWrite(ROTATION_SPEED_RIGHT_PIN_B, 0);
  }

}


void turn_right(const int rotation_speed){

  // Serial.println("right turn");

  // analogWrite(ROTATION_SPEED_LEFT_PIN_F, STOP_ROTATION_SPEED);
  // analogWrite(ROTATION_SPEED_RIGHT_PIN_F, rotation_speed);

  // analogWrite(ROTATION_SPEED_LEFT_PIN_B, rotation_speed);
  // analogWrite(ROTATION_SPEED_RIGHT_PIN_B, STOP_ROTATION_SPEED);


  // digitalWrite(ROTATION_SPEED_LEFT_PIN_F, 0);
  // analogWrite(ROTATION_SPEED_RIGHT_PIN_F, ROTATION_SPEED);

  // analogWrite(ROTATION_SPEED_LEFT_PIN_B, ROTATION_SPEED);
  // digitalWrite(ROTATION_SPEED_RIGHT_PIN_B, 0);

  digitalWrite(ROTATION_SPEED_LEFT_PIN_F, 0);
  digitalWrite(ROTATION_SPEED_RIGHT_PIN_F, 1);

  if (inverse_wheel_rotation) {
    digitalWrite(ROTATION_SPEED_LEFT_PIN_B, 1);
    digitalWrite(ROTATION_SPEED_RIGHT_PIN_B, 0);
  } else {
    digitalWrite(ROTATION_SPEED_LEFT_PIN_B, 0);
    digitalWrite(ROTATION_SPEED_RIGHT_PIN_B, 1);
  }

}


void block_wheels() {
  digitalWrite(ALL_ENGINE_BRAKE_PIN, HIGH);
}


//Use crsf.getChannel(x) to get us channel values (1-16).
void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 12; ChannelNum++)
  {
    Serial.printf("%d: %d", ChannelNum, crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}
