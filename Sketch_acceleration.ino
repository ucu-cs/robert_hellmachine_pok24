// Constants for engine control pins
const int ALL_ENGINE_BRAKE_PIN = 45;
const int ALL_ENGINE_SPEED_CONTROL_PIN = 35;
const int ALL_ENGINE_DIRECTION_CONTROL_PIN = 37;

// Speed constants
constexpr int MAX_SPEED = 255;
int starting_motor_speed = 0; // Initial speed
int moving_motor_speed = 50;
int acceleration_steep = 5;
int acceleration_delay = 50;

void setup() {
  pinMode(ALL_ENGINE_SPEED_CONTROL_PIN, OUTPUT);
  pinMode(ALL_ENGINE_BRAKE_PIN, OUTPUT);
  pinMode(ALL_ENGINE_DIRECTION_CONTROL_PIN, OUTPUT);
  
  Serial.begin(9600); // Initialize serial communication
  Serial.println("Initialization complete");
}

void loop() {
  move_forward();
  delay(3000);
  
  decelerating(moving_motor_speed, starting_motor_speed, acceleration_steep);

  delay(100);

  move_backward();
  delay(3000);

  decelerating(moving_motor_speed, starting_motor_speed, acceleration_steep);

  delay(100);
}

void decelerating(int speed_start, int speed_end, int step) {

  for (int speed = speed_start; speed >= speed_end; speed -= step) {
    analogWrite(ALL_ENGINE_SPEED_CONTROL_PIN, speed);/
    delay(acceleration_delay);
    Serial.printf("Deceletaring: %d\n", speed);
  }

}

void accelerating(int speed_start, int speed_end, int step) {

  for (int speed = speed_start; speed <= speed_end; speed += step) {
    analogWrite(ALL_ENGINE_SPEED_CONTROL_PIN, speed);
    delay(acceleration_delay);
    Serial.printf("Acceletaring: %d\n", speed);
  }

}

void move_forward() {

  digitalWrite(ALL_ENGINE_DIRECTION_CONTROL_PIN, HIGH);
  accelerating(starting_motor_speed, moving_motor_speed, acceleration_steep);
  Serial.println("Moving forward");

}

void move_backward() {
  
  digitalWrite(ALL_ENGINE_DIRECTION_CONTROL_PIN, LOW);
  accelerating(starting_motor_speed, moving_motor_speed, acceleration_steep);
  Serial.println("Moving backward");

}
