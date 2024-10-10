// Constants for engine control pins
const int ENGINE1_BRAKE_PIN = 12;
const int ENGINE2_BRAKE_PIN = 13;
const int ENGINE1_SPEED_CONTROL_PIN = 23;
const int ENGINE2_SPEED_CONTROL_PIN = 22;
const int ENGINE1_DIRECTION_CONTROL_PIN = 26;
const int ENGINE2_DIRECTION_CONTROL_PIN = 27;

// Speed constants
constexpr int MAX_SPEED = 255;
int current_motor_speed = 120; // Initial speed

void setup() {
  pinMode(ENGINE1_SPEED_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE2_SPEED_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE1_BRAKE_PIN, OUTPUT);
  pinMode(ENGINE2_BRAKE_PIN, OUTPUT);
  pinMode(ENGINE1_DIRECTION_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE2_DIRECTION_CONTROL_PIN, OUTPUT);
  
  Serial.begin(9600); // Initialize serial communication
  Serial.println("Initialization complete");
}

void loop() {
  move_forward();
  delay(1000); // Move forward for 1 second
  move_backward();
  delay(1000); // Move backward for 1 second
}

void move_forward() {
  analogWrite(ENGINE1_SPEED_CONTROL_PIN, current_motor_speed);
  analogWrite(ENGINE2_SPEED_CONTROL_PIN, current_motor_speed);
  digitalWrite(ENGINE1_DIRECTION_CONTROL_PIN, HIGH);
  digitalWrite(ENGINE2_DIRECTION_CONTROL_PIN, LOW);
  Serial.println("Moving forward");
}

void move_backward() {
  analogWrite(ENGINE1_SPEED_CONTROL_PIN, current_motor_speed);
  analogWrite(ENGINE2_SPEED_CONTROL_PIN, current_motor_speed);
  digitalWrite(ENGINE1_DIRECTION_CONTROL_PIN, LOW);
  digitalWrite(ENGINE2_DIRECTION_CONTROL_PIN, HIGH);
  Serial.println("Moving backward");
}