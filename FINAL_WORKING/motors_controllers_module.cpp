#include "motors_controllers_module.h"

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