#ifndef MOTORS_CONTROLLERS_MODULE_H
#define MOTORS_CONTROLLERS_MODULE_H

#include <Arduino.h>
#include "config.h"

// To .ino
void no_turn_b();
void no_turn_f();
void no_turn();
void turn_left_b();
void turn_right_b();
void turn_left_f();
void turn_right_f();
void turn_left();
void turn_right();
void block_wheels();

// From .ino
extern bool inverse_wheel_rotation;

#endif