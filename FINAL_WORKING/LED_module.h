#ifndef LED_MODULE_H
#define LED_MODULE_H

#include <Adafruit_NeoPixel.h>

#include <Arduino.h>
#include "config.h"

// To .ino
extern Adafruit_NeoPixel strip_f;
extern Adafruit_NeoPixel strip_b;

void update_led();
void all_leds_show();
void led_moving();
void led_braking();
void led_no_turn();
void update_animation();
void led_turn_left();
void led_turn_right();
void no_led();


// From .ino
extern int get_target_rotation_speed();

extern bool braking;
extern int turn;
extern int current_rotation_speed;
extern int input_delay;
extern int rotation_angle_f;

#endif