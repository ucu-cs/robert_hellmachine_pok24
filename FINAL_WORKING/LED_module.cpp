#include <Adafruit_NeoPixel.h>
#include "LED_module.h"

// LED strip pins and variables
constexpr uint32_t NUM_PIXELS_HALF = NUM_PIXELS / 2;

Adafruit_NeoPixel strip_f(NUM_PIXELS, PIN_LED_STRIP_F, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_b(NUM_PIXELS, PIN_LED_STRIP_B, NEO_GRB + NEO_KHZ800);

uint32_t LED_RED = strip_f.Color(255, 0, 0);
uint32_t LED_LIGHT_RED = strip_f.Color(50, 0, 0);
uint32_t LED_ORANGE = strip_f.Color(255, 50, 0);
uint32_t LED_WHITE = strip_f.Color(255, 255, 255);
uint32_t LED_NO_COLOR = strip_f.Color(0, 0, 0);

int animation_delay = 30;
int restarting_animation_delay = 300;
int animation_timer = 0;

int current_num_turning_pixels = 0;
    

void update_led() {
  led_moving();
  if (braking) {
    led_braking();
  }

  if (rotation_angle_f > 200) {
    led_turn_right();
  } else if (rotation_angle_f < -200) {
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