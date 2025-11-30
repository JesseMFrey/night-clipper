#ifndef __PATTERN_H
#define __PATTERN_H

#include "leds.h"
#include <Adafruit_NeoPixel.h>

enum FlightMode_t {FLIGHT_MODE_NONE=0, FLIGHT_MODE_GLIDE, FLIGHT_MODE_LAUNCH};
enum LED_mode_t {LED_MODE_STARTUP=0, LED_MODE_HUE_SEG, LED_MODE_RAINBOWS,
                 LED_MODE_WAVE};


class Pattern
{
public:
  uint16_t hue;
  uint8_t sat, brt;
  FlightMode_t flight_mode;
  LED_mode_t led_mode;

  Pattern();
  void begin(Adafruit_NeoPixel *leds);
  void update_LEDs(void);
  void update_colors(void);
  void update_params(uint16_t hue, uint8_t sat, uint8_t brt, FlightMode_t fm,
                    int mode_val);
private:
  uint16_t position;
  LED_mode_t mode_table[3] = {LED_MODE_HUE_SEG, LED_MODE_RAINBOWS,
                              LED_MODE_WAVE};
  PIXEL_COLOR front_color, right_color, left_color, rear_color;
  Adafruit_NeoPixel *leds;
  void led_hue_segments(int led_idx);
  void led_rainbows(int led_idx);
  void led_wave(int led_idx);
};

#endif
