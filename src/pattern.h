#ifndef __PATTERN_H
#define __PATTERN_H

typedef struct {
  uint16_t hue;
  uint8_t sat, brt;
  uint8_t flight_mode;
  uint8_t led_mode;
} pattern_state_t;

extern pattern_state_t pattern_info;
#endif
