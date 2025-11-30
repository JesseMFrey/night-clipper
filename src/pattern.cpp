
#include <cstdint>
#include "leds.h"
#include "pattern.h"

Pattern::Pattern() :
  led_mode(LED_MODE_STARTUP), flight_mode(FLIGHT_MODE_NONE)
{
}

void Pattern::begin(Adafruit_NeoPixel *leds)
{
  this->leds = leds;
}

void Pattern::led_hue_segments(int led_idx)
{
  switch(index_to_segment(led_idx))
  {
    case LED_SEG_FRONT_LEFT:
    case LED_SEG_FRONT_RIGHT:
      leds->setPixelColor(led_idx, front_color);
      break;
    case LED_SEG_VERT_LEFT:
    case LED_SEG_VERT_RIGHT:
      leds->setPixelColor(led_idx, rear_color);
      break;
    case LED_SEG_WING_BOT_LEFT:
    case LED_SEG_WING_TOP_LEFT:
      leds->setPixelColor(led_idx, left_color);
      break;
    case LED_SEG_WING_BOT_RIGHT:
    case LED_SEG_WING_TOP_RIGHT:
      leds->setPixelColor(led_idx, right_color);
      break;
    // These LEDs have a different color order and need to be fixed
    case LED_SEG_TOP_LEFT:
      leds->setPixelColor(led_idx, color_swap(left_color));
      break;
    case LED_SEG_TOP_RIGHT:
      leds->setPixelColor(led_idx, color_swap(right_color));
      break;
    case LED_SEG_AFT_LEFT:
    case LED_SEG_AFT_RIGHT:
      leds->setPixelColor(led_idx, color_swap(rear_color));
      break;
  }
}

void Pattern::led_rainbows(int led_idx)
{

  if( contiguous_length(led_idx) <=1)
  {
    leds->setPixelColor(led_idx, rear_color);
  }
  else
  {
    PIXEL_COLOR c;
    uint16_t hue;
    hue = (HUE_MAX * contiguous_index(led_idx))/contiguous_length(led_idx);
    hue += this->hue;
    c = leds->ColorHSV(hue, this->sat, this->brt);
    c = leds->gamma32(c);
    leds->setPixelColor(led_idx, c);
  }
}

void Pattern::led_wave(int led_idx)
{
  uint16_t hue;
  PIXEL_COLOR c;
  if( contiguous_length(led_idx) <=1)
  {
    hue = HUE_50PCT - position * 256;
    c = leds->ColorHSV(hue, this->sat, this->brt);
    c = leds->gamma32(c);
    leds->setPixelColor(led_idx, c);
  }
  else
  {
    hue = (HUE_MAX * contiguous_index(led_idx))/contiguous_length(led_idx);
    hue -= position * 256;
    c = leds->ColorHSV(hue, this->sat, this->brt);
    c = leds->gamma32(c);
    leds->setPixelColor(led_idx, c);
  }
}


void Pattern::update_LEDs(void)
{
  position += 1;
  for(int i=0;i<NUM_LEDS;i++)
  {
    switch(led_mode)
    {
      case LED_MODE_HUE_SEG:
        led_hue_segments(i);
        break;
      case LED_MODE_RAINBOWS:
        led_rainbows(i);
        break;
      case LED_MODE_WAVE:
        led_wave(i);
        break;
    }
  }
  leds->show();
  if(led_mode == LED_MODE_WAVE)
  {
    uint16_t hue;
    PIXEL_COLOR c;
    hue = -1 *  position * 256;
    c = leds->ColorHSV(hue, this->sat, this->brt);
    c = leds->gamma32(c);
    set_nosecone(c);
  }
  else
  {
    set_nosecone(front_color);
  }
}


void Pattern::update_colors(void)
{
  PIXEL_COLOR c;

  c = leds->ColorHSV(this->hue, this->sat, this->brt);
  this->front_color = leds->gamma32(c);

  c = leds->ColorHSV(this->hue + HUE_25PCT, this->sat, this->brt);
  this->right_color = leds->gamma32(c);

  c = leds->ColorHSV(this->hue + HUE_50PCT, this->sat, this->brt);
  this->left_color = leds->gamma32(c);

  c = leds->ColorHSV(this->hue + HUE_75PCT, this->sat, this->brt);
  this->rear_color = leds->gamma32(c);

}

void Pattern::update_params(uint16_t hue, uint8_t sat, uint8_t brt,
             FlightMode_t fm, int mode_val)
{
  this->hue = hue;
  this->sat = sat;
  this->brt = brt;
  this->flight_mode = fm;

  if(mode_val < 1250)
  {
    this->led_mode =  this->mode_table[0];
  }
  else if(mode_val < 1750)
  {
    this->led_mode =  this->mode_table[1];
  }
  else
  {
    this->led_mode =  this->mode_table[2];
  }
}

