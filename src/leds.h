#ifndef __LEDS_H
#define __LEDS_H

#include "pindefs.h"

typedef uint32_t PIXEL_COLOR;

#define HUE_MAX       (0xFFFF)
#define HUE_25PCT     (HUE_MAX/4)
#define HUE_50PCT     (HUE_MAX/2)
#define HUE_75PCT     (HUE_MAX - HUE_25PCT)
#define RED_MASK      (0x00FF0000)
#define GREEN_MASK    (0x0000FF00)
#define BLUE_MASK     (0x000000FF)

#define RED_SHIFT     (16)
#define GREEN_SHIFT   (8)
#define BLUE_SHIFT    (0)

#define NC_PWM_RESOLUTION     (8)
#define NC_PWM_FREQ           (500)

#define WING_TIP_LEDS (28)
#define PARALLEL_LEDS (44)
#define WING_BOT_LEDS (PARALLEL_LEDS + WING_TIP_LEDS)
#define WING_TOP_LEDS (50)
#define TAIL_LEDS (20)

#define AFT_LED_IDX_L (WING_BOT_LEDS + WING_TOP_LEDS + 1)
#define AFT_LED_IDX_R (WING_BOT_LEDS + WING_TOP_LEDS + 2 + 2*TAIL_LEDS + 1)

#define TOP_LED_IDX_L (WING_BOT_LEDS + WING_TOP_LEDS)
#define TOP_LED_IDX_R (WING_BOT_LEDS + WING_TOP_LEDS + 2 + 2*TAIL_LEDS)

#define LEFT_WING_START_IDX     (0)
#define LEFT_TAIL_START_IDX     (WING_BOT_LEDS + 2 + WING_TOP_LEDS)
#define RIGHT_WING_START_IDX    (WING_BOT_LEDS + 2*TAIL_LEDS + 4 + 2*WING_TOP_LEDS)
#define RIGHT_TAIL_START_IDX    (LEFT_TAIL_START_IDX + TAIL_LEDS)
#define LEFT_WING_TOP_START_IDX (WING_BOT_LEDS)
#define RIGHT_WING_TOP_START_IDX (WING_BOT_LEDS + 2*TAIL_LEDS + 4 + WING_TOP_LEDS)

// 2 strings (left and right) 72 LEDS parallel to body, 28 to wingtip
// 2 tail strings (one on either side) of 20 LEDs each
// 4 discrete LEDS on the aft end
#define NUM_LEDS (2 * (WING_BOT_LEDS + WING_TOP_LEDS + TAIL_LEDS) + 4)

//swap red and green on a color
extern inline PIXEL_COLOR color_swap(PIXEL_COLOR color)
{
  PIXEL_COLOR temp = color & ~(RED_MASK|GREEN_MASK);

  temp |= ((color & RED_MASK) >> RED_SHIFT) << GREEN_SHIFT;
  temp |= ((color & GREEN_MASK) >> GREEN_SHIFT) << RED_SHIFT;
  return temp;
}

enum LED_SEGMENT {LED_SEG_INVALID=0, LED_SEG_FRONT_LEFT, LED_SEG_WING_BOT_LEFT,
    LED_SEG_WING_TOP_LEFT, LED_SEG_TOP_LEFT, LED_SEG_AFT_LEFT,
    LED_SEG_VERT_LEFT, LED_SEG_VERT_RIGHT, LED_SEG_TOP_RIGHT, LED_SEG_AFT_RIGHT,
    LED_SEG_WING_TOP_RIGHT, LED_SEG_WING_BOT_RIGHT, LED_SEG_FRONT_RIGHT};

extern inline LED_SEGMENT index_to_segment(int index)
{
      if(index < PARALLEL_LEDS)
      {
        return LED_SEG_FRONT_LEFT;
      }
      else if(index < WING_BOT_LEDS)
      {
        return LED_SEG_WING_BOT_LEFT;
      }
      else if(index < WING_BOT_LEDS + WING_TOP_LEDS)
      {
        return LED_SEG_WING_TOP_LEFT;
      }
      else if(index == TOP_LED_IDX_L)
      {
        return LED_SEG_TOP_LEFT;
      }
      else if(index == AFT_LED_IDX_L)
      {
        return LED_SEG_AFT_LEFT;
      }
      else if(index < LEFT_TAIL_START_IDX + TAIL_LEDS)
      {
        return LED_SEG_VERT_LEFT;
      }
      else if(index < LEFT_TAIL_START_IDX + 2 * TAIL_LEDS)
      {
        return LED_SEG_VERT_RIGHT;
      }
      else if( index == TOP_LED_IDX_R)
      {
        return LED_SEG_TOP_RIGHT;
      }
      else if(index == AFT_LED_IDX_R)
      {
        return LED_SEG_AFT_RIGHT;
      }
      else if(index < RIGHT_WING_START_IDX)
      {
        return LED_SEG_WING_TOP_RIGHT;
      }
      else if(index < RIGHT_WING_START_IDX + WING_TIP_LEDS)
      {
        return LED_SEG_WING_BOT_RIGHT;
      }
      else if(index < NUM_LEDS)
      {
        return LED_SEG_FRONT_RIGHT;
      }
      else
      {
        return LED_SEG_INVALID;
      }
}

//return contiguous index for strip.
//index starts at the front/inner end
extern inline int contiguous_index(int index)
{
  switch(index_to_segment(index))
  {
    case LED_SEG_FRONT_LEFT:
    case LED_SEG_WING_BOT_LEFT:
      return index;
    case LED_SEG_FRONT_RIGHT:
    case LED_SEG_WING_BOT_RIGHT:
      return NUM_LEDS - index -1;
    case LED_SEG_VERT_LEFT:
      return LEFT_TAIL_START_IDX + TAIL_LEDS - index - 1;
    case LED_SEG_VERT_RIGHT:
      return index - RIGHT_TAIL_START_IDX;
    case LED_SEG_WING_TOP_LEFT:
      return LEFT_WING_TOP_START_IDX + WING_TOP_LEDS - index - 1;
    case LED_SEG_WING_TOP_RIGHT:
      return index - RIGHT_WING_TOP_START_IDX;

    // single LEDs
    case LED_SEG_TOP_LEFT:
    case LED_SEG_TOP_RIGHT:
    case LED_SEG_AFT_LEFT:
    case LED_SEG_AFT_RIGHT:
      return 0;
  }
  //Invalid
  return -1;
}

//return contiguous length for strip.
extern inline int contiguous_length(int index)
{
  switch(index_to_segment(index))
  {
    case LED_SEG_FRONT_LEFT:
    case LED_SEG_WING_BOT_LEFT:
    case LED_SEG_FRONT_RIGHT:
    case LED_SEG_WING_BOT_RIGHT:
      return WING_BOT_LEDS;
    case LED_SEG_VERT_LEFT:
    case LED_SEG_VERT_RIGHT:
      return TAIL_LEDS;
    case LED_SEG_WING_TOP_LEFT:
    case LED_SEG_WING_TOP_RIGHT:
      return WING_TOP_LEDS;

    // single LEDs
    case LED_SEG_TOP_LEFT:
    case LED_SEG_TOP_RIGHT:
    case LED_SEG_AFT_LEFT:
    case LED_SEG_AFT_RIGHT:
      return 1;
  }
  //Invalid
  return 0;
}

extern inline void set_nosecone(PIXEL_COLOR color)
{
    ledcWrite(NC_RED_PIN, (color & RED_MASK) >> RED_SHIFT);
    ledcWrite(NC_GREEN_PIN, (color & GREEN_MASK) >> GREEN_SHIFT);
    ledcWrite(NC_BLUE_PIN, (color & BLUE_MASK) >> BLUE_SHIFT);
}

#endif
