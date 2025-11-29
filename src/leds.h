#ifndef __LEDS_H
#define __LEDS_H

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

#endif
