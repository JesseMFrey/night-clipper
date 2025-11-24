#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "led_task.h"
#include "leds.h"
#include "pindefs.h"
#include "pattern.h"

StackType_t LED_task_stack[LED_TASK_STACK_SIZE];

StaticTask_t LED_task;

Led_Stat board_led_state = LED_STAT_STARTUP;

pattern_state_t  pattern_info;

void LED_task_func(void *p)
{
  //add one LED for the board LED
  static Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN);
  static Adafruit_NeoPixel board_led(1, PIN_NEOPIXEL);
  PIXEL_COLOR color;
  PIXEL_COLOR front_color, right_color, left_color, rear_color;
  uint16_t led_hue = 0;
  unsigned int led_num=0, led_brt=0;
  TickType_t LED_LastWakeTime;
  const TickType_t LED_delay = LED_UPDATE_PERIOD / portTICK_PERIOD_MS;
  BaseType_t WasDelayed;
  int board_led_count=0;
  int board_led_step=0;

  //setup NC pins for PWM
  ledcAttach(NC_RED_PIN, NC_PWM_FREQ, NC_PWM_RESOLUTION);
  ledcWrite(NC_RED_PIN, 0);

  ledcAttach(NC_GREEN_PIN, NC_PWM_FREQ, NC_PWM_RESOLUTION);
  ledcWrite(NC_GREEN_PIN, 0);

  ledcAttach(NC_BLUE_PIN, NC_PWM_FREQ, NC_PWM_RESOLUTION);
  ledcWrite(NC_BLUE_PIN, 0);

  pinMode(REG_PIN, OUTPUT);
  digitalWrite(REG_PIN, HIGH); //turn regulator on

  leds.begin();
  board_led.setBrightness(255);
  board_led.begin();

  leds.setBrightness(255);
  leds.show();
  board_led.show();


  // Initialise the xLastWakeTime variable with the current time.
  LED_LastWakeTime = xTaskGetTickCount ();

  while(led_num < WING_BOT_LEDS)
  {
    led_brt += 30;
    if(led_brt > 255)
    {
      led_brt = 0;
      led_num += 1;
      led_hue += 4*256;
    }
    //this is here to make sure we don't write off the end of the array
    if(led_brt)
    {
      //set LED brightness
      //leds[WING_BOT_LEDS - led_num] = CHSV(led_hue, 255, led_brt);
      //leds[RIGHT_WING_START_IDX + led_num -1] = CHSV(led_hue, 255, led_brt);
      color = leds.gamma32(leds.ColorHSV(led_hue, 255, led_brt));
      leds.setPixelColor(WING_BOT_LEDS - led_num - 1, color);
      leds.setPixelColor(RIGHT_WING_START_IDX + led_num, color);
    }
    //show LEDs
    leds.show();
    // Wait for the next cycle.
    WasDelayed = xTaskDelayUntil( &LED_LastWakeTime, LED_delay);
  }
  for(;;)
  {
    color = leds.gamma32(leds.ColorHSV(pattern_info.hue, pattern_info.sat, pattern_info.brt));

    front_color = leds.gamma32(leds.ColorHSV(pattern_info.hue, pattern_info.sat, pattern_info.brt));
    right_color = leds.gamma32(leds.ColorHSV(pattern_info.hue + HUE_25PCT, pattern_info.sat, pattern_info.brt));
    left_color = leds.gamma32(leds.ColorHSV(pattern_info.hue + HUE_50PCT, pattern_info.sat, pattern_info.brt));
    rear_color = leds.gamma32(leds.ColorHSV(pattern_info.hue + HUE_75PCT, pattern_info.sat, pattern_info.brt));

    for(int i=0;i<NUM_LEDS;i++)
    {
      if(i < PARALLEL_LEDS)
      {
        leds.setPixelColor(i, front_color);
      }
      else if(i < WING_BOT_LEDS)
      {
        leds.setPixelColor(i, left_color);
      }
      else if(i < WING_BOT_LEDS + WING_TOP_LEDS)
      {
        leds.setPixelColor(i, left_color);
      }
      else if(i == TOP_LED_IDX_L)
      {
        leds.setPixelColor(i, color_swap(left_color));
      }
      else if(i == AFT_LED_IDX_L)
      {
        leds.setPixelColor(i, color_swap(rear_color));
      }
      else if(i < TOP_LED_IDX_R)
      {
        leds.setPixelColor(i, rear_color);
      }
      else if( i == TOP_LED_IDX_R)
      {
        leds.setPixelColor(i, color_swap(right_color));
      }
      else if( i == AFT_LED_IDX_R)
      {
        leds.setPixelColor(i, color_swap(rear_color));
      }
      else if(i < RIGHT_WING_START_IDX)
      {
        leds.setPixelColor(i, right_color);
      }
      else if(i < RIGHT_WING_START_IDX + WING_TIP_LEDS)
      {
        leds.setPixelColor(i, right_color);
      }
      else
      {
        leds.setPixelColor(i, front_color);
      }
    }
    //set nosecone color
    ledcWrite(NC_RED_PIN, (front_color & RED_MASK) >> RED_SHIFT);
    ledcWrite(NC_GREEN_PIN, (front_color & GREEN_MASK) >> GREEN_SHIFT);
    ledcWrite(NC_BLUE_PIN, (front_color & BLUE_MASK) >> BLUE_SHIFT);

    if(board_led_count <= 0)
    {
      //set default period 1s
      board_led_count = 1000/LED_UPDATE_PERIOD;
      switch(board_led_state)
      {
      case LED_STAT_STARTUP:
        board_led_count = 100/LED_UPDATE_PERIOD;
        board_led.setPixelColor(0, 0, 0, 255);
        break;
      case LED_ERROR_CRSF:
        board_led.setPixelColor(0, 255, 0, 0);
        break;
      case LED_ERROR_GPS:
        board_led_count = 50/LED_UPDATE_PERIOD;
        board_led.setPixelColor(0, 255, 0, 0);
        break;
      case LED_STAT_GOOD:
        board_led.setPixelColor(0, 0, 255, 0);
        break;
      case LED_STAT_WIFI:
        board_led_count = 50/LED_UPDATE_PERIOD;
        board_led.setPixelColor(0, 0, 255, 0);
        break;
      case LED_STAT_LINK_LOST:
        board_led.setPixelColor(0, 255, 255, 0);
        break;
      }
      if(board_led_step)
      {
        board_led.setPixelColor(0, 0, 0, 0);
      }
      board_led_step = !board_led_step;
      board_led.show();
    }
    else
    {
      board_led_count -= 1;
    }
    //show LEDs
    leds.show();
    // Wait for the next cycle.
    WasDelayed = xTaskDelayUntil( &LED_LastWakeTime, LED_delay);
  }
}
