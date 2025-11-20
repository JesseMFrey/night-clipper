// vim: set ts=2 sw=2 sts=2 et :

#include <Adafruit_NeoPixel.h>
#include <AlfredoCRSF.h>
#include <limits.h>
#include <math.h>

enum{FLIGHT_MODE_GLIDE=0,FLIGHT_MODE_LAUNCH};
enum{LED_MODE_STARTUP=0,LED_MODE_0,LED_MODE_1,LED_MODE_2};

typedef uint32_t PIXEL_COLOR;

#define HUE_25PCT     ((0xFFFF)/4)
#define HUE_50PCT     ((0xFFFF)/2)
#define HUE_75PCT     ((0xFFFF) - HUE_25PCT)

#define LED_PIN        A0
#define REG_PIN        A1

#define NC_RED_PIN    A2
#define NC_GREEN_PIN  A3
#define NC_BLUE_PIN   SDA

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

#define LED_TASK_PRIORITY     2


#define LED_TASK_STACK_SIZE   10000
StackType_t LED_task_stack[LED_TASK_STACK_SIZE];

StaticTask_t LED_task;

enum Led_Stat {LED_STAT_STARTUP=0,LED_ERROR_CRSF, LED_STAT_GOOD, LED_STAT_LINK_LOST, LED_STAT_WIFI};

// alias for serial port redifine as needed
#define Serial_elrs Serial1

Led_Stat board_led_state = LED_STAT_STARTUP;
AlfredoCRSF rc_link = AlfredoCRSF();


//swap red and green on a color
PIXEL_COLOR color_swap(PIXEL_COLOR color)
{
  PIXEL_COLOR temp = color & ~(RED_MASK|GREEN_MASK);

  temp |= ((color & RED_MASK) >> RED_SHIFT) << GREEN_SHIFT;
  temp |= ((color & GREEN_MASK) >> GREEN_SHIFT) << RED_SHIFT;
  return temp;
}


void LED_task_func(void *p);

void setup()
{
  xTaskCreateStaticPinnedToCore(LED_task_func,
                                "LED",
                                LED_TASK_STACK_SIZE,
                                NULL,
                                LED_TASK_PRIORITY,
                                LED_task_stack,
                                &LED_task,
                                1
                                );

  Serial.begin();

  Serial_elrs.begin(420000);
  if(!Serial_elrs)
  {
    board_led_state = LED_ERROR_CRSF;
    while (!Serial);
    Serial.println("Failed to init CRSF");
    for(;;);
  }
  rc_link.begin(Serial_elrs);
}

unsigned long last_flash = 0;
unsigned long last_rc_update = 0;

int map_led_mode(int mode_val)
{
  int mode_idx;
  if(mode_val < 1250)
  {
    mode_idx = 0;
  }
  else if(mode_val < 1750)
  {
    mode_idx = 1;
  }
  else
  {
    mode_idx =2;
  }
  //TODO: apply map table?
  return LED_MODE_0 + mode_idx;
}

struct {
  uint16_t hue;
  uint8_t sat, brt;
  uint8_t flight_mode;
  uint8_t led_mode;
} pattern_info;

uint16_t map_rc(uint16_t val, uint16_t max_val)
{
  const uint16_t min_rc=1000, max_rc=2000;
  float tmp;
  //limit value to minimum
  if(val < min_rc)
  {
    val = min_rc;
  }
  if(val > max_rc)
  {
    val = max_rc;
  }
  val -= min_rc;

  tmp = val;
  tmp *= max_val/((float)(max_rc-min_rc));
  return round(tmp);
}

void loop()
{
  rc_link.update();

  if (millis() - last_rc_update >= 20)
  {
    last_rc_update = millis();
    pattern_info.hue = map_rc(rc_link.getChannel(3), 0xFFFF);
    pattern_info.brt = map_rc(rc_link.getChannel(4), 0xFF);
    pattern_info.sat = map_rc(rc_link.getChannel(6), 0xFF);
    pattern_info.flight_mode = (rc_link.getChannel(7) > 1500)?FLIGHT_MODE_GLIDE:FLIGHT_MODE_LAUNCH;
    pattern_info.led_mode = map_led_mode(rc_link.getChannel(8));
  }
  if(rc_link.isLinkUp())
  {
    board_led_state = LED_STAT_GOOD;
  }
  else
  {
    board_led_state = LED_STAT_LINK_LOST;
  }
}

// LED task update period in ms
#define LED_UPDATE_PERIOD 10

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

  //setup NC pins
  /*
  digitalWrite(NC_RED_PIN, LOW);
  digitalWrite(NC_GREEN_PIN, LOW);
  digitalWrite(NC_BLUE_PIN, HIGH);
  pinMode(NC_RED_PIN, OUTPUT);
  pinMode(NC_GREEN_PIN, OUTPUT);
  pinMode(NC_BLUE_PIN, OUTPUT);
  */

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
      Serial.print("Startup LED ");
      Serial.println(led_num);
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
