// vim: set ts=2 sw=2 sts=2 et :

#include <Adafruit_NeoPixel.h>
#include <AlfredoCRSF.h>

enum{FLIGHT_MODE_GLIDE=0,FLIGHT_MODE_LAUNCH};
enum{LED_MODE_STARTUP=0,LED_MODE_0,LED_MODE_1,LED_MODE_2};

typedef uint32_t PIXEL_COLOR;

#define LED_PIN        A0
#define REG_PIN        A1

#define NC_RED_PIN    A2
#define NC_GREEN_PIN  A3
#define NC_BLUE_PIN   SDA


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
#define NUM_LEDS (2 * (WING_BOT_LEDS + TAIL_LEDS) + 4)

#define LED_TASK_PRIORITY     2


#define LED_TASK_STACK_SIZE   10000
StackType_t LED_task_stack[LED_TASK_STACK_SIZE];

StaticTask_t LED_task;

enum Led_Stat {LED_BLINK_OFF=0, LED_ERROR_CRSF, LED_STAT_GOOD, LED_STAT_LINK_LOST};

Led_Stat board_led_state = LED_BLINK_OFF;
AlfredoCRSF rc_link = AlfredoCRSF();


//swap red and green on a color
PIXEL_COLOR color_swap(PIXEL_COLOR color)
{
  PIXEL_COLOR temp = color & 0xFFFF00;

  temp |= (color & 0xFF0000) >> 8;
  temp |= (color & 0xFF00) << 8;
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

  Serial1.begin(420000);
  if(!Serial1)
  {
    board_led_state = LED_ERROR_CRSF;
    while (!Serial);
    Serial.println("Failed to init CRSF");
    for(;;);
  }
  rc_link.begin(Serial1);
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
  uint8_t hue, sat, brt;
  uint8_t flight_mode;
  uint8_t led_mode;
} pattern_info;

void loop()
{
  rc_link.update();

  if (millis() - last_rc_update >= 20)
  {
    last_rc_update = millis();
    pattern_info.hue = map(rc_link.getChannel(3), 1000, 2000, 0, 255);
    pattern_info.brt = map(rc_link.getChannel(4), 1000, 2000, 0, 255);
    pattern_info.sat = map(rc_link.getChannel(6), 1000, 2000, 0, 255);
    pattern_info.flight_mode = (rc_link.getChannel(7) > 1500)?FLIGHT_MODE_GLIDE:FLIGHT_MODE_LAUNCH;
    pattern_info.led_mode = map_led_mode(rc_link.getChannel(8));
  }

  if (millis() - last_flash >= 500)
  {
    last_flash = millis();
    if(board_led_state == LED_BLINK_OFF)
    {
      if(rc_link.isLinkUp())
      {
        board_led_state = LED_STAT_GOOD;
        Serial.println("Blink good!");
      }
      else
      {
        board_led_state = LED_STAT_LINK_LOST;
        Serial.println("Blink link lost");
      }
    }
    else
    {
      board_led_state = LED_BLINK_OFF;
    }
  }
}

void LED_task_func(void *p)
{
  //add one LED for the board LED
  static Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN);
  static Adafruit_NeoPixel board_led(1, PIN_NEOPIXEL);
  PIXEL_COLOR color;
  unsigned char led_hue = 0;
  unsigned int led_num=0, led_brt=0;
  TickType_t LED_LastWakeTime;
  const TickType_t LED_delay = 10 / portTICK_PERIOD_MS;
  BaseType_t WasDelayed;

  //setup NC pins
  digitalWrite(NC_RED_PIN, LOW);
  digitalWrite(NC_GREEN_PIN, LOW);
  digitalWrite(NC_BLUE_PIN, LOW);
  pinMode(NC_RED_PIN, OUTPUT);
  pinMode(NC_GREEN_PIN, OUTPUT);
  pinMode(NC_BLUE_PIN, OUTPUT);

  pinMode(REG_PIN, OUTPUT);
  digitalWrite(REG_PIN, HIGH); //turn regulator on

  leds.begin();
  board_led.begin();
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
      led_hue += 4;
      Serial.print("Startup LED ");
      Serial.println(led_num);
    }
    //this is here to make sure we don't write off the end of the array
    if(led_brt)
    {
      //set LED brightness
      //leds[WING_BOT_LEDS - led_num] = CHSV(led_hue, 255, led_brt);
      //leds[RIGHT_WING_START_IDX + led_num -1] = CHSV(led_hue, 255, led_brt);
      leds.setPixelColor(WING_BOT_LEDS - led_num, leds.ColorHSV(led_hue, 255, led_brt));
      leds.setPixelColor(RIGHT_WING_START_IDX + led_num -1, leds.ColorHSV(led_hue, 255, led_brt));
    }
    //show LEDs
    leds.show();
    // Wait for the next cycle.
    WasDelayed = xTaskDelayUntil( &LED_LastWakeTime, LED_delay);
  }
  for(;;)
  {
    //fill_solid(leds, NUM_LEDS, CHSV(pattern_info.hue, pattern_info.sat, pattern_info.brt));
    switch(board_led_state)
    {
    case LED_BLINK_OFF:
      board_led.setPixelColor(0, 0, 0, 0);
      break;
    case LED_ERROR_CRSF:
      board_led.setPixelColor(0, 255, 0, 0);
      break;
    case LED_STAT_GOOD:
      board_led.setPixelColor(0, 0, 255, 0);
      break;
    case LED_STAT_LINK_LOST:
      board_led.setPixelColor(0, 255, 255, 0);
      break;
    }
    //show LEDs
    board_led.show();
    leds.show();
    // Wait for the next cycle.
    WasDelayed = xTaskDelayUntil( &LED_LastWakeTime, LED_delay);
  }
}
