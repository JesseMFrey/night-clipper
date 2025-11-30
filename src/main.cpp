// vim: set ts=2 sw=2 sts=2 et :

#include <Adafruit_NeoPixel.h>
#include <AlfredoCRSF.h>
#include <limits.h>
#include <math.h>
#include "MSP_GPS.h"
#include "leds.h"
#include "pindefs.h"
#include "led_task.h"
#include "pattern.h"

AlfredoCRSF rc_link = AlfredoCRSF();
Pattern LED_pattern = Pattern();

void setup()
{
  xTaskCreateStaticPinnedToCore(LED_task_func,
                                "LED",
                                LED_TASK_STACK_SIZE,
                                &LED_pattern,
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
  //start serial 2 for MSP GPS from EZID
  // RX on A1
  Serial_gps.begin(115200, SERIAL_8N1, GPS_SERIAL_PIN, GPS_DUMMY_TX_PIN);
  if(!Serial_gps)
  {
    board_led_state = LED_ERROR_GPS;
    while (!Serial);
    Serial.println("Failed to init MSP GPS UART");
    for(;;);
  }

}

unsigned long last_flash = 0;
unsigned long last_rc_update = 0;
unsigned long last_gps_update = 0;

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


void handle_GPS_packet(mspPacket_t *gpsPacket)
{
  static crsf_sensor_gps_t gps_to_send = { 0 };

  // we got a GPS packet, update GPS packet time
  last_gps_update = millis();

  // translate from MSP to CRSF
  populate_gps_packet(&gps_to_send, (mspGPSdat_t*)gpsPacket->inBuf);
  //send packet
  rc_link.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_GPS, &gps_to_send, sizeof(gps_to_send));
}


void loop()
{
  rc_link.update();

  if (millis() - last_rc_update >= 20)
  {
    last_rc_update = millis();
    LED_pattern.update_params(
        map_rc(rc_link.getChannel(3), 0xFFFF),
        map_rc(rc_link.getChannel(6), 0xFF),
        map_rc(rc_link.getChannel(4), 0xFF),
        (rc_link.getChannel(7) > 1500)?FLIGHT_MODE_GLIDE:FLIGHT_MODE_LAUNCH,
        rc_link.getChannel(8)
      );
  }
  if(rc_link.isLinkUp())
  {
    board_led_state = LED_STAT_GOOD;
  }
  else
  {
    board_led_state = LED_STAT_LINK_LOST;
  }

  if(Serial_gps.available())
  {
    char c = Serial_gps.read();
    mspGPS_ProcessReceivedPacketData(&gps_stat, c);
    if(gps_stat.packetState == MSP_COMMAND_RECEIVED)
    {
      //Packet recived, see what it is
      if(gps_stat.cmdMSP == MSP2_SENSOR_GPS)
      {
        handle_GPS_packet(&gps_stat);
      }
      // packet processed, back to idle state
      gps_stat.packetState = MSP_IDLE;
    }
  }
}
