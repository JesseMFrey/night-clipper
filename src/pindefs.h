#ifndef __PINDEFS_H
#define __PINDEFS_H

#define LED_PIN        A0
#define REG_PIN        A1

#define GPS_SERIAL_PIN SCL
//dummy pin for GPS serial
#define GPS_DUMMY_TX_PIN MOSI

#define NC_RED_PIN    A2
#define NC_GREEN_PIN  A3
#define NC_BLUE_PIN   SDA

// aliases for serial ports redifine as needed
#define Serial_elrs Serial1
#define Serial_gps  Serial2

#endif
