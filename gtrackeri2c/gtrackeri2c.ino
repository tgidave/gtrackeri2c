///////////////////////////////////////////////////////////////////////////////
//
// gtrackeri2c.ino
//
// This code will track G forces.
//
// The gtracker program will track G forces that occur for the first user
// defined period of time and then send the results to another device through
// the i2c port to be reported back to the user.
//
// Author
//
// Uncle Dave
//
//  License
//  Unknown (Talk to Cy)
//
//  HISTORY
//
//  v1.0   - First release
//
///////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <DateTime.h>
#include <Wire.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <RTCZero.h> // Include RTC library - make sure it's installed!
#include <Adafruit_NeoPixel.h>
#include <ArduinoLowPower.h>
//#include <Adafruit_SleepyDog.h>
#include "gtrackeri2c.h"

#define DEBUG

#ifdef DEBUG
  #define DEBUG_SERIAL Serial
#endif  // DEBUG

#define ALARM_HOUR    0
#define ALARM_MINUTE  5
#define ALARM_SECOND  0

i2cData gData;

bool H3LIS331Down  = false;

bool reportResults  = false;
bool reportDone     = false;

bool firstTime  = true;

Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

RTCZero rtc; // Create an RTC object

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

volatile char msgBuff[MESSAGE_BUFF_SIZE];

volatile char *msgPtr = (char *)&msgBuff; 

void reqISR(void);

const char hexchars[] = "0123456789ABCDEF";

void printHexChar(uint8_t x) {
  Serial.print(hexchars[(x >> 4)]);
  Serial.print(hexchars[(x & 0x0f)]);
}

void setup(void) {

  int i;

  gData.maxX = 0;
  gData.maxY = 0;
  gData.maxZ = 0;
  gData.maxMag = 0;

#ifdef DEBUG
  DEBUG_SERIAL.begin(115200);
  delay(1000);     // Wait for the serial port to become ready.
  DEBUG_SERIAL.println("gtracker code");
#endif

//  REPORT_SERIAL.begin(9600);

  rtc.begin();

  pixels.begin();  // initialize the pixel

#ifdef DEBUG
    DEBUG_SERIAL.println("Starting up the H3LIS331.");
#endif  // DEBUG    H3LIS331Down = true;

  if (!lis.begin_I2C()) {   // change this to 0x19 for alternative i2c address

#ifdef DEBUG
    DEBUG_SERIAL.println("H3LIS331 not found!!!");
#endif  // DEBUG    H3LIS331Down = true;

  } else {

#ifdef DEBUG
    DEBUG_SERIAL.println("H3LIS331 found!");
#endif  // DEBUG

    //lis.setRange(H3LIS331_RANGE_100_G);   // 100, 200, or 400 G!
    //lis.setRange(H3LIS331_RANGE_200_G);   // 100, 200, or 400 G!
    lis.setRange(H3LIS331_RANGE_400_G);   // 100, 200, or 400 G!

#ifdef DEBUG
    DEBUG_SERIAL.print("Range set to: ");

    switch (lis.getRange()) {
    case H3LIS331_RANGE_100_G:
      DEBUG_SERIAL.println("100 g");
      break;
    case H3LIS331_RANGE_200_G:
      DEBUG_SERIAL.println("200 g");
      break;
    case H3LIS331_RANGE_400_G:
      DEBUG_SERIAL.println("400 g");
      break;
    }
#endif  // DEBUG

    lis.setDataRate(LIS331_DATARATE_1000_HZ);

#ifdef DEBUG
    DEBUG_SERIAL.print("Data rate set to: ");

    switch (lis.getDataRate()) {
    case LIS331_DATARATE_POWERDOWN:
      DEBUG_SERIAL.println("Powered Down");
      break;
    case LIS331_DATARATE_50_HZ:
      DEBUG_SERIAL.println("50 Hz");
      break;
    case LIS331_DATARATE_100_HZ:
      DEBUG_SERIAL.println("100 Hz");
      break;
    case LIS331_DATARATE_400_HZ:
      DEBUG_SERIAL.println("400 Hz");
      break;
    case LIS331_DATARATE_1000_HZ:
      DEBUG_SERIAL.println("1000 Hz");
      break;
    case LIS331_DATARATE_LOWPOWER_0_5_HZ:
      DEBUG_SERIAL.println("0.5 Hz Low Power");
      break;
    case LIS331_DATARATE_LOWPOWER_1_HZ:
      DEBUG_SERIAL.println("1 Hz Low Power");
      break;
    case LIS331_DATARATE_LOWPOWER_2_HZ:
      DEBUG_SERIAL.println("2 Hz Low Power");
      break;
    case LIS331_DATARATE_LOWPOWER_5_HZ:
      DEBUG_SERIAL.println("5 Hz Low Power");
      break;
    case LIS331_DATARATE_LOWPOWER_10_HZ:
      DEBUG_SERIAL.println("10 Hz Low Power");
      break;
    }
#endif  // DEBUG

  }

  //!!! notice The year is limited to 2000-2099
  rtc.begin();

  rtc.setTime(0, 0, 0); // Then set the time
  rtc.setDate(0, 0, 0); // And the date

#ifdef DEBUG
  DEBUG_SERIAL.println("RTC Started!");

  DEBUG_SERIAL.println("adjusted time!");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print(rtc.getYear(), DEC);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(rtc.getMonth(), DEC);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(rtc.getDay(), DEC);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(rtc.getHours(), DEC);
  DEBUG_SERIAL.print(':');
  DEBUG_SERIAL.print(rtc.getMinutes(), DEC);
  DEBUG_SERIAL.print(':');
  DEBUG_SERIAL.print(rtc.getSeconds(), DEC);
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();
#endif  // DEBUG

  rtc.attachInterrupt(alarmMatch); // callback while alarm is match

  rtc.setAlarmHours(ALARM_HOUR);
  rtc.setAlarmMinutes(ALARM_MINUTE);
  rtc.setAlarmSeconds(ALARM_SECOND);

  rtc.enableAlarm(rtc.MATCH_HHMMSS); // match Every Day

}

// Interrupts are disabled during this ISR.
void alarmMatch(void) {

  reportResults = true;

}
// Interrupts are enabled on return from this ISR.

void loop() {

  /* Get a new sensor event, normalized */
  sensors_event_t event;

  int i;

  uint8_t *gDataPtr = (uint8_t *)&gData;

  float tempX;
  float tempY;
  float tempZ;
  float tempMag;

  bool localReportResults;

  noInterrupts();
  localReportResults = reportResults;
  interrupts();

  if (localReportResults == false) {
    if (!H3LIS331Down) {

      lis.getEvent(&event);

      if (abs(gData.maxX) < abs(tempX = (event.acceleration.x / SENSORS_GRAVITY_STANDARD))) {
        gData.maxX = tempX;
      }

      if (abs(gData.maxY) < abs(tempY = (event.acceleration.y / SENSORS_GRAVITY_STANDARD))) {
        gData.maxY = tempY;
      }

      if (abs(gData.maxZ) < abs(tempZ = (event.acceleration.z / SENSORS_GRAVITY_STANDARD))) {
        gData.maxZ = tempZ;
      }

      tempMag = sqrt((tempX * tempX) + (tempY * tempY) + (tempZ * tempZ));

      if (tempMag > gData.maxMag) {
        gData.maxMag  = tempMag;
        gData.maxMagX = tempX;
        gData.maxMagY = tempY;
        gData.maxMagZ = tempZ;
      }
    }
  } else { // localReportResults == true

    // This code only need to run once.
    if (firstTime == true) {

#ifdef DEBUG
      DEBUG_SERIAL.println("reportResults is true");
      // set the first pixel #0 to red
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      // and write the data
      pixels.show();
#endif  // DEBUG

      lis.setDataRate(LIS331_DATARATE_POWERDOWN);

      Wire.end();

      // Initialize the peripheral I2C code and assign it's address.
      Wire.begin(GTRACKER_PERIPHERAL_I2C_ADDR);
      Wire.onRequest(reqISR);

      firstTime = false;
    }


    while (1) {
      if (reportDone == true) {
#ifdef DEBUG
        DEBUG_SERIAL.println("\nreportDone is true...\n");
        pixels.clear();
        pixels.show();

        i = 0;
        gDataPtr = (uint8_t *)&gData;
        while (i < i2cDataSize) {
          printHexChar(*(gDataPtr + i));
          ++i;
        }

        sprintf((char *)&msgBuff,
                "maxX = %.2f maxY = %.2f maxZ = %.2f g\nmaxMagX = %.2f maxMagY = %.2f maxMagZ = %.2f maxMag = %.2f g\n\r",
                gData.maxX, gData.maxY, gData.maxZ, gData.maxMagX, gData.maxMagY, gData.maxMagZ, gData.maxMag);

        DEBUG_SERIAL.print("\n");
        DEBUG_SERIAL.print((char *)msgBuff);
        DEBUG_SERIAL.print("\n");
#endif // DEBUG


        delay(5000);

        while(1) { 
          if (reportDone == true) {
            // Put the samd21 into deep sleep mode.
            //Watchdog.enable();
            LowPower.deepSleep();
          }
        }
      }
      yield();
    }
  }
}

void reqISR(void) {

  if (reportDone == false) {

    Wire.write((uint8_t *)&gData, i2cDataSize);
    reportDone = true;

  } 
}

