/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  v3HP/v3HP_PWM
------------------------------------------------------------------------------*/

#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v3HP.h"

LIDARLite_v3HP myLidarLite;

#define MonitorPin    3
#define TriggerPin    2

uint32_t distance;
uint32_t startTime;
uint32_t endTime;
bool     newDistance = false;
bool     measuring   = false;

// this is made so that you can use I2C within the PWM mode
//#define USE_I2C

void setup()
{
    Serial.begin(115200);

    #ifdef USE_I2C
        Wire.begin();
        Wire.setClock(400000UL); 
    #endif

    pinMode(MonitorPin, INPUT);
    pinMode(TriggerPin, OUTPUT);
    digitalWrite(TriggerPin, LOW); // Set trigger LOW for continuous read

    startTime = micros();
    endTime   = startTime;
}

void loop()
{
    if (digitalRead(MonitorPin))
    {
        if (measuring == false)
        {
            startTime   = micros();
            measuring   = true;
        }
    }
    else
    {
        if (measuring == true)
        {
            endTime     = micros();
            measuring   = false;
            newDistance = true;
        }
    }
  
    if (newDistance == true)
    {
        distance = (endTime - startTime) / 10; // 10usec = 1 cm of distance
        Serial.println(distance); 

        #ifdef USE_I2C
            uint8_t  signalStrength = 0;
            myLidarLite.read(0x0e, &signalStrength, 1);
            Serial.print("SS = ");
            Serial.println(signalStrength, DEC);
            Serial.println();
        #endif

        newDistance = false;
    }
}
