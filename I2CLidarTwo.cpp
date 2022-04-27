/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  v3HP/v3HP_I2C

  this test shows multiple ways to use this thing
------------------------------------------------------------------------------*/

#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>

LIDARLite_v3HP lidarLite;

#define FAST_I2C

enum rangeType_T
{
    RANGE_NONE,
    RANGE_SINGLE,
    RANGE_CONTINUOUS,
    RANGE_TIMER
};

void setup()
{
    // Initialize Arduino serial port (for display of ASCII output to PC)
    Serial.begin(115200);

    // Initialize Arduino I2C (for communication to LidarLite)
    Wire.begin();
    #ifdef FAST_I2C
        #if ARDUINO >= 157
            Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
        #else
            TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
        #endif
    #endif

    // Configure the LidarLite internal parameters so as to lend itself to
    // various modes of operation by altering 'configure' input integer to
    // anything in the range of 0 to 5.
    lidarLite.configure(0);
}


void loop()
{
    uint16_t distance;
    uint8_t  newDistance = 0;
    uint8_t  c;
    rangeType_T rangeMode = RANGE_NONE;

    PrintMenu();

    // Continuous loop
    while (1)
    {
        // Each time through the loop, look for a serial input character
        if (Serial.available() > 0)
        {
            c = (uint8_t) Serial.read();
          
            switch (c)
            {
                case 'S':
                case 's':
                    rangeMode = RANGE_SINGLE;
                    break;

                case 'C':
                case 'c':
                    rangeMode = RANGE_CONTINUOUS;
                    break;

                case 'T':
                case 't':
                    rangeMode = RANGE_TIMER;
                    break;

                case '.':
                    rangeMode = RANGE_NONE;
                    break;

                case 'D':
                case 'd':
                    rangeMode = RANGE_NONE;
                    dumpCorrelationRecord();
                    break;

                case 'P':
                case 'p':
                    rangeMode = RANGE_NONE;
                    peakStackExample();
                    break;

                case 0x0D:
                case 0x0A:
                    break;

                default:
                    rangeMode = RANGE_NONE;
                    PrintMenu();
                    break;
            }
        }

        switch (rangeMode)
        {
            case RANGE_NONE:
                newDistance = 0;
                break;

            case RANGE_SINGLE:
                newDistance = distanceSingle(&distance);
                break;

            case RANGE_CONTINUOUS:
                newDistance = distanceContinuous(&distance);
                break;

            case RANGE_TIMER:
                delay(250); // 4 Hz
                newDistance = distanceFast(&distance);
                break;

            default:
                newDistance = 0;
                break;
        }

        // When there is new distance data, print it to the serial port
        if (newDistance)
        {
            Serial.println(distance);
        }

        // Single measurements print once and then stop
        if (rangeMode == RANGE_SINGLE)
        {
            rangeMode = RANGE_NONE;
        }
    }
}

void PrintMenu(void)
{
    Serial.println("=====================================");
    Serial.println("== Type a single character command ==");
    Serial.println("=====================================");
    Serial.println(" S - Single Measurement");
    Serial.println(" C - Continuous Measurement");
    Serial.println(" T - Timed Measurement");
    Serial.println(" . - Stop Measurement");
    Serial.println(" D - Dump Correlation Record");
    Serial.println(" P - Peak Stack Example");
}

//---------------------------------------------------------------------
// Read Single Distance Measurement
//---------------------------------------------------------------------
uint8_t distanceSingle(uint16_t * distance)
{
    lidarLite.waitForBusy();
    lidarLite.takeRange();
    lidarLite.waitForBusy();
    *distance = lidarLite.readDistance();

    return 1;
}

//---------------------------------------------------------------------
// Read Continuous Distance Measurements
//---------------------------------------------------------------------
uint8_t distanceContinuous(uint16_t * distance)
{
    uint8_t newDistance = 0;

    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
    if (lidarLite.getBusyFlag() == 0)
    {
        // Trigger the next range measurement
        lidarLite.takeRange();

        // Read new distance data from device registers
        *distance = myLidarLite.readDistance();

        // Report to calling function that we have new data
        newDistance = 1;
    }

    return newDistance;
}

//---------------------------------------------------------------------
// Read Distance Measurement
//---------------------------------------------------------------------
uint8_t distanceFast(uint16_t * distance)
{
    lidarLite.waitForBusy();
    lidarLite.takeRange();
  
    *distance = lidarLite.readDistance();

    return 1;
}

//---------------------------------------------------------------------
// Print the correlation record for analysis
//---------------------------------------------------------------------
void dumpCorrelationRecord()
{
    lidarLite.correlationRecordToSerial(256);
}

//---------------------------------------------------------------------
// Print peaks and calculated distances from the peak stack
//---------------------------------------------------------------------
void peakStackExample()
{
    int16_t   peakArray[8];
    int16_t   distArray[8];
    uint8_t   i;

    // - Read the Peak Stack.
    // - Peaks and calculated distances are returned in local arrays.
    lidarLite.peakStackRead(peakArray, distArray);

    // Print peaks and calculated distances to the serial port.
    Serial.println();
    Serial.println("IDX PEAK DIST");
    for (i=0 ; i<8 ; i++)
    {
        Serial.print(i);
        Serial.print("   ");
        Serial.print(peakArray[i]);
        Serial.print("  ");
        Serial.print(distArray[i]);
        Serial.println();
    }
}
