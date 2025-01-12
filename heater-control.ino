#include <Arduino.h>
#include <DHT.h>
#include <string.h>
#include <stddef.h>
#include <OneButton.h>
#include <PID_v1.h>

#define PIN_DHT 10
#define PIN_TEMP_DOWN 6
#define PIN_TEMP_UP 8
#define TEMP_MIN 55u
#define TEMP_MAX 80u
#define PIN_DIMMER_OUT 9 //dimmer output
#define PIN_DIMMER_ZC 2 //zero cross

#define PIN_CYCLE 3
#define PIN_HEAT 4

typedef unsigned long ulong;

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#define LIMIT(min, in, max) (MAX((min), MIN((max),(in))))

static DHT dht(PIN_DHT, DHT11);
static OneButton buttonTempDown;
static OneButton buttonTempUp;

double temperature = 0.0f, dimmerOutput = 0.0f, tempSetPoint = 72.0f;
double Kp=2, Ki=0, Kd=0;
PID myPID(&temperature, &dimmerOutput, &tempSetPoint, Kp, Ki, Kd, DIRECT);

static float zcMicrosAverage = 0; // average microseconds between zero cross events
static ulong zcMicrosLast = 0; // microseconds between the last zero cross event
static ulong scopeMillis = 0; // millis of last scope execution
static long zcMicrosMax = 0;
static ulong zcMicrosMin = 20000;
static int acCycle = 0; // count of the ac cycle. 0-9


float average(float in, float last, float samples)
{
    return last + (in - last) / samples;
}

void tempDownClick()
{
    tempSetPoint = LIMIT(TEMP_MIN, tempSetPoint - 1.0, TEMP_MAX);
}

void tempUpClick()
{
    tempSetPoint = LIMIT(TEMP_MIN, tempSetPoint + 1.0, TEMP_MAX);
}

void setup() {
    dht.begin();

    // The built-in serial class was left as c++
    Serial.begin(115200);

    buttonTempDown.setup(PIN_TEMP_DOWN, INPUT_PULLUP,true);
    buttonTempDown.attachClick(tempDownClick);

    buttonTempUp.setup(PIN_TEMP_UP, INPUT_PULLUP,true);     
    buttonTempUp.attachClick(tempUpClick);

    pinMode(PIN_DIMMER_OUT, OUTPUT);
    pinMode(PIN_CYCLE, OUTPUT);
    pinMode(PIN_HEAT, OUTPUT);

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0.0f, 10.0f);
}

void loop()
{
    buttonTempDown.tick();
    buttonTempUp.tick();

    ulong zcMicros; //micros since last zc event
    zcMicros = micros() - zcMicrosLast;

    // Look for zc 7.5ms after the last zc
    // Should occur at 8ms
    if (zcMicros > 7500)
    {
        if (1 == digitalRead(PIN_DIMMER_ZC))
        {
            // track zc
            zcMicrosMin = min(zcMicros, zcMicrosMin);
            zcMicrosMax = max(zcMicros, zcMicrosMax);
            zcMicrosAverage = average((float)zcMicros, zcMicrosAverage, 10);
            zcMicrosLast = micros();

            // count the cycles
            acCycle++;
            if (acCycle > 9)
            {
                acCycle = 0;
            }

            // get temp and calculate pid at the zc so the intervals are equal
            temperature = dht.readTemperature(true,false);
            myPID.Compute();

            // Enable the pin output if the dimmer output (0-10) is greater than the current cycle number
            // This will cause the dimmerOutput to turn on at cycle 0
            if ((int)dimmerOutput > acCycle)
            {
                digitalWrite(PIN_DIMMER_OUT, HIGH);
            }

            digitalWrite(PIN_CYCLE, acCycle % 2 == 0);

            // Scope output
            if ((scopeMillis - millis()) > 100)
            {
                Serial.print("sp:");
                Serial.print(tempSetPoint,1);
                Serial.print(", t:");
                Serial.print(dht.readTemperature(true),1);
                Serial.print(", cycle:");
                Serial.print(acCycle);
                Serial.print(", dimmerOutput:");
                Serial.println(dimmerOutput);
            }
        }

        // Disable the output if the output will be off on the next cycle
        // The triac will disable the output at the zero cross
        if ((int)dimmerOutput <= acCycle)
        {
            digitalWrite(PIN_DIMMER_OUT, LOW);
        }
    }
}
