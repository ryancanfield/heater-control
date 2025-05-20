#include <Arduino.h>
#include <DHT.h>
#include <string.h>
#include <stddef.h>
#include <OneButton.h>
#include <PID_v1.h>
#include <math.h>

#define PIN_DHT 10
#define PIN_TEMP_DOWN 6
#define PIN_TEMP_UP 8
#define TEMP_MIN 55u
#define TEMP_MAX 85u
#define PIN_DIMMER_OUT 9 //dimmer output
#define PIN_DIMMER_ZC 2 //zero cross

#define PIN_CYCLE 3
#define PIN_HEAT 4

typedef unsigned long ulong;

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#define LIMIT(min, in, max) (MAX((min), MIN((max),(in))))

static DHT dht(PIN_DHT, DHT22);
static OneButton buttonTempDown;
static OneButton buttonTempUp;

bool enable = false;
double temperature = 0.0f, dimmerOutput = 0.0f, tempSetPoint = 72.0f;
int dimmerOutputInt = 0;
double Kp=4.5, Ki=0.007, Kd=0;
//kp 10 had an oscilation of 800 seconds
//zeigler nicholds pid = 
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

void tempUpLongPress()
{
    enable = true;
    digitalWrite(PIN_HEAT,1);
}

void tempDownLongPress()
{
    enable = false;
    digitalWrite(PIN_HEAT,0);
}

void setup() {
    dht.begin();

    // The built-in serial class was left as c++
    Serial.begin(115200);

    buttonTempDown.setup(PIN_TEMP_DOWN, INPUT_PULLUP,true);
    buttonTempDown.attachClick(tempDownClick);
    buttonTempDown.attachLongPressStart(tempDownLongPress);

    buttonTempUp.setup(PIN_TEMP_UP, INPUT_PULLUP,true);     
    buttonTempUp.attachClick(tempUpClick);
    buttonTempUp.attachLongPressStart(tempUpLongPress);

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
            double t = dht.readTemperature(true,false);
            if (!isnan(t))
            {
                temperature = t;
            }
            // set error values if temp is invalid
            else
            {
                temperature = 0;
            }

            // Run PID
            if (!isnan(t) & enable)
            {
                myPID.Compute();
            }
            else
            {
                dimmerOutput = 0;
            }
            dimmerOutputInt = round(dimmerOutput);

            // Enable the pin output if the dimmer output (0-10) is greater than the current cycle number
            // This will cause the dimmerOutput to turn on at cycle 0
            if (dimmerOutputInt > acCycle and enable)
            {
                digitalWrite(PIN_DIMMER_OUT, HIGH);
            }

            // Scope output
            if ((millis() - scopeMillis) > 10000)
            {
                scopeMillis = millis();
                Serial.print("sp:");
                Serial.print(tempSetPoint,1);
                Serial.print(",t:");
                Serial.print(temperature,1);
                Serial.print(",dimmerOutput:");
                Serial.println(dimmerOutputInt);
            }
        }

        // Disable the output if the output will be off on the next cycle
        // The triac will disable the output at the zero cross
        if ((int)dimmerOutput <= acCycle or !enable)
        {
            digitalWrite(PIN_DIMMER_OUT, LOW);
        }
    }
}
