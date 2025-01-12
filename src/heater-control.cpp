#include <Arduino.h>
#include <DHT.h>
#include <string.h>
#include <stddef.h>
#include "OneButton.h"

#define PIN_DHT 10
#define PIN_TEMP_DOWN 6
#define PIN_TEMP_UP 8
#define PIN_FAN 3
#define PIN_HEAT 4
#define PIN_COOL 5
#define TEMP_MIN 55u
#define TEMP_MAX 80u
#define PIN_DIMMER_OUT 9 //dimmer output
#define PIN_DIMMER_ZC 2 //zero cross

typedef unsigned long ulong;

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#define LIMIT(min, in, max) (MAX((min), MIN((max),(in))))

enum HvacMode{HvacMode_Off, HvacMode_Fan, HvacMode_Heat, HvacMode_Cool};

static DHT dht(PIN_DHT, DHT11);
static unsigned long dhtReadIntervalMillis = 0;
static float tempSetPoint = 72.0f;
static OneButton buttonTempDown;
static OneButton buttonTempUp;
static OneButton zeroCross;
static float tempHysteresis = 1.0f;

static HvacMode hvacMode = HvacMode_Off;

static int zc = 0; // last zero cross input value
static float zcMicrosAverage = 0; // average microseconds between zero cross events
static ulong zcMicrosLast = 0; // microseconds between the last zero cross event
static ulong scopeMillis = 0; // millis of last scope execution

float average(float in, float last, float samples)
{
    return last + (in - last) / samples;
}


void loopDimmer()
{
    // trigger the 
    if (zc != digitalRead(PIN_DIMMER_ZC))
    {
        zc = digitalRead(PIN_DIMMER_ZC);
        if(zc == 1)
        {
            zcMicrosAverage = average((float)(micros() - zcMicrosLast), zcMicrosAverage, 10);
            zcMicrosLast = micros();
        }
    }
}


void loopThermostat()
{
    // **** Handle setpoint ****
    buttonTempDown.process(&buttonTempDown);
    buttonTempUp.process(&buttonTempUp);

    if (buttonTempUp.oneShot){
        tempSetPoint = LIMIT(TEMP_MIN, tempSetPoint + 1.0, TEMP_MAX);
        // Serial.print("Temp Setpoint Changed to ");
        // Serial.println(tempSetPoint);
    }

    if (buttonTempDown.oneShot){
        tempSetPoint = LIMIT(TEMP_MIN, tempSetPoint - 1.0, TEMP_MAX);
        // Serial.print("Temp Setpoint Changed to ");
        // Serial.println(tempSetPoint);
    }

    // **** Handle Temperature and thermostat ****
    if ((millis() - dhtReadIntervalMillis) > 2000){
        dhtReadIntervalMillis = millis();

        // Determine mode
        if (tempSetPoint - tempHysteresis > dht.readTemperature(true)){
            hvacMode = HvacMode_Heat;
        }
        else if (tempSetPoint + tempHysteresis < dht.readTemperature(true)){
            hvacMode = HvacMode_Cool;
        }
        else if (HvacMode_Cool == hvacMode && 
                tempSetPoint > dht.readTemperature(true)){
            hvacMode = HvacMode_Off;
        }
        else if (HvacMode_Heat == hvacMode &&
                 tempSetPoint < dht.readTemperature(true)){
            hvacMode = HvacMode_Off;
        }

        // Set Outputs
        switch (hvacMode)
        {
            case HvacMode_Off:
                // Serial.println(" Hvac Off");
                digitalWrite(PIN_FAN, 0);
                digitalWrite(PIN_COOL, 0);
                digitalWrite(PIN_HEAT, 0);
                break;
            case HvacMode_Cool:
                // Serial.println(" Hvac Cool");
                digitalWrite(PIN_FAN, 1);
                digitalWrite(PIN_COOL, 1);
                digitalWrite(PIN_HEAT, 0);
                break;   
            case HvacMode_Heat:
                // Serial.println(" Hvac Heat");
                digitalWrite(PIN_FAN, 1);
                digitalWrite(PIN_COOL, 0);
                digitalWrite(PIN_HEAT, 1);
                break;                 
            default:
                break;
        }
    }
}

void tempDownClick()
{

}

void tempUpClick()
{

}

void zeroCrossClick()
{
    zcMicrosAverage = average((float)(micros() - zcMicrosLast), zcMicrosAverage, 10);
    zcMicrosLast = micros();
}


void setup() {
    // dht.begin();

    // The built-in serial class was left as c++
    Serial.begin(115200);

    buttonTempDown.setup(PIN_TEMP_DOWN, INPUT_PULLUP,true);
    buttonTempDown.attachClick(tempDownClick);

    buttonTempUp.setup(PIN_TEMP_UP, INPUT_PULLUP,true);     
    buttonTempUp.attachClick(tempUpClick);

    zeroCross.setup(PIN_DIMMER_ZC, INPUT, true);
    zeroCross.attachClick(zeroCrossClick);

    pinMode(PIN_FAN, OUTPUT);
    pinMode(PIN_HEAT, OUTPUT);
    pinMode(PIN_COOL, OUTPUT);
}

void loop()
{
    buttonTempDown.tick();
    buttonTempUp.tick();

    loopDimmer();
    loopThermostat();

    // write scope
    if ((millis() - scopeMillis) > 100)
    {
        scopeMillis = millis();
        // Serial.print("sp:");
        // Serial.print(tempSetPoint,1);
        // Serial.print(",t:");
        // Serial.print(dht.readTemperature(true),1);
        // Serial.print(",h:");
        // Serial.print(dht.readHumidity(),1);
        // Serial.print(",us:");
        Serial.print("us:");
        Serial.println(zcMicrosAverage);
    }
}
