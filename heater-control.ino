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

#define PID_MAX 100
#define PID_MIN 49

typedef unsigned long ulong;

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#define LIMIT(min, in, max) (MAX((min), MIN((max),(in))))

static DHT dht(PIN_DHT, DHT22);
const double DHT_IIR_N = 3.0f; //iir filter average n

static OneButton ButtonTempDown;
static OneButton ButtonTempUp;

bool Enable = false;
double Temperature = 0.0f, TemperatureRaw = 0.0f, PidOutput = 0.0f, TempSetPoint = 78.0f;
int DimmerOutputInt = 0;
double Kp=100.0f, Ki=0.0f, Kd=0.0f;
//kp 100 had an oscilation of 15 seconds

//Kp = 0.6 * Ku
//Ki = 1.2 * (Ku/Tu)
//Kd = 0.075 * (Ku * Tu)

//zeigler nicholds pid: Kp = 0.6 * Ku, Ki = 2/Tu, Kd = Tu/8
PID myPID(&Temperature, &PidOutput, &TempSetPoint, Kp, Ki, Kd, DIRECT);

static ulong LastComputeMs = 0; // last ms the pid and dht read were completed
const ulong ComputeInvertalMs = 2000; // ms interval to compute temp and PID

static float ZcMicrosAverage = 0; // average microseconds between zero cross events
static ulong ZcMicrosLast = 0; // microseconds between the last zero cross event
static ulong ZcMicrosMax = 0;
static ulong ZcMicrosMin = 20000;

static int Period;
static int CyclesOn;

float average(float in, float last, float samples)
{
    return last + (in - last) / samples;
}

void tempDownClick()
{
    TempSetPoint = LIMIT(TEMP_MIN, TempSetPoint - 1.0, TEMP_MAX);
}

void tempUpClick()
{
    TempSetPoint = LIMIT(TEMP_MIN, TempSetPoint + 1.0, TEMP_MAX);
}

void tempUpLongPress()
{
    Enable = true;
    digitalWrite(PIN_HEAT,1);
    myPID.SetMode(AUTOMATIC);

}

void tempDownLongPress()
{
    Enable = true;
    digitalWrite(PIN_HEAT,0);
    myPID.SetMode(MANUAL);
}

void setup() {
    dht.begin();

    // The built-in serial class was left as c++
    Serial.begin(115200);

    ButtonTempDown.setup(PIN_TEMP_DOWN, INPUT_PULLUP,true);
    ButtonTempDown.attachClick(tempDownClick);
    ButtonTempDown.attachLongPressStart(tempDownLongPress);

    ButtonTempUp.setup(PIN_TEMP_UP, INPUT_PULLUP,true);     
    ButtonTempUp.attachClick(tempUpClick);
    ButtonTempUp.attachLongPressStart(tempUpLongPress);

    pinMode(PIN_DIMMER_OUT, OUTPUT);
    pinMode(PIN_CYCLE, OUTPUT);
    pinMode(PIN_HEAT, OUTPUT);

    myPID.SetOutputLimits(PID_MIN, PID_MAX);
    myPID.SetSampleTime(2000); //dht22 will only read a value every 2s
}

void loop()
{
    static int cycleCount = 0;

    ButtonTempDown.tick();
    ButtonTempUp.tick();

    ulong zcMicros; //micros since last zc event
    zcMicros = micros() - ZcMicrosLast;

    // Look for zc 7.5ms after the last zc
    // Should occur at 8ms
    if (zcMicros > 7500)
    {
        // after the zc is first observed, calculated the PID and write outputs for the next cycle
        if (1 == digitalRead(PIN_DIMMER_ZC))
        {
            // track zc
            ZcMicrosMin = min(zcMicros, ZcMicrosMin);
            ZcMicrosMax = max(zcMicros, ZcMicrosMax);
            ZcMicrosAverage = average((float)zcMicros, ZcMicrosAverage, 10);
            ZcMicrosLast = micros();

            // Enable the pin output if the dimmer output is greater than the current cycle number
            if (CyclesOn > cycleCount and Enable)
            {
                digitalWrite(PIN_DIMMER_OUT, HIGH);
            }
            else
            {
                digitalWrite(PIN_DIMMER_OUT, LOW);
            }

            // count the cycles
            cycleCount++;
            if (cycleCount >= Period)
            {
                cycleCount = 0;
            }

            updatePidOutput();
            DimmerOutputInt = cleanPidOutput();
            calculateCycles(DimmerOutputInt, &Period, &CyclesOn);

            writeTraceOutputs();
            updateKValueFromSerial();
        }
    }
}

// writes global pidOutput
void updatePidOutput()
{
    static ulong computeMs;

    if ((millis() - computeMs) > 2000)
    {
        computeMs = millis(); 

        // get temp and calculate pid at the zc so the intervals are equal
        TemperatureRaw = dht.readTemperature(true,false);

        if (!isnan(TemperatureRaw))
        {
            if (Temperature == 0)
            {
                Temperature = TemperatureRaw;
            } 
            else
            {
                Temperature = Temperature + (TemperatureRaw-Temperature) / DHT_IIR_N;
            }
        }
        // set error values if temp is invalid
        else
        {
            Temperature = 0;
        }

        // Run PID
        if ((Temperature > 0) & Enable)
        {
            if (myPID.GetKp() != Kp)
            {
                myPID.SetTunings(Kp,Ki,Kd);
            }

            // writes global pidOutput directly
            myPID.Compute();
        }
        else
        {
            PidOutput = 0;
        }
    }
}

// rounds the dimmer output to init, shuts down if below floor.
// Runs at 100% for init time when first turning on
// returns int from 0 to 100
int cleanPidOutput()
{
    static int cleanDimmerLastOutput;
    static ulong cleanDimmerInitMillis; // counter for heater init

    int output = round(PidOutput);

    // turn off when at the floor
    if (output <= PID_MIN)
    {
        output = 0;
    }

    // reinit
    if (cleanDimmerLastOutput == 0 and output > PID_MIN)
    {
        cleanDimmerInitMillis = millis();
    }

    if ((millis() - cleanDimmerInitMillis) < 2000)
    {
        output = PID_MAX;
    }

    cleanDimmerLastOutput = output;
    return output;
}

// returns cycles on and cycle period
// the intent here is to keep the off time as low as possible (1 cycle) to keep the heat output smooth
// input is array {cycle period, cycles on}
void calculateCycles(int input, int* period, int* cyclesOn)
{
    static const int table[7][3] = {
        {100,1,1},
        {90,10,9},
        {80,5,4},
        {75,4,3},
        {67,3,2},
        {50,2,1},
        {0,1,0}
    };

    // find the table entry
    for (int i = 0; i < 7; i++)
    {
        if (input >= table[i][0])
        {
            *period = table[i][1];
            *cyclesOn = table[i][2];
            break;
        }
    }
}

int updateKValueFromSerial()
{  // if there's any serial available, read it:
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    long val = Serial.parseInt();
    if (val > 0)
    {
        Kp = val;
    }
  }
}

void writeTraceOutputs()
{
    static ulong traceMs; // millis of last scope execution

    // Scope output
    if ((millis() - traceMs) > 10000)
    {
        traceMs = millis();
        Serial.print("sp:");
        Serial.print(TempSetPoint,1);
        Serial.print(",t:");
        Serial.print(Temperature,1);
        Serial.print(",PidOutput:");
        Serial.print(PidOutput);
        Serial.print(",Period:");
        Serial.print(Period);
        Serial.print(",CyclesOn:");
        Serial.print(CyclesOn);
        
        Serial.println();
    }
}

