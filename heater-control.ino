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
#define PID_MIN 39

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
double Kp=40.0f; 
double Ki=0.125f;
// double Ki = 0.0;
double Kd=700.0f;
// double Kd = 0.0;

//kp 75 had an oscilation of 89 seconds

// Lots of overshoot
//Kp = 0.6 * Ku
//Ki = 1.2 * (Ku/Tu) = 
//Kd = 0.075 * (Ku * Tu)

// Some overshoot
//Kp = 0.33 * Ku
//Ki = 0.66 * Ku/Tu
//Kd = 0.11 * Ku * Tu

// No Overshoot
//Kp = 0.2 * Ku
//Ki = 0.4 * Ku/Tu
//Kd = 0.066 * Ku * Tu - (this seems wrong)

//zeigler nicholds pid: Kp = 0.6 * Ku, Ki = 2/Tu, Kd = Tu/8
PID myPID(&Temperature, &PidOutput, &TempSetPoint, Kp, Ki, Kd, DIRECT);

static bool Compute = false; // global command to process inputs, outputs, and traces
const ulong ComputeInvertalMs = 2500; // ms interval to compute temp and PID

static float ZcMicrosAverage = 0; // average microseconds between zero cross events
static ulong ZcMicrosLast = 0; // microseconds between the last zero cross event
static ulong ZcMicrosMax = 0;
static ulong ZcMicrosMin = 20000;

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
}

void tempDownLongPress()
{
    Enable = false;
    digitalWrite(PIN_HEAT,0);
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
    myPID.SetSampleTime(ComputeInvertalMs); //dht22 will only read a value every 2s
}

void loop()
{
    static bool cycleOn = false;
    static bool lowTempOverride = false;

    ButtonTempDown.tick();
    ButtonTempUp.tick();

    ulong zcMicros; //micros since last zc event
    zcMicros = micros() - ZcMicrosLast;

    // Look for zc
    // Should occur every 8.3ms.
    // Execute the math for the next cycle after the zc is observed to allow the most time for calculatino.
    if (zcMicros > 8000)
    {
        // after the zc is first observed, calculated the PID and write outputs for the next cycle
        if (1 == digitalRead(PIN_DIMMER_ZC))
        {
            // track zc
            ZcMicrosMin = min(zcMicros, ZcMicrosMin);
            ZcMicrosMax = max(zcMicros, ZcMicrosMax);
            ZcMicrosAverage = iir((float)zcMicros, ZcMicrosAverage, 10, 0.0 == ZcMicrosAverage);
            ZcMicrosLast = micros();

            // Enable the pin output if the dimmer output is greater than the current cycle number
            if (Enable & cycleOn)
            {
                digitalWrite(PIN_DIMMER_OUT, HIGH);
            }
            else
            {
                digitalWrite(PIN_DIMMER_OUT, LOW);
            }

            computeCompute();
            updateTemperature();

            // calc low temp override
            // reset when enabled and temperature high
            if (Enable & 1.0 > TempSetPoint - Temperature)
            {
                lowTempOverride = false;
            }
            // set when not enabled and temperature low
            else if (!Enable & 1 < TempSetPoint - Temperature)
            {
                lowTempOverride = true;
            }

            if (!Enable)
            {
                // In manual mode, the pid output wiill be 100% if the setpoint is higher, 0% if lower
                myPID.SetMode(MANUAL);
            }
            else
            {
                myPID.SetMode(AUTOMATIC);
            }

            // Run PID and tracing after the output is set, to prepare for the next zc
            updatePidOutput();

            //cycleOn = calculateCycleEnableByPeriodic(DimmerOutputInt, &EffectiveOutput, &Period, &CyclesOn);
            cycleOn = calculateCycleByHalfDutyModulation(DimmerOutputInt);

            updateKValueFromSerial();
            writeTraceOutputs();
        }
    }
}

void computeCompute()
{
    static ulong computeMs;

    if(millis() - computeMs > ComputeInvertalMs)
    {
        Compute = true;
        computeMs = millis();
    }
    else
    {
        Compute = false;
    }
}

void updateTemperature()
{
    static double t[3] = {0,0,0};

    if (Compute)
    {
        // get temp and calculate pid at the zc so the intervals are equal
        TemperatureRaw = dht.readTemperature(true,false);

        if (!isnan(TemperatureRaw))
        {
            Temperature = iir3(TemperatureRaw, t, DHT_IIR_N, 0.0f == Temperature);
        }
        // set error values if temp is invalid
        else
        {
            Temperature = 0;
        }
    }
}

// writes global pidOutput
void updatePidOutput()
{
    if (Compute)
    {
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

        // turn off when at the floor. the heater isn't outputing useful heat below this level
        if (PidOutput <= PID_MIN)
        {
            PidOutput = 0;
        }
    }
}

// returns true when next cycle should be on
// the intent here is to keep the off time as low as possible (1 cycle) to keep the heat output smooth
// input is array {cycle period, cycles on}
// 20260318 - the flicker is still observable because it creates a pattern
bool calculateCycleEnableByPeriodic(int percentIn)
{
    static int cycleCount = 0;
    int period;
    int cyclesOn;
    int percentOut;

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
        if (percentIn >= table[i][0])
        {
            percentOut = table[i][0];
            period = table[i][1];
            cyclesOn = table[i][2];
            break;
        }
    }

    // count the cycles
    if (++cycleCount >= period)
    {
        cycleCount = 0;
    }

    // Enable the pin output if the current cycle is less than the number of on-cycles
    return cycleCount < cyclesOn;
}

// calculate the off cycle by alternating betwen 50% DC, and 100%.
// the intent here is to limit the visible voltage changes
// returns true if the next half-cycle should be on.
bool calculateCycleByHalfDutyModulation(double percentIn)
{
    static double bucket = 0;

    bucket += percentIn;

    if (bucket > 100)
    {
        bucket -= 100.0;
        return true;
    }
    else
    {
        return false;
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
    // Scope output
    if (Compute)
    {
        Serial.print(",sp:");
        Serial.print(TempSetPoint,0);
        Serial.print(",t:");
        Serial.print(Temperature,2);
        Serial.print(",tRaw:");
        Serial.print(TemperatureRaw,2);     
        Serial.print(",PidOutput:");
        Serial.print(PidOutput);        
        Serial.println();
    }
}

double iir3(double in, double* last, double samples, bool init)
{
    last[0] = iir(in, last[0], samples, init);
    last[1] = iir(last[0], last[1], samples, init);
    last[2] = iir(last[1], last[2], samples, init);
    return last[2];
}

double iir(double in, double last, double samples, bool init)
{
    if (init)
    {
        return in;
    }
    else
    {
        return last + (in - last) / samples;
    }
}
