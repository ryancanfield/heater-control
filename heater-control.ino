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

// low temperature override hysteresis (deg F below setpoint)
#define OVERRIDE_SET_ERROR 2.0
#define OVERRIDE_RESET_ERROR 1.0

typedef unsigned long ulong;

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#define LIMIT(min, in, max) (MAX((min), MIN((max),(in))))

static DHT dht(PIN_DHT, DHT22);

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
            //****** Track ZC metrics and set triac output ******
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

            //****** Calculate next cycle after the outputs for the current cycle are set ******
            // This allows the longest time to run calculations and output any trace data without impacting the zc detection.
            computeCompute();
            updateTemperature();

            // Disabled or no valid temperature: heater off.
            // Far below setpoint: full output with the PID bypassed to avoid integral windup.
            // Otherwise: PID control.
            if (!Enable | (Temperature <= 0))
            {
                myPID.SetMode(MANUAL);
                PidOutput = 0.0;
            }
            else if (deriveLowTemperatureOverride())
            {
                myPID.SetMode(MANUAL);
                PidOutput = PID_MAX;
            }
            else if (MANUAL == myPID.GetMode())
            {
                // SetMode(AUTOMATIC) seeds the integral from PidOutput; start it at the floor
                // so the full-power warm-up doesn't carry over and cause overshoot.
                PidOutput = PID_MIN;
                myPID.SetMode(AUTOMATIC);
            }

            // Run PID and tracing after the output is set, to prepare for the next zc
            updatePidOutput();

            cycleOn = calculateCycleByDeltaSigma(PidOutput);

            updateKValueFromSerial();
            writeTraceOutputs();
        }
    }
}

// Returns true if the temperature is low (error is high).
// Latches until error is low.
bool deriveLowTemperatureOverride()
{
    static bool lowTempOverride = false;
    double error = TempSetPoint - Temperature;

    // calc low temp override
    // reset when temperature is near the setpoint
    if (error < OVERRIDE_RESET_ERROR)
    {
        lowTempOverride = false;
    }
    // set when temperature is low
    else if (error > OVERRIDE_SET_ERROR)
    {
        lowTempOverride = true;
    }

    return lowTempOverride;
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
    const double n = 3.0; //iir filter average n
    static double t[3] = {0,0,0};

    if (Compute)
    {
        TemperatureRaw = dht.readTemperature(true,false);

        if (!isnan(TemperatureRaw))
        {
            Temperature = iir3(round(TemperatureRaw * 10.0) / 10.0, t, n, 0.0 == Temperature);
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

// Determine if cycle should be on using Bresenham/Delta-Sigma Algorithm
// returns true if the next half-cycle should be on.
bool calculateCycleByDeltaSigma(double percentIn)
{
    static double bucket = 0;

    bucket += percentIn;

    if (bucket >= 100.0)
    {
        bucket -= 100.0;
        return true;
    }
    else
    {
        return false;
    }
}

// Non-blocking read of a positive integer Kp from serial; applied when a non-digit (e.g. newline) arrives.
// Serial.parseInt() is not used because it blocks for up to its timeout, which would stall zc detection.
void updateKValueFromSerial()
{
    static long val = 0;

    while (Serial.available() > 0)
    {
        char c = Serial.read();

        if (('0' <= c) & (c <= '9'))
        {
            val = val * 10 + (c - '0');
        }
        else
        {
            if (val > 0)
            {
                Kp = val;
            }
            val = 0;
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
