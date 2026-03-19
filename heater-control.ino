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
const double DHT_IIR_N = 6.0f; //iir filter average n

static OneButton ButtonTempDown;
static OneButton ButtonTempUp;

bool Enable = false;
double Temperature = 0.0f, TemperatureRaw = 0.0f, PidOutput = 0.0f, TempSetPoint = 78.0f;
int DimmerOutputInt = 0;
double Kp=22.5f, Ki=0.5f, Kd=500.0f;
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

float average(double in, double last, double samples)
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
    Enable = false;
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
            // track zc
            ZcMicrosMin = min(zcMicros, ZcMicrosMin);
            ZcMicrosMax = max(zcMicros, ZcMicrosMax);
            ZcMicrosAverage = average((float)zcMicros, ZcMicrosAverage, 10);
            ZcMicrosLast = micros();

            // Enable the pin output if the dimmer output is greater than the current cycle number
            if (Enable and cycleOn)
            {
                digitalWrite(PIN_DIMMER_OUT, HIGH);
            }
            else
            {
                digitalWrite(PIN_DIMMER_OUT, LOW);
            }

            computeCompute();

            if (Compute)
            {
                // Run PID and tracing after the output is set, to prepare for the next zc
                updatePidOutput();
                DimmerOutputInt = cleanPidOutput();

            }

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

// writes global pidOutput
void updatePidOutput()
{
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
                Temperature = average(TemperatureRaw, Temperature, DHT_IIR_N);
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

// rounds the dimmer output to init, shuts down if below floor.
// Runs at 100% for init time when first turning on
// returns int from 0 to 100
// returns 0 when the output is at PID_MIN
int cleanPidOutput()
{

    int output = round(PidOutput);

    // turn off when at the floor
    if (output <= PID_MIN)
    {
        output = 0;
    }

    // reinit
    // static int cleanDimmerLastOutput;
    // static ulong cleanDimmerInitMillis; // counter for heater init

    // if (cleanDimmerLastOutput == 0 and output > PID_MIN)
    // {
    //     cleanDimmerInitMillis = millis();
    // }

    // if ((millis() - cleanDimmerInitMillis) < ComputeInvertalMs)
    // {
    //     output = PID_MAX;
    // }

    // cleanDimmerLastOutput = output;
    return output;
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

// calculate the off cycle by alternating betwen 50% DC and 100%.
// this limits the low output to 50% but allows high control resolution.
// returns true if the next half-cycle should be on.
bool calculateCycleByHalfDutyModulation(int percentIn)
{
    // ms/% = Ms/Interval / Percent/Interval
    // ms/period = 50%/period * ms/% (because 50%, not 0%, is the lowest limit with this method)
    // Period at 100ms/interval and 5%/interval is 100ms/interval * 20ms/interval = 2000ms
    const int halfCyclesPerInterval = 12; // 12 half-cycles is 100ms/interval
    const int percentPerInterval = 5; // PID output percent that each interval represents.
    const int intervalsPerPeriod = 50 / percentPerInterval;
    static int halfCycleCount = 0;
    static int intervalCount = 0;
    static bool halfCycleToggle = false;

    int fullCycleIntervalsPerPeriod = round(((float)percentIn - 50.0) / (float)percentPerInterval); //go to 50% duty cycle at this interval

    halfCycleToggle = !halfCycleToggle;

    if (++halfCycleCount >= halfCyclesPerInterval)
    {
        halfCycleCount = 0;

        if (++intervalCount >= intervalsPerPeriod)
        {
            intervalCount = 0;
        }
    }

    if (Compute)
    {
        Serial.print(",EffectiveOutput:");
        Serial.print(fullCycleIntervalsPerPeriod * percentPerInterval + 50);
    }

    // output 0 at < 50%. This method can't control below that
    if (percentIn <= 50)
    {
        return false;
    }
    else if (intervalCount < fullCycleIntervalsPerPeriod)
    {
        return true;
    }
    else
    {
        return halfCycleToggle;
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

