#include <Arduino.h>
#include <DHT.h>
#include <string.h>
#include <stddef.h>
#include <PID_v1.h>
#include <math.h>
#include <LiquidCrystal.h>

#define PIN_DHT_0V 13
#define PIN_DHT_VCC 12
#define PIN_DHT_OUT 11
#define TEMP_MIN 55u
#define TEMP_MAX 85u
#define PIN_DIMMER_OUT 3 //dimmer output
#define PIN_DIMMER_ZC 2 //zero cross
#define PIN_LIGHT 1

#define BUTTON_PIN A0 // resistor ladder buttons on lcd shield

// 1602A LCD, 4-bit mode, RW tied to ground
#define PIN_LCD_RS 8
#define PIN_LCD_EN 9
#define PIN_LCD_D4 4
#define PIN_LCD_D5 5
#define PIN_LCD_D6 6
#define PIN_LCD_D7 7
#define LCD_COLS 16
#define LCD_ROWS 2

#define PID_MAX 100
#define PID_MIN 40 // Below a certain percentage, the heat provided is ineffective and it makes the lights flicker noticably.

// PID deadband (deg F): errors within +/- this band are treated as zero, and the band is
// subtracted from larger errors so the P and I terms stay continuous at its edges.
#define PID_DEADBAND 0.2

// low temperature override hysteresis (deg F below setpoint)
#define OVERRIDE_SET_ERROR 100. //was 2
#define OVERRIDE_RESET_ERROR 99.0 // was 1

typedef unsigned long ulong;

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#define LIMIT(min, in, max) (MAX((min), MIN((max),(in))))

static DHT dht(PIN_DHT_OUT, DHT22);
static LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// Buttons on the LCD shield resistor ladder, decoded from BUTTON_PIN by readButton()
enum Button
{
    BUTTON_NONE,
    BUTTON_RIGHT,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_SELECT,
    BUTTON_RESET
};

// Display screens, rotated with the left/right buttons. Up/down act on the current screen.
enum Screen
{
    SCREEN_MAIN, // temp, humidity, setpoint, PID output; up/down change the setpoint
    SCREEN_KP,   // up/down change Kp by 10%, whole numbers, min step 1
    SCREEN_KI,   // up/down change Ki by 10%, rounded to 0.01, min step 0.01
    SCREEN_COUNT
};

static Screen CurrentScreen = SCREEN_MAIN;

const ulong DisplayIntervalMs = 250; // ms between display refreshes
static bool LcdOk = false;

static bool Enable = false;
static bool CycleOn = false;

static bool Error = false;
static char ErrorString[LCD_COLS + 2] = "";

double Temperature = 0.0f, TemperatureRaw = 0.0f, Humidity, PidOutputRaw = 0.0f, PidOutput = 0.0f;
double TempSetPoint = 73.0f;
double PidSetPoint = 0.0f; // setpoint passed to the PID, adjusted each compute to apply PID_DEADBAND
int DimmerOutputInt = 0;
// No-overshoot PI (see the Ziegler-Nichols PI notes below); Kd = 0 because derivative on a
// 2.5 s sample of a slow heater is mostly noise, and iir3 already smooths the input.
double Kp=33.0f;
double Ki=0.15f;
double Kd=0.0f;

//Ziegler-Nichols PID formulas
//Kp 75 had an oscillation of 89 seconds (Ku = 75, Tu = 89 s)

// Lots of overshoot
//Kp = 0.6 * Ku          = 45
//Ki = 1.2 * Ku / Tu     = 1.0
//Kd = 0.075 * Ku * Tu   = 500.6

// Some overshoot
//Kp = 0.33 * Ku         = 24.75    
//Ki = 0.66 * Ku / Tu    = 0.56
//Kd = 0.11 * Ku * Tu    = 734.25

// No Overshoot
//Kp = 0.2 * Ku          = 15
//Ki = 0.4 * Ku / Tu     = 0.34
//Kd = 0.0667 * Ku * Tu  = 445.0   (Kp * Tu/3, i.e. Ku * Tu / 15)

//Classic Ziegler-Nichols PID in time-constant form: Kp = 0.6 * Ku, Ti = Tu/2, Td = Tu/8
//(PID_v1 takes gains: Ki = Kp / Ti, Kd = Kp * Td, matching the "Lots of overshoot" row)

//Ziegler-Nichols PI formulas (Kd = 0), same Ku = 75, Tu = 89 s

// Classic PI (standard Ziegler-Nichols): Kp = 0.45 * Ku, Ti = Tu / 1.2
//Kp = 0.45 * Ku         = 33.75
//Ki = 0.54 * Ku / Tu    = 0.455   (Ti = 74.2 s)

// No overshoot PI (not a standard ZN rule; ZN only defines "no overshoot" for PID).
// Derived the same way the PID no-overshoot row is: Kp cut to 1/3 of classic, Ti kept.
//Kp = 0.15 * Ku         = 11.25
//Ki = 0.18 * Ku / Tu    = 0.152   (Ti = Tu / 1.2 = 74.2 s)

PID myPID(&Temperature, &PidOutputRaw, &PidSetPoint, Kp, Ki, Kd, DIRECT);

static bool Compute = false; // global command to process inputs, outputs, and traces
const ulong ComputeInvertalMs = 2500; // ms interval to compute temp and PID

ulong zcMicros; //micros since last zc event
static float ZcMicrosAverage = 0; // average microseconds between zero cross events
static ulong ZcMicrosLast = 0; // microseconds between the last zero cross event
static ulong ZcMicrosMax = 0;
static ulong ZcMicrosMin = 20000;

// Status light patterns. Add a state here and a case in updateLight() for new indications.
enum LightState
{
    LIGHT_OFF,       // heater disabled
    LIGHT_ON,        // heater enabled
    LIGHT_BLINK_1HZ  // temperature sensor error
};

const ulong LightBlink1HzHalfPeriodMs = 500;

void setup() {
    pinMode(PIN_DIMMER_OUT, OUTPUT);

    pinMode(PIN_DHT_VCC, OUTPUT);
    digitalWrite(PIN_DHT_VCC, HIGH);

    pinMode(PIN_DHT_0V, OUTPUT);
    digitalWrite(PIN_DHT_0V, LOW);

    dht.begin();

    // The built-in serial class was left as c++
    Serial.begin(115200);

    // begin() blocks for a few ms; fine here, before zc tracking starts
    lcd.begin(LCD_COLS, LCD_ROWS);

    myPID.SetOutputLimits(PID_MIN, PID_MAX);
    myPID.SetSampleTime(ComputeInvertalMs); //dht22 will only read a value every 2s
}

void loop()
{
    heater();
    buttons();
}

// Decode the resistor ladder voltage into the pressed button
Button getButtonPressed()
{
    int key = analogRead(BUTTON_PIN);

    if (key < 50)        return BUTTON_RIGHT;
    else if (key < 200)  return BUTTON_UP;
    else if (key < 400)  return BUTTON_DOWN;
    else if (key < 600)  return BUTTON_LEFT;
    else if (key < 800)  return BUTTON_SELECT;
    else if (key < 1000) return BUTTON_RESET;
    else                 return BUTTON_NONE;
}

// Act once per press (on the transition from no button / another button)
void buttons()
{
    static Button last = BUTTON_NONE;
    Button button = getButtonPressed();

    if (button == last)
    {
        return;
    }
    last = button;

    switch (button)
    {
        case BUTTON_RIGHT:
            CurrentScreen = (Screen)((CurrentScreen + 1) % SCREEN_COUNT);
            break;

        case BUTTON_LEFT:
            CurrentScreen = (Screen)((CurrentScreen + SCREEN_COUNT - 1) % SCREEN_COUNT);
            break;

        case BUTTON_UP:
            adjustScreenValue(true);
            break;

        case BUTTON_DOWN:
            adjustScreenValue(false);
            break;

        case BUTTON_SELECT:
            Enable = !Enable;
            break;

        default:
            break;
    }
}

// Up/down action for the current screen
void adjustScreenValue(bool up)
{
    double step;

    switch (CurrentScreen)
    {
        case SCREEN_KP:
            // 10% of Kp, whole numbers, at least 1
            step = MAX(1.0, round(Kp * 0.1));
            Kp = MAX(0.0, round(up ? Kp + step : Kp - step));
            break;

        case SCREEN_KI:
            // 10% of Ki, rounded to 0.01, at least 0.01
            step = MAX(0.01, round(Ki * 10.0) / 100.0);
            Ki = MAX(0.0, round((up ? Ki + step : Ki - step) * 100.0) / 100.0);
            break;

        case SCREEN_MAIN:
        default:
            TempSetPoint = LIMIT(TEMP_MIN, TempSetPoint + (up ? 1.0 : -1.0), TEMP_MAX);
            break;
    }
}

// Runs the heater loops and message sending
void heater()
{
    zcMicros = micros() - ZcMicrosLast;

    // No zcOberved for 3 cycles, turn off the heater
    if (zcMicros > 24000)
    {
        ZcMicrosLast = micros();

        // Disable the output
        Enable = false;
        digitalWrite(PIN_DIMMER_OUT, LOW);

        // Disable the PID
        myPID.SetMode(MANUAL);
        PidOutput = 0.0;

        // Message the user
        computeCompute();
        if (Compute)
        {
            Serial.println("No AC signal detected");
        }

        Error = true;
        strcpy(ErrorString,"AC Disconnected");
        UpdateDisplay();
    }

    // Look for zc. It should occur every 8.3ms. 
    // Execute the math for the next cycle after the zc is observed to allow the most time for calculation.
    else if (zcMicros > 8000 & 1 == digitalRead(PIN_DIMMER_ZC))
    {
        //****** Track ZC metrics and set triac output ******
        // track zc
        ZcMicrosMin = min(zcMicros, ZcMicrosMin);
        ZcMicrosMax = max(zcMicros, ZcMicrosMax);
        ZcMicrosAverage = iir((float)zcMicros, ZcMicrosAverage, 10, 0.0 == ZcMicrosAverage);
        ZcMicrosLast = micros();

        // Enable the pin output if the dimmer output is greater than the current cycle number
        if (Enable & CycleOn)
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
        if (Compute)
            Error = false;
        
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

        CycleOn = calculateCycleByDeltaSigma(PidOutput);

        updateKValueFromSerial();
        writeTraceOutputs();
        UpdateDisplay();
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

// Select the light state. Light is off when disabled; otherwise errors take priority over normal status, highest priority first.
LightState deriveLightState()
{
    if (!Enable)
    {
        return LIGHT_OFF;
    }
    else if (Error)
    {
        return LIGHT_BLINK_1HZ;
    }
    else
    {
        return LIGHT_ON;
    }
}

// Drive the status light for the given state. Call every loop; blink timing is based on millis().
void updateLight(LightState state)
{
    bool on;

    switch (state)
    {
        case LIGHT_ON:
            on = true;
            break;

        case LIGHT_BLINK_1HZ:
            on = 0 == (millis() / LightBlink1HzHalfPeriodMs) % 2;
            break;

        case LIGHT_OFF:
        default:
            on = false;
            break;
    }

    digitalWrite(PIN_LIGHT, on ? HIGH : LOW);
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
    double humidityTemp = 0.0;

    if (Compute)
    {
        TemperatureRaw = dht.readTemperature(true,false);
        humidityTemp = dht.readHumidity(false);

        if (!isnan(TemperatureRaw))
        {
            Temperature = iir3(round(TemperatureRaw * 10.0) / 10.0, t, n, 0.0 == Temperature);
        }
        // set error values if temp is invalid
        else
        {
            Temperature = 0;
            Error = true;
            strcpy(ErrorString,"Sensor Disc.");
        }

        if(!isnan(humidityTemp))
        {
            Humidity = humidityTemp;
        }
        // set error avlue if humidity is invalid - this is not considered a system error since it's not used for ocntrol
        else
        {
            Humidity = 0;
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
            if ((myPID.GetKp() != Kp) | (myPID.GetKi() != Ki))
            {
                myPID.SetTunings(Kp,Ki,Kd);
            }

            // Apply the deadband through the setpoint rather than the input: PID_v1 takes
            // the derivative on the input, so this adds no derivative kick.
            PidSetPoint = Temperature + applyDeadband(TempSetPoint - Temperature, PID_DEADBAND);

            // writes global pidOutput directly
            myPID.Compute();

            if (PidOutput <= PID_MIN)
            {
                PidOutput = 0;
            }
        }
        else
        {
            PidOutputRaw = 0;
        }

        // turn off when at the floor. the heater isn't outputing useful heat below this level
        if (PidOutputRaw <= PID_MIN)
        {
            PidOutput = 0;
        }
        else
        {
            PidOutput = PidOutputRaw;
        }
    }
}

// Returns 0 when |error| <= deadband, otherwise error moved toward 0 by deadband.
double applyDeadband(double error, double deadband)
{
    if (error > deadband)
    {
        return error - deadband;
    }
    else if (error < -deadband)
    {
        return error + deadband;
    }
    else
    {
        return 0.0;
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
        Serial.print(",PidOutputRaw:");
        Serial.print(PidOutputRaw);    
        Serial.println();
    }
}

void UpdateDisplay()
{
    static uint8_t row = 0;
    static bool error = false;
    static bool enable = false;
    static Screen screen = SCREEN_MAIN;

    if (Error)
    {
        if (!error)
            lcd.clear();

        lcd.setCursor(0, 0);
        lcd.print("ERROR!");
        lcd.setCursor(0, 1);
        lcd.print(ErrorString);
        error = true;
    }

    else
    {
        error = false;

        if ((enable != Enable) | (screen != CurrentScreen))
        {
            lcd.clear();
            enable = Enable;
            screen = CurrentScreen;
        }

        if (CurrentScreen != SCREEN_MAIN)
        {
            updateGainDisplay();
        }
        else if (row == 0)
        {
            lcd.setCursor(0, 0);
            lcd.print("Temp ");
            lcd.print(Temperature, 1);
            lcd.print("F RH ");
            lcd.print(Humidity, 0);
            lcd.print("%");
            row++;
        }

        if (row == 1)
        {
            lcd.setCursor(0, 1);
            lcd.print("Set: ");
            lcd.print(TempSetPoint, 0);
            lcd.print("F ");

            if (Enable)
            {
                lcd.print("PID ");
                lcd.print(PidOutput,0);
                lcd.print("%");
            }
            else
            {
                lcd.print("OFF");
            }

            row = 0;
        }
    }
}

// Kp / Ki screen: the selected gain on row 0, temperature and PID output on row 1 for feedback while tuning.
// Rows are padded with spaces so a shorter value overwrites a longer one.
void updateGainDisplay()
{
    size_t n;

    lcd.setCursor(0, 0);
    if (CurrentScreen == SCREEN_KP)
    {
        n = lcd.print("Kp: ");
        n += lcd.print(Kp, 0);
    }
    else
    {
        n = lcd.print("Ki: ");
        n += lcd.print(Ki, 2);
    }
    padLcdRow(n);

    lcd.setCursor(0, 1);
    n = lcd.print("T ");
    n += lcd.print(Temperature, 1);
    n += lcd.print("F ");
    if (Enable)
    {
        n += lcd.print("PID ");
        n += lcd.print(PidOutput, 0);
        n += lcd.print("%");
    }
    else
    {
        n += lcd.print("OFF");
    }
    padLcdRow(n);
}

// Fill the rest of an LCD row with spaces after n characters were written
void padLcdRow(size_t n)
{
    for (; n < LCD_COLS; n++)
    {
        lcd.print(' ');
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
