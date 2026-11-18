#include <Arduino.h>
#include <DHT.h>
#include <string.h>
#include <stddef.h>
#include <math.h>
#include <LiquidCrystal.h>

#define PIN_DHT_0V 13
#define PIN_DHT_VCC 12
#define PIN_DHT_OUT 11
// Setpoint limits in deg F. The setpoint is adjusted in whole deg F but stored and controlled in deg C.
#define TEMP_MIN_F 55.0
#define TEMP_MAX_F 85.0
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

// PID deadband (deg C): errors within +/- this band are treated as zero, and the band is
// subtracted from larger errors so the P and I terms stay continuous at its edges.
// The DHT22 reads in 0.1 C steps, so a band of at least half a step (0.05 C) always contains a
// reachable reading, whatever the setpoint; without it the integral hunts between two readings.
#define PID_DEADBAND 0.06

// Starting value of the PI integral (output %) at power-up. Ideally near the output needed to
// hold the setpoint, so control starts close to steady state.
#define PI_INTEGRAL_SEED 50.0

typedef unsigned long ulong;

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#define LIMIT(min, in, max) (MAX((min), MIN((max),(in))))

// Macros, not functions: a global initializer uses fToC, and a function defined this early would
// make the Arduino preprocessor put its generated prototypes above the enums they need.
#define cToF(c) ((c) * 1.8 + 32.0)
#define fToC(f) (((f) - 32.0) / 1.8)

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

// All temperatures are deg C internally; the LCD shows deg F.
double Temperature = 0.0f, TemperatureRaw = 0.0f, Humidity, PidOutputRaw = 0.0f, PidOutput = 0.0f;
static bool TemperatureValid = false; // false until the first good DHT read, and after a failed one
double TempSetPoint = fToC(73.0);
int DimmerOutputInt = 0;
// PI gains (see the Ziegler-Nichols PI notes below). Kp is % output per deg C; Ki is % output
// per deg C per second. No derivative: on a 2.5 s sample of a slow heater it is mostly noise,
// and iir3 already smooths the input.
double Kp=59.0f; // was 33 %/F
double Ki=0.27f; // was 0.15 %/F/s
double PiIntegral = PI_INTEGRAL_SEED; // integral term, in output %, kept between [PID_MIN, PID_MAX]

//Ziegler-Nichols PI formulas (Kd = 0), Tu = 89 s
//Ku = 75 %/F measured, = 135 %/C (gains in deg C units are 1.8x the deg F ones)
// Classic PI (standard Ziegler-Nichols): Kp = 0.45 * Ku, Ti = Tu / 1.2
//Kp = 0.45 * Ku         = 60.75   (33.75 in %/F)
//Ki = 0.54 * Ku / Tu    = 0.819   (0.455 in %/F/s, Ti = 74.2 s)

// No overshoot PI (not a standard ZN rule; ZN only defines "no overshoot" for PID).
// Derived the same way the PID no-overshoot row is: Kp cut to 1/3 of classic, Ti kept.
//Kp = 0.15 * Ku         = 20.25   (11.25 in %/F)
//Ki = 0.18 * Ku / Tu    = 0.273   (0.152 in %/F/s, Ti = Tu / 1.2 = 74.2 s)

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
            // step in whole deg F; rounding first keeps the setpoint on whole deg F despite the C storage
            TempSetPoint = fToC(LIMIT(TEMP_MIN_F, round(cToF(TempSetPoint)) + (up ? 1.0 : -1.0), TEMP_MAX_F));
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

        PidOutputRaw = 0.0;
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

        // Run PI and tracing after the output is set, to prepare for the next zc
        updatePidOutput();

        CycleOn = calculateCycleByDeltaSigma(PidOutput);

        updateKValueFromSerial();
        writeTraceOutputs();
        UpdateDisplay();
    }
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
    const double n = 2.0; //iir filter average n
    static double t[3] = {0,0,0};
    double humidityTemp = 0.0;

    if (Compute)
    {
        // deg C, the sensor's native unit (0.1 C steps), so no rounding is needed
        TemperatureRaw = dht.readTemperature(false,false);
        humidityTemp = dht.readHumidity(false);

        if (!isnan(TemperatureRaw))
        {
            // re-initialize the filter on the first good read after startup or a failure
            //Temperature = iir3(TemperatureRaw, t, n, !TemperatureValid); TODO testing no filter
            Temperature = TemperatureRaw;
            TemperatureValid = true;
        }
        // set error values if temp is invalid
        else
        {
            Temperature = 0;
            TemperatureValid = false;
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

// Writes globals PidOutputRaw (PI output) and PidOutput (what drives the heater).
// Disabled or no valid temperature: output 0 immediately and the integral is held.
void updatePidOutput()
{
    if (!Enable | !TemperatureValid)
    {
        PidOutputRaw = 0;
        PidOutput = 0;
    }
    else if (Compute)
    {
        PidOutputRaw = computePi(TempSetPoint, Temperature);

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

// One PI step, called every ComputeInvertalMs. Returns the output in [PID_MIN, PID_MAX].
// Anti-windup by conditional integration: the integral is only updated when that doesn't push
// the output further into saturation. During warm-up the P term holds the output at PID_MAX
// while the integral stays put, so there is no stored excess to unwind as overshoot.
// The integral is kept in output %, so changing Ki doesn't bump the output.
double computePi(double setPoint, double input)
{
    const double dtSec = ComputeInvertalMs / 1000.0;
    double error = applyDeadband(setPoint - input, PID_DEADBAND);
    double p = Kp * error;
    double integral = PiIntegral + Ki * dtSec * error;
    double output = p + integral;

    bool saturatedHigh = (output > PID_MAX) & (error > 0);
    bool saturatedLow = (output < PID_MIN) & (error < 0);

    if (!(saturatedHigh | saturatedLow))
    {
        PiIntegral = LIMIT(PID_MIN, integral, PID_MAX);
    }

    return LIMIT(PID_MIN, p + PiIntegral, PID_MAX);
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
    // Scope output, temperatures in deg C
    if (Compute)
    {
        Serial.print(",sp:");
        Serial.print(TempSetPoint,2);
        Serial.print(",t:");
        Serial.print(Temperature,2);
        Serial.print(",tRaw:");
        Serial.print(TemperatureRaw,2);     
        Serial.print(",PidOutput:");
        Serial.print(PidOutput);        
        Serial.print(",PidOutputRaw:");
        Serial.print(PidOutputRaw);
        Serial.print(",PiIntegral:");
        Serial.print(PiIntegral);
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
            lcd.print(cToF(Temperature), 1);
            lcd.print("F RH ");
            lcd.print(Humidity, 0);
            lcd.print("%");
            row++;
        }

        if (row == 1)
        {
            lcd.setCursor(0, 1);
            lcd.print("Set: ");
            lcd.print(cToF(TempSetPoint), 0);
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
    n += lcd.print(cToF(Temperature), 1);
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
