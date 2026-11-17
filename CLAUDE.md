# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Single-file Arduino sketch ([heater-control.ino](heater-control.ino)) implementing integral-cycle (zero-cross) TRIAC control of a resistive heater, with PID temperature regulation from a DHT22 sensor. The goal is smooth, quiet heat output (no phase-angle chopping), e.g. for an under-desk heater. `heater control worksheet.xlsx` holds PID tuning/design calculations.

## Build / Upload

Built with the Arduino IDE; there are no tests. Required libraries, installed via the IDE Library Manager: `DHT` (Adafruit DHT sensor library), `LiquidCrystal`. The PI controller is in the sketch (no PID library). The target board is not pinned in the repo; assume an AVR board (Uno/Nano class).

To compile-check from the command line, use the arduino-cli bundled with the IDE (not on PATH):

```
"$LOCALAPPDATA/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe" compile --fqbn arduino:avr:uno --warnings all .
```

Serial monitor/plotter at **115200 baud**. Trace lines are `,sp:..,t:..,tRaw:..,PidOutput:..,PidOutputRaw:..,PiIntegral:..` formatted for the Arduino Serial Plotter. Sending a positive integer followed by a newline sets `Kp` live (applied on the next PID compute). Serial input is parsed non-blocking on purpose; don't use `Serial.parseInt()`, since its timeout would stall zero-cross detection.

## Architecture

Everything runs in `loop()`, synchronized to the AC zero cross by **polling** (not an interrupt) `PIN_DIMMER_ZC`:

1. Once >8000 µs have passed since the last zero cross and the ZC pin reads high, a new half-cycle (~8.3 ms at 60 Hz) begins.
2. The TRIAC output (`PIN_DIMMER_OUT`) is set **first**, using the `cycleOn` decision computed on the previous half-cycle — this keeps switching aligned to the zero cross.
3. The remaining time in the half-cycle is used for slower work: `computeCompute()` raises the `Compute` flag every `ComputeInvertalMs` (2.5 s, DHT22 minimum read interval ≈2 s); temperature read + filtering, PID, and serial trace only run when `Compute` is true.
4. `calculateCycleByDeltaSigma(PidOutput)` runs every half-cycle and converts the 0–100 % PID output into an on/off pulse density (Bresenham / sigma-delta accumulator).

Any work added to the loop must fit inside one half-cycle, or zero-cross timing (tracked in `ZcMicrosMin/Max/Average`) will slip.

Control details:
- Temperature is °F, rounded to 0.1, then smoothed by `iir3` (three cascaded first-order IIRs).
- Control is a PI loop written in the sketch, `computePi()`, run every `Compute` from `updatePidOutput()`. `Kp` is % output per °F, `Ki` is % output per °F per second, and `PID_DEADBAND` is applied to the error.
  - The integral `PiIntegral` is stored in output %, so changing `Ki` doesn't bump the output. It starts at `PI_INTEGRAL_SEED` (50) at power-up and is kept within `[PID_MIN, PID_MAX]`.
  - Anti-windup: the integral is only updated when that wouldn't push the output further into saturation. During warm-up, P holds the output at `PID_MAX` while the integral stays put, so there is no separate warm-up override.
  - The PI output (`PidOutputRaw`) is limited to `[PID_MIN, PID_MAX]` = `[40, 100]`. `PidOutput`, which drives the heater, is 0 when the raw output is ≤ `PID_MIN`, because the heater produces no useful heat below that level.
  - Disabled, or no valid temperature (`Temperature <= 0`, i.e. a DHT read failed): output 0 every half-cycle, and the integral is held (not reset) until control resumes.
- Buttons (LCD shield resistor ladder on `BUTTON_PIN` A0): `readButton()` decodes the analog value to a `Button`, and `getButtons()` acts once per press. Select toggles `Enable`. Left/right rotate `CurrentScreen` through `Screen` (`SCREEN_MAIN` default, `SCREEN_KP`, `SCREEN_KI`); up/down call `adjustScreenValue()`, which acts on the current screen: Main = setpoint ±1 °F (clamped 55–85), Kp = ±10 % rounded to whole numbers (min step 1), Ki = ±10 % rounded to 0.01 (min step 0.01). Gains are clamped at ≥ 0. To add a screen, add a `Screen` value before `SCREEN_COUNT`, a case in `adjustScreenValue()`, and its display in `UpdateDisplay()`.
- Status LED (`PIN_LIGHT`): `deriveLightState()` maps error flags and `Enable` to a `LightState`, and `updateLight()` drives the pin every `loop()` iteration, using `millis()` for blink timing. Disabled always means off (errors are not shown); when enabled, errors take priority over the steady-on state. Currently: off = disabled, on = enabled, 1 Hz blink = temperature sensor error while enabled (`TempSensorError`). To add an indication, add an error flag, a `LightState` value, a branch in `deriveLightState()`, and a case in `updateLight()`.
- LCD (1602A, `hd44780_pinIO`, 4-bit, RW tied to GND; RS=3, E=5, D4=7, D5=11, D6=12, D7=13): `updateDisplay()` shows setpoint, temperature and `PidOutput`. It refreshes every `DisplayIntervalMs` (250 ms), writing one row per half-cycle (~1.5 ms each) and skipping `Compute` half-cycles so the DHT read and LCD writes never share a half-cycle.
- Tuning gains `Kp/Ki` are globals near the top, with commented Ziegler–Nichols notes and observed oscillation periods.

## Known quirks

- Conditions use bitwise `&`/`|` on bools (e.g. `Enable & cycleOn`); this works because the operands are `bool`.
- Gain screens (`updateGainDisplay()`) show the gain on row 0 and temperature/PID output on row 1, padded with `padLcdRow()`; the LCD is cleared when the screen or `Enable` changes.
