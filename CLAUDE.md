# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Single-file Arduino sketch ([heater-control.ino](heater-control.ino)) implementing integral-cycle (zero-cross) TRIAC control of a resistive heater, with PID temperature regulation from a DHT22 sensor. The goal is smooth, quiet heat output (no phase-angle chopping), e.g. for an under-desk heater. `heater control worksheet.xlsx` holds PID tuning/design calculations.

## Build / Upload

Built with the Arduino IDE; there are no tests. Required libraries, installed via the IDE Library Manager: `DHT` (Adafruit DHT sensor library), `OneButton`, `PID_v1` (Brett Beauregard). The target board is not pinned in the repo; assume an AVR board (Uno/Nano class).

To compile-check from the command line, use the arduino-cli bundled with the IDE (not on PATH):

```
"$LOCALAPPDATA/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe" compile --fqbn arduino:avr:uno --warnings all .
```

Serial monitor/plotter at **115200 baud**. Trace lines are `,sp:..,t:..,tRaw:..,PidOutput:..` formatted for the Arduino Serial Plotter. Sending a positive integer followed by a newline sets `Kp` live (applied on the next PID compute). Serial input is parsed non-blocking on purpose; don't use `Serial.parseInt()`, since its timeout would stall zero-cross detection.

## Architecture

Everything runs in `loop()`, synchronized to the AC zero cross by **polling** (not an interrupt) `PIN_DIMMER_ZC`:

1. Once >8000 µs have passed since the last zero cross and the ZC pin reads high, a new half-cycle (~8.3 ms at 60 Hz) begins.
2. The TRIAC output (`PIN_DIMMER_OUT`) is set **first**, using the `cycleOn` decision computed on the previous half-cycle — this keeps switching aligned to the zero cross.
3. The remaining time in the half-cycle is used for slower work: `computeCompute()` raises the `Compute` flag every `ComputeInvertalMs` (2.5 s, DHT22 minimum read interval ≈2 s); temperature read + filtering, PID, and serial trace only run when `Compute` is true.
4. `calculateCycleByDeltaSigma(PidOutput)` runs every half-cycle and converts the 0–100 % PID output into an on/off pulse density (Bresenham / sigma-delta accumulator).

Any work added to the loop must fit inside one half-cycle, or zero-cross timing (tracked in `ZcMicrosMin/Max/Average`) will slip.

Control details:
- Temperature is °F, rounded to 0.1, then smoothed by `iir3` (three cascaded first-order IIRs).
- PID output is limited to `[PID_MIN, PID_MAX]` = `[39, 100]`; anything ≤ `PID_MIN` is forced to 0 because the heater produces no useful heat below that level.
- Output mode, decided every half-cycle in `loop()`:
  - Disabled, or no valid temperature (`Temperature <= 0`, i.e. a DHT read failed): PID in `MANUAL`, output 0.
  - Low-temperature override (`deriveLowTemperatureOverride()`): PID in `MANUAL`, output `PID_MAX`, to warm up without integral windup. It latches on when more than `OVERRIDE_SET_ERROR` (2 °F) below setpoint and clears when within `OVERRIDE_RESET_ERROR` (1 °F).
  - Otherwise: PID in `AUTOMATIC`. On the switch from `MANUAL`, `PidOutput` is set to `PID_MIN` first, because `SetMode(AUTOMATIC)` seeds the PID integral from the current output.
- Buttons (OneButton, active-low with pullups): click = setpoint ±1 °F (clamped 55–85); long-press up = enable heater, long-press down = disable. `PIN_HEAT` mirrors the enable state.
- Tuning gains `Kp/Ki/Kd` are globals near the top, with commented Ziegler–Nichols notes and observed oscillation periods.

## Known quirks

- Conditions use bitwise `&`/`|` on bools (e.g. `Enable & cycleOn`); this works because the operands are `bool`.
- `updatePidOutput` only re-applies tunings when `Kp` changes.
