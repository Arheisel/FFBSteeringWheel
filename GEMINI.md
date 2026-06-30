# FFB Steering Wheel — Project Rules

## Project Overview

Firmware for a Force Feedback (FFB) steering wheel controller targeting Euro Truck Simulator 2. Runs on a Raspberry Pi Pico (RP2040, dual-core Cortex-M0+, no FPU, 264KB RAM, 2MB Flash). Implements USB HID with PID (Physical Interface Device) force feedback over TinyUSB.

## Hardware

- **Motor:** 775 DC motor via BTS7960 H-Bridge (20kHz PWM). Pins: EN (bridged), LPWM, RPWM.
- **Encoder:** AS5600 magnetic encoder on I2C at 100kHz. 1:2 gear ratio (8192 counts/wheel rev). CONF register initialized on boot for low-latency operation (PM=00, SF=10, FTH=100, WD=0).
- **Buttons:** 16 buttons via 2× HCF4021B shift registers on SPI at 100kHz (pull-up: 0 = pressed).
- **Pedals:** 2 analog pedals (0–3V) on ADC (compensated ratiometrically via a VBUS reference on ADC2).
- **LED:** Single LED for status/error flash codes.

**Peripheral mapping:** I2C instance, SPI instance, and ADC channels are derived at compile time from pin numbers in `config.h` using RP2040 GPIO function table formulas. Source files resolve these via `I2C_PORT` / `SPI_PORT` macros — no hardcoded peripheral instances.


## Dual-Core Architecture

```
Core 0 (main.cpp)                    Core 1 (core1_entry.cpp)
├── USB HID (TinyUSB) >1kHz          ├── 1ms alarm → I2C DMA read (AS5600)
├── Decoupled input polling:         ├── DMA ISR → AS5600Parser → FFBProcessor → MotorControl
│   ├── Pedals at 2000Hz (ADC)       ├── Calibration (blocking, runs before ISR enabled)
│   ├── Buttons at 500Hz (SPI DMA)   └── Background watchdog loop
│   └── LED status at 100Hz
├── USB HID reports at 1000Hz
├── Boot button wait & flash cal
└── Flash storage (flash_safe_execute)

     SharedState (shared_state.h)
     ├── SensorState      — Core 1 writes (atomics), Core 0 reads
     ├── EffectState       — Core 0 writes (spinlock), Core 1 reads
     ├── CalibrationLUTs   — Core 1 writes during calibration only
     ├── StatusState       — Both cores write (atomic CAS), tracks min display cycles
     └── Inputs            — Core 0 writes (atomics): buttons, pedals, center_offset
     └── DebugSerial       — AGC read request flag and ring buffer for error logs
```

### Data Flow

1. `FFBProcessor` (Physics/Force) → computes a force value in `-10000..+10000`
2. `SharedState` (Memory) → cross-core communication via atomics and spinlocks
3. `MotorControl` (Hardware) → maps force to raw PWM, applies stall governor + protection envelope + static friction compensation

### Boot Sequence

```
──────────────────                           ──────────────────────
1. board_init, stdio_init_all                (waiting to be launched)
2. Init LED, buttons, pedals, USB HID
3. Load flash calibration data
4. Wait for button press:                    
   - Short press → Normal Boot
   - Long press / no flash → Run Calibration 

If Normal Boot:
   Core 0 launches Core 1 ────────────────→ 5. Init I2C, motor, parser
                                            6. Apply flash center offset
                                            7. Init FFBProcessor
                                            8. Apply friction compensation
                                            9. Enable DMA IRQ, create alarm
                                           10. Start 1ms alarm + first I2C read
                                           11. Enter watchdog loop

If Run Calibration:
   Core 0 instantiates I2C, motor, parser
   Core 0 runs motor sweeps and captures center
   Core 0 runs pedal calibration
   Core 0 saves all data to flash
   Core 0 triggers watchdog reboot
```

**Critical:** Core 1 is not launched during the calibration phase. Core 0 handles the entire calibration locally since it has full memory and peripheral access. After saving data, it reboots into normal mode.

### Core 0 Main Loop Timing

- `tud_task()` runs every iteration (>1kHz USB polling)
- Pedal sampling (ADC) at 2000Hz (every 0.5ms)
- Button sampling (SPI DMA) at 500Hz (every 2ms) — provides 8ms debounce window (4-read)
- LED status updates at 100Hz (every 10ms)
- USB HID input reports at 1000Hz (every 1ms)
- `sleep_us(250)` per iteration to prevent pegging Core 0

## Core Development Rules

### 1. Integer Math & Raw Units Only

The RP2040 has no FPU. **Do not use floats.** All physical quantities must be expressed in raw hardware units:
- Angles: raw encoder counts (4096/encoder rev, 8192/wheel rev)
- PWM: raw duty cycle values (0 to `PWM_WRAP`, currently 6249)
- Force: normalized integer range `-10000..+10000`
- Velocity: raw encoder counts per millisecond

Convert all human-readable concepts (degrees, RPM, percentages) into `constexpr` raw units in `config.h`.

### 2. Memory Safety

- **Single writer per data path.** Each shared variable has exactly one core that writes to it.
- **Core 1 → Core 0:** Use `std::atomic` (no lock needed for single-writer scalars on Cortex-M0+).
- **Core 0 → Core 1:** Use hardware spinlocks (`spin_lock_blocking` / `spin_unlock`) for multi-field structs like `EffectState`.
- **Inter-core commands:** Use Pico SDK FIFO queues (`multicore_fifo_push/pop_blocking`).
- **LED status (both cores):** Use `compare_exchange_weak` for priority-based atomic updates.
- **Pass by Reference:** Always pass `SharedState` (and other guaranteed-to-exist objects) by reference (`SharedState&`) instead of by pointer (`SharedState*`) to enforce safety and avoid null checks.

### 3. Global Configuration (`config.h`)

All tunable parameters live in `config.h`. This includes:
- `constexpr` GPIO pin assignments for all peripherals
- PWM frequency, wrap values, and software limits
- Motor safety constants (dead-time, stall thresholds, PWM caps, protection envelope)
- Sensor parameters (I2C address, register map, CONF values, glitch thresholds)
- Calibration force levels and sweep distances
- Dynamic damping tuning parameters
- `enum class SystemStatus` defining all LED flash codes

**Do not scatter magic numbers in implementation files.** If a value is tunable, it belongs in `config.h`.

### 4. USB HID Descriptors

**Do NOT generate or modify** the HID report descriptor or PID FFB structs. Use `#include "usb_ffb_descriptors.h"` which contains the exact `hid_report_descriptor[]` and all `__attribute__((packed))` structs required for DirectInput compliance.

### 5. Build Verification

**The project must compile after every change.** After modifying any source file, build the project and fix all compilation errors before considering the change complete. A change that doesn't compile is not a valid change.

### 6. Documentation Currency

**Any changes to the overall architecture of the project must be reflected in this file (`GEMINI.md`).** This includes changes to data flow, boot sequence, safety invariants, timing, shared state structures, new files, or removed files. This file must always accurately describe the current state of the firmware.

### 7. Global Includes and Defines

**No `#include` or `#define` statements in the middle of files.** All preprocessor directives must be placed at the top of the file, immediately after the header comment block. This ensures dependencies and macros are clearly visible and globally scoped.

## Key Files

| File | Responsibility |
|------|---------------|
| `config.h` | All constants, pin assignments, enums |
| `shared_state.h` | Cross-core shared memory structures |
| `core1_entry.cpp/.h` | Core 1 boot, DMA ISR (FFB loop), watchdog |
| `main.cpp` | Core 0 boot, USB task, inputs, LED, boot/cal logic |
| `motor_control.cpp/.h` | BTS7960 driver, dead-time, stall governor, protection envelope, friction compensation |
| `ffb_processor.cpp/.h` | FFB effect engine (integer-only math), LUT-based damper/friction, dynamic damping |
| `i2c_dma.cpp/.h` | Non-blocking I2C DMA driver, AS5600 CONF init, bus recovery |
| `as5600_parser.cpp/.h` | Encoder position/velocity, glitch filter, dead-reckoning, desync detection |
| `calibration.cpp/.h` | Static friction + speed LUT calibration sweeps, center capture |
| `usb_hid.cpp/.h` | TinyUSB HID callbacks, PID report routing, effect management |
| `usb_descriptors.cpp` | USB device/config/HID/string descriptors, composite HID+CDC assembly |
| `usb_ffb_descriptors.h` | Raw PID HID descriptor + packed structs (DO NOT EDIT) |
| `debug_serial.cpp/.h` | USB CDC single-char command CLI for live status, error logs, and calibration dumps |
| `pedal_reader.cpp/.h` | ADC reading, VBUS ratiometric compensation, spike rejection, signed 16-bit scaling |
| `button_reader.cpp/.h` | SPI DMA reading, 4-read debounce |
| `led_controller.cpp/.h` | Non-blocking LED flash code state machine with minimum display guarantees |
| `flash_storage.cpp/.h` | Flash read/write with CRC32, `flash_safe_execute` for multicore safety |
| `tusb_config.h` | TinyUSB configuration (endpoint sizes, HID buffer) |

## Safety Invariants

1. **`MotorControl` is the sole owner of the BTS7960 pins.** No other code may touch PWM or EN GPIOs.
2. **Dead-time insertion** is mandatory on every direction change to prevent H-bridge shoot-through. Direction memory is preserved across zero-force intervals so dead-time fires correctly on reversal.
3. **Stall protection governor** dynamically limits PWM based on a peak/continuous current-limiting model:
   - **Tuned limits:** peak duty cycle cap (`PEAK_STALL_PWM` at ~60% duty) and continuous duty cycle cap (`CONT_STALL_PWM` at ~30% duty).
   - **Time-based decay:** If commanded PWM exceeds `CONT_STALL_PWM`, the remaining peak time accumulator (`remaining_peak_time_us_`) decays towards 0 from `PEAK_FALLOFF_TIME_US` (10s).
   - **Linear envelope:** The actual allowed max PWM scales down linearly from peak to continuous limits based on the remaining peak time.
   - **Cool-down recovery:** When commanded PWM drops below `CONT_STALL_PWM`, the accumulator recovers at a reduced rate governed by `PEAK_RECOVERY_PENALTY` (3x penalty factor) until it resets to the full 10s.
4. **Protection envelope (soft speed limiter)** prevents dangerous wheel speeds. When velocity exceeds `VELOCITY_FADE_START` (~110 RPM), forward PWM fades linearly to 0 at `MAX_SAFE_VELOCITY` (~140 RPM). Braking/damping forces are not limited.
5. **Static friction compensation** adds a calibrated `cw_zero_pwm` / `ccw_zero_pwm` offset to all force commands so the motor breaks static friction at any non-zero force level.
6. **Electronic end-stops** in `FFBProcessor` apply a proportional reverse spring force when the wheel exceeds `MAX_HALF_ANGLE_COUNTS`, short-circuiting all other effect processing.
7. **Dynamic damping (overpower detection)** in `FFBProcessor`: if the wheel velocity exceeds the expected speed for the commanded force (from calibration LUTs + safety margin), a damping counter-force is applied to prevent the wheel from overshooting.
8. **I2C watchdog** uses a leaky-bucket accumulator to tolerate transient bus glitches before killing the motor:
   - Each watchdog timeout (DMA ISR hasn't fired within `I2C_WATCHDOG_TIMEOUT_US` = 5ms) adds **+5 penalty points**.
   - Once the accumulator reaches **50 points** (~10 consecutive timeouts, ~50ms), the motor is stopped and the `I2CWatchdogFired` LED code is set.
   - Each successful DMA loop reduces the accumulator by **-1 point** (~2ms per iteration = ~100ms to fully recover).
   - On full recovery (accumulator reaches 0), the `I2CWatchdogFired` LED and error flag are cleared.
   - On every timeout, regardless of threshold, the 1ms alarm is cancelled, `reset_bus()` is called (SCL bit-bang + peripheral reinit + AS5600 CONF rewrite), and the alarm is re-armed. This prevents `start_read()` racing with the reset.
9. **Sensor error tolerance:** Transient I2C/EMI dropouts (missing MD bit) are ignored for up to `MAGNET_ERROR_TOLERANCE_FRAMES`. If the error persists, the DMA ISR stops the motor, skips FFB processing, logs to the debug buffer, and sets the LED error code.
10. **LED visibility guarantees:** Transient status codes are guaranteed to flash for at least `LED_MIN_DISPLAY_CYCLES` complete sequences before a self-clearing condition can turn the LED back to solid ON.
11. **Glitch filter with dead-reckoning:** If the AS5600 reports an impossible position jump (delta > `MAX_PHYSICAL_DELTA`), the parser extrapolates position from the last known velocity instead of accepting the bad data. After 10 consecutive impossible jumps, a fatal `EncoderDesync` error stops the motor and halts the I2C loop.
12. **Recovery desync detection:** On the first read after an I2C watchdog recovery, if the delta exceeds `MAX_PHYSICAL_DELTA`, a fatal `DesyncAfterRecovery` error is raised rather than trusting potentially stale data.
13. **Flash writes** are performed exclusively during the calibration phase when Core 1 is NOT running. The `flash_safe_execute` wrapper is used with a dynamic check, bypassing the multicore locks if Core 1 is inactive.
14. **Error Log Ring Buffer:** When the buffer is full, the write operation advances the read index (`g_error_log_read_idx`) to drop the oldest entry, maintaining a proper circular log.

## Timing Constraints

- **1ms:** I2C DMA read cycle (alarm-triggered)
- **5ms:** Watchdog timeout threshold (`I2C_WATCHDOG_TIMEOUT_US`) — triggers bus recovery attempt
- **~50ms:** Leaky-bucket threshold before motor kill (10 consecutive watchdog timeouts)
- **0.5ms / 2ms:** Decoupled input sampling intervals (pedals at 2000Hz, buttons at 500Hz)
- **1ms:** USB HID input report rate (1000Hz)
- **50µs:** Dead-time between H-bridge direction changes
- **250µs:** Core 0 main loop sleep (keeps `tud_task()` responsive at >1kHz)
- Use `time_us_64()` for absolute timestamps and long-duration calculations. For short-duration intervals and elapsed-time deltas, 32-bit `time_us_32()` is acceptable as unsigned wrap-around math handles overflow correctly.

