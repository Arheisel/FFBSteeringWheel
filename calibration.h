#pragma once
#include "shared_state.h"
#include "button_reader.h"
#include "pedal_reader.h"
#include "led_controller.h"
#include "flash_storage.h"

// Runs the full flash calibration logic (captures center, sweeps motor, calibrates pedals, saves to flash, reboots)
// Executed strictly on Core 0 before Core 1 is launched.
void run_full_calibration(SharedState& state, ButtonReader& buttons, PedalReader& pedals, LEDController& led, FlashStorage& flash);

// Runs only the LUT speed calibration logic (keeps center and zero pwm intact, calibrates speed LUTs, saves to flash, reboots)
// Executed strictly on Core 0 before Core 1 is launched.
void run_lut_calibration(SharedState& state, ButtonReader& buttons, LEDController& led, FlashStorage& flash);
