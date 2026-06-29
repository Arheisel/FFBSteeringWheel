#pragma once
#include "config.h"
#include "shared_state.h"
#include <cstdint>

class LEDController
{
public:
    void init(SharedState &state);
    void update();              // Call from main loop — non-blocking
    void sleep_ms(uint32_t ms); // Blocking sleep that continuously updates the LED

private:
    enum class FlashPhase : uint8_t
    {
        IDLE,      // Solid on (normal) or waiting for next flash sequence
        FLASH_ON,  // LED is ON for LED_FLASH_ON_MS
        FLASH_OFF, // LED is OFF for LED_FLASH_OFF_MS between flashes
        PAUSE,     // Inter-sequence pause (LED_PAUSE_MS)
    };

    FlashPhase phase_ = FlashPhase::IDLE;
    uint32_t phase_start_us_ = 0;
    uint8_t flash_count_ = 0;  // Number of flashes in current code
    uint8_t flashes_done_ = 0; // How many flashes completed so far
    uint8_t current_code_ = 0; // Currently displaying code

    LEDState *status_state_ptr_ = nullptr;
    bool rapid_flash_state_ = false;
};
