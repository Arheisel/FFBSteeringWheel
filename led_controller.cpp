// =========================================================================
// LED Status Controller — Non-blocking flash code display
// =========================================================================
// Solid ON during normal operation.
// Flash codes: N blinks (200ms ON, 500ms OFF), then 2s pause.
// Priority: highest SystemStatus enum value wins.
// =========================================================================

#include "led_controller.h"
#include "shared_state.h"
#include "hardware/gpio.h"
#include "pico/time.h"

void LEDController::init(SharedState &state)
{
    status_state_ptr_ = &state.led_status;
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1); // Start with LED on (normal)
    phase_ = FlashPhase::IDLE;
}

void LEDController::sleep_ms(uint32_t ms)
{
    while (ms > 0)
    {
        update();
        uint32_t step = (ms > LED_UPDATE_INTERVAL_MS) ? LED_UPDATE_INTERVAL_MS : ms;
        ::sleep_ms(step); // Use :: to call the global Pico SDK function, not ourselves!
        ms -= step;
    }
}

void LEDController::update()
{
    uint64_t now = time_us_64();

    // Read current status code
    uint8_t code = 0;
    if (status_state_ptr_)
    {
        code = static_cast<uint8_t>(status_state_ptr_->get());
    }

    if (code == static_cast<uint8_t>(SystemStatus::RapidFlash))
    {
        if (code != current_code_)
        {
            current_code_ = code;
            phase_start_us_ = now;
            rapid_flash_state_ = true;
            gpio_put(PIN_LED, 1);
        }
        else if (now - phase_start_us_ >= LED_RAPID_FLASH_MS * 1000ULL)
        {
            rapid_flash_state_ = !rapid_flash_state_;
            gpio_put(PIN_LED, rapid_flash_state_ ? 1 : 0);
            phase_start_us_ = now;
        }
        return; // Skip normal flash sequences
    }

    // If status is Normal (0), keep LED solid on
    if (code == 0)
    {
        gpio_put(PIN_LED, 1);
        phase_ = FlashPhase::IDLE;
        current_code_ = 0;
        return;
    }

    // If the code changed, restart the flash sequence
    if (code != current_code_)
    {
        current_code_ = code;
        flash_count_ = code;
        flashes_done_ = 0;
        phase_ = FlashPhase::FLASH_ON;
        phase_start_us_ = now;
        gpio_put(PIN_LED, 1);
        return;
    }

    // State machine for flashing
    uint64_t elapsed_us = now - phase_start_us_;

    switch (phase_)
    {
    case FlashPhase::IDLE:
        // Shouldn't get here if code != 0, but start fresh
        phase_ = FlashPhase::FLASH_ON;
        phase_start_us_ = now;
        flashes_done_ = 0;
        gpio_put(PIN_LED, 1);
        break;

    case FlashPhase::FLASH_ON:
        if (elapsed_us >= LED_FLASH_ON_MS * 1000ULL)
        {
            gpio_put(PIN_LED, 0);
            flashes_done_++;
            if (flashes_done_ >= flash_count_)
            {
                // All flashes done, enter pause
                phase_ = FlashPhase::PAUSE;
            }
            else
            {
                phase_ = FlashPhase::FLASH_OFF;
            }
            phase_start_us_ = now;
        }
        break;

    case FlashPhase::FLASH_OFF:
        if (elapsed_us >= LED_FLASH_OFF_MS * 1000ULL)
        {
            gpio_put(PIN_LED, 1);
            phase_ = FlashPhase::FLASH_ON;
            phase_start_us_ = now;
        }
        break;

    case FlashPhase::PAUSE:
        if (elapsed_us >= LED_PAUSE_MS * 1000ULL)
        {
            // One complete flash cycle finished — decrement the minimum display counter
            if (status_state_ptr_)
            {
                status_state_ptr_->decrement_display_cycle();
            }

            // Restart the flash sequence
            flashes_done_ = 0;
            phase_ = FlashPhase::FLASH_ON;
            phase_start_us_ = now;
            gpio_put(PIN_LED, 1);
        }
        break;
    }
}
