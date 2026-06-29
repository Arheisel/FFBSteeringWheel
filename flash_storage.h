#pragma once
#include <cstdint>
#include "config.h"
#include "shared_state.h"
#include "hardware/flash.h"

#define MAGIC_NUMBER 0xFEEDFACE
#define FLASH_DATA_VERSION 1

class FlashStorage
{
public:
    // Load data from flash into the CalibrationState. Returns true if data was valid.
    bool load(CalibrationState &out_state);

    // Save data from the CalibrationState to flash. Uses flash_safe_execute for multicore safety if core1_running is true.
    bool save(const CalibrationState &state, bool core1_running = true);

private:
    struct __attribute__((packed)) FlashCalibrationData
    {
        uint32_t magic{MAGIC_NUMBER};        // Must be 0xFEEDFACE
        uint8_t version{FLASH_DATA_VERSION}; // Version number for future upgrades

        int32_t center_position;
        uint16_t accel_min;
        uint16_t accel_max;
        uint16_t brake_min;
        uint16_t brake_max;

        // Motor Calibration
        uint16_t cw_zero_pwm;
        uint16_t ccw_zero_pwm;
        uint32_t cw_speed[CAL_FORCE_LEVEL_COUNT];
        uint32_t ccw_speed[CAL_FORCE_LEVEL_COUNT];

        uint16_t wheel_angle_deg;
        uint16_t system_damper_strength;
        uint16_t force_scale_percent;
        uint16_t friction_fade_force;

        uint8_t reserved_space[183]{0};

        uint32_t crc32; // Integrity check
    };

    static_assert(sizeof(FlashCalibrationData) == FLASH_PAGE_SIZE, "Struct size must be exactly 256 bytes.");

    bool load_internal(FlashCalibrationData &out_data);
    bool save_internal(const FlashCalibrationData &data, bool core1_running);
    uint32_t calculate_crc(const FlashCalibrationData &data) const;
};
