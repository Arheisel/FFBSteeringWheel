#pragma once
#include <cstdint>
#include "config.h"
#include "shared_state.h"

class FlashStorage {
public:
    // Load data from flash into the CalibrationState. Returns true if data was valid.
    bool load(CalibrationState& out_state);

    // Save data from the CalibrationState to flash. Uses flash_safe_execute for multicore safety if core1_running is true.
    bool save(const CalibrationState& state, bool core1_running = true);

private:
    struct FlashCalibrationData {
        uint32_t magic;             // Must be 0xFEEDFACE
        uint8_t version;           // Version number for future upgrades
        int32_t  center_position;   // Absolute raw encoder counts
        uint16_t accel_min;         // Pedal min ADC
        uint16_t accel_max;         // Pedal max ADC
        uint16_t brake_min;
        uint16_t brake_max;
        
        // Motor Calibration LUTs
        uint16_t cw_zero_pwm;
        uint16_t ccw_zero_pwm;
        uint32_t  cw_speed[CAL_FORCE_LEVEL_COUNT];
        uint32_t  ccw_speed[CAL_FORCE_LEVEL_COUNT];
        
        uint16_t  wheel_angle_deg;
        uint16_t  system_damper_strength;
        uint16_t  forward_max_pwm;
        uint16_t  force_scale_percent;
        uint16_t  friction_fade_force;

        uint32_t crc32;             // Integrity check
    };

    bool load_internal(FlashCalibrationData& out_data);
    bool save_internal(const FlashCalibrationData& data, bool core1_running);
    uint32_t calculate_crc(const FlashCalibrationData& data) const;
};
