#pragma once
#include <cstdint>
#include "config.h"

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

class FlashStorage {
public:
    // Load data from flash. Returns true if data was valid.
    bool load(FlashCalibrationData& out_data);

    // Save data to flash. Uses flash_safe_execute for multicore safety if core1_running is true.
    bool save(const FlashCalibrationData& data, bool core1_running = true);

private:
    uint32_t calculate_crc(const FlashCalibrationData& data) const;
};
