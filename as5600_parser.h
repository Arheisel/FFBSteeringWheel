#pragma once
#include <cstdint>
#include "config.h"

class AS5600Parser
{
public:
    void init();

    // Called from DMA interrupt with raw I2C data.
    // Returns true if data is valid (no hardware error).
    // On hardware error, flags the error and returns false.
    bool update(uint8_t status_reg, uint16_t raw_angle, bool is_recovery = false);

    int32_t get_position() const { return accumulated_position_; }
    int32_t get_velocity() const { return filtered_velocity_cps_; }
    int32_t get_absolute_raw() const { return (turn_count_ * ENCODER_COUNTS_PER_REV) + last_raw_angle_; }
    uint8_t get_error_flags() const { return error_flags_; }

    // Set the center offset
    void set_center(int32_t center)
    {
        center_offset_ = center;
        first_read_ = true;
    }

private:
    // Position tracking
    int32_t accumulated_position_ = 0; // Continuous position from center
    int32_t center_offset_ = 0;        // Flash-saved center position
    int32_t turn_count_ = 0;           // Number of full encoder wraps
    uint16_t last_raw_angle_ = 0;
    bool first_read_ = true;

    // Velocity calculation
    int32_t velocity_cps_ = 0;          // Raw instantaneous counts/sec (used for dead-reckoning)
    int32_t filtered_velocity_cps_ = 0; // EMA-smoothed counts/sec (used by motor governor)
    uint8_t zero_count_ = 0;            // Consecutive zero-velocity reads
    uint32_t last_time_us_ = 0;

    // Error state
    uint8_t error_flags_ = 0;
    uint8_t desync_counter_ = 0;
};
