// =========================================================================
// AS5600 Parser — Position, Velocity & Error Detection
// =========================================================================
// Processes raw I2C reads from the AS5600 magnetic encoder.
// - Early exit on hardware errors (MH, ML, MD)
// - Handles 12-bit wrap-around with 1:2 gear ratio
// - Calculates velocity, filters impossible physics jumps
// =========================================================================

#include "as5600_parser.h"
#include "shared_state.h"
#include "pico/time.h"

void AS5600Parser::init()
{
    accumulated_position_ = 0;
    velocity_cps_ = 0;
    filtered_velocity_cps_ = 0;
    turn_count_ = 0;
    last_raw_angle_ = 0;
    first_read_ = true;
    error_flags_ = 0;
    last_time_us_ = time_us_64();
}

bool AS5600Parser::update(uint8_t status_reg, uint16_t raw_angle, bool is_recovery)
{
    // ---- EARLY EXIT: Hardware Error Check ----
    // MH (bit 3) = magnet too strong
    // ML (bit 4) = magnet too weak
    // MD (bit 5) = magnet detected
    error_flags_ = 0;

    if (status_reg & AS5600_STATUS_MH)
    {
        error_flags_ |= SensorState::ERR_MAGNET_HIGH;
    }
    if (status_reg & AS5600_STATUS_ML)
    {
        error_flags_ |= SensorState::ERR_MAGNET_LOW;
    }
    if (!(status_reg & AS5600_STATUS_MD))
    {
        error_flags_ |= SensorState::ERR_MAGNET_MISSING;
    }

    if (error_flags_ & SensorState::ERR_MAGNET_MISSING)
    {
        // Fatal hardware error (Magnet Missing) — do NOT update position or velocity.
        // We allow the frame to process if only MH or ML warnings are present,
        // because the motor's own magnetic field can trigger them at high PWM.
        return false;
    }

    // ---- Parse Position ----
    // Mask to 12 bits (0..4095)
    raw_angle &= 0x0FFF;

    uint64_t now = time_us_64();

    if (first_read_)
    {
        last_raw_angle_ = raw_angle;
        last_time_us_ = now;
        first_read_ = false;

        // At boot, we ALWAYS assume the wheel is at 0 turns relative to the center.
        // We find the shortest path to the center to set the initial position.
        // If center is 4090 and we read 10, the delta is +16, so turn_count_ must be 1.
        int32_t raw_diff = static_cast<int32_t>(raw_angle) - center_offset_;
        if (raw_diff > 2048)
        {
            turn_count_ = -1;
        }
        else if (raw_diff < -2048)
        {
            turn_count_ = 1;
        }
        else
        {
            turn_count_ = 0;
        }

        accumulated_position_ = (turn_count_ * 4096) + static_cast<int32_t>(raw_angle) - center_offset_;
        return true;
    }

    // Compute raw delta and check for wraps
    uint64_t dt_us = now - last_time_us_;
    if (dt_us == 0)
        dt_us = 1; // Prevent division by zero

    int32_t wraps = 0;
    int32_t delta = 0;

    if (dt_us > 0)
    {
        // Determine shortest path for wrapped values
        delta = static_cast<int32_t>(raw_angle) - static_cast<int32_t>(last_raw_angle_);
        if (delta > ENCODER_COUNTS_PER_REV / 2)
        {
            delta -= ENCODER_COUNTS_PER_REV;
            wraps = -1;
        }
        else if (delta < -(ENCODER_COUNTS_PER_REV / 2))
        {
            delta += ENCODER_COUNTS_PER_REV;
            wraps = 1;
        }

        // Calculate instantaneous velocity to check for physically impossible jumps
        // CRITICAL: dt_us must be cast to int64_t. Otherwise, dividing a negative int64_t
        // by a uint64_t promotes the negative number to a massive positive uint64_t!
        int32_t inst_velocity_cps = static_cast<int32_t>((static_cast<int64_t>(delta) * 1000000) / static_cast<int64_t>(dt_us));

        // ---- Filter Impossible Physics Jumps ----
        if (inst_velocity_cps > MAX_PHYSICAL_DELTA_CPS || inst_velocity_cps < -MAX_PHYSICAL_DELTA_CPS)
        {
            if (is_recovery)
            {
                error_flags_ |= SensorState::ERR_RECOVERY_DESYNC;
                return false;
            }

            desync_counter_++;
            if (desync_counter_ >= 10)
            {
                error_flags_ |= SensorState::ERR_DESYNC;
                return false;
            }

            // Impossible jump (likely I2C glitch)
            // Recover gracefully by dead-reckoning the delta using the last known FILTERED velocity.
            // CRITICAL: dt_us must be cast to int64_t to prevent promotion bugs here too!
            delta = static_cast<int32_t>((static_cast<int64_t>(filtered_velocity_cps_) * static_cast<int64_t>(dt_us)) / 1000000);

            // Extrapolate what the raw angle should have been, accounting for potential wraps
            int32_t total_raw = static_cast<int32_t>(last_raw_angle_) + delta;
            wraps = 0;
            while (total_raw >= ENCODER_COUNTS_PER_REV)
            {
                total_raw -= ENCODER_COUNTS_PER_REV;
                wraps++;
            }
            while (total_raw < 0)
            {
                total_raw += ENCODER_COUNTS_PER_REV;
                wraps--;
            }

            raw_angle = static_cast<uint16_t>(total_raw);
        }
        else
        {
            desync_counter_ = 0;
            velocity_cps_ = inst_velocity_cps;
        }
    }

    // Apply EMA smoothing to velocity for downstream consumers (motor governor)
    if (delta == 0)
    {
        zero_count_++;
        // If physically stopped for multiple reads (3ms), kill velocity immediately to prevent EMA lag from tricking the stall governor
        if (zero_count_ >= 3)
        {
            filtered_velocity_cps_ = 0;
        }
        else
        {
            filtered_velocity_cps_ += (velocity_cps_ - filtered_velocity_cps_) / VELOCITY_EMA_N;
        }
    }
    else
    {
        zero_count_ = 0;
        filtered_velocity_cps_ += (velocity_cps_ - filtered_velocity_cps_) / VELOCITY_EMA_N;
    }

    turn_count_ += wraps;
    accumulated_position_ = (turn_count_ * ENCODER_COUNTS_PER_REV) + static_cast<int32_t>(raw_angle) - center_offset_;

    last_raw_angle_ = raw_angle;
    last_time_us_ = now;

    return true;
}
