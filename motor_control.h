#pragma once
#include <cstdint>
#include "config.h"
#include "shared_state.h"

class MotorControl
{
public:
    enum class Direction : uint8_t
    {
        OFF = 0,
        CW = 1,
        CCW = 2,
        BRAKE = 3
    };

    void init();

    void apply_calibration(const CalibrationState &cal_state);

    // Set the target PWM and direction.
    // Handles dead-time, friction compensation, and stall protection.
    // Set physical hardware PWM (-10000 to +10000 scaled internally to FORWARD_MAX_PWM)
    void set_force(int16_t force, int32_t velocity);

    // Set raw PWM for calibration, with stall governor logic
    void set_pwm(uint16_t pwm, Direction dir, int32_t velocity);

    // Immediate stop (coast)
    void stop() { apply_pwm(0, Direction::OFF); }

    // Active stop (short terminals)
    void brake() { apply_pwm(0, Direction::BRAKE); }

private:
    uint16_t cw_zero_pwm_ = 0;
    uint16_t cw_active_range_ = PEAK_STALL_PWM;
    uint16_t ccw_zero_pwm_ = 0;
    uint16_t ccw_active_range_ = PEAK_STALL_PWM;
    uint16_t force_scale_percent_ = DEFAULT_FORCE_SCALE_PERCENT;
    uint16_t friction_fade_force_ = DEFAULT_FRICTION_FADE_FORCE;
    uint16_t dynamic_force_ = 10000 - friction_fade_force_;

    Direction last_active_dir_ = Direction::OFF;
    uint32_t last_stall_time_us_ = 0;   
    uint32_t remaining_peak_time_us_ = PEAK_FALLOFF_TIME_US;

    // Gets the hardware limits for the current speed
    uint16_t get_safe_max_pwm(int32_t velocity);

    void update_stall_time(uint16_t pwm);

    void apply_pwm(uint16_t pwm, Direction dir);
};
