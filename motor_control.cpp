// =========================================================================
// Motor Control — BTS7960 Driver
// =========================================================================
// Manages the BTS7960 H-Bridge via hardware PWM.
// Features:
// - Hardware PWM (20kHz, phase-correct preferred but edge-aligned used here)
// - Non-blocking dead-time delay on direction change
// - Stall protection governor to prevent burning out the driver
// - Static friction compensation (from calibration)
// =========================================================================

#include "motor_control.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/time.h"

void MotorControl::init()
{
    // Configure PWM pins
    gpio_set_function(PIN_PWM_LPWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWM_RPWM, GPIO_FUNC_PWM);

    uint slice_l = pwm_gpio_to_slice_num(PIN_PWM_LPWM);
    uint slice_r = pwm_gpio_to_slice_num(PIN_PWM_RPWM);

    // Assuming LPWM and RPWM might be on different slices, configure both.
    // Ideally they are on the same slice (e.g. 6=3A, 7=3B).
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f); // 125MHz
    pwm_config_set_wrap(&config, PWM_WRAP);

    pwm_init(slice_l, &config, true);
    if (slice_l != slice_r)
    {
        pwm_init(slice_r, &config, true);
    }

    pwm_set_gpio_level(PIN_PWM_LPWM, 0);
    pwm_set_gpio_level(PIN_PWM_RPWM, 0);

    // Configure EN pin
    gpio_init(PIN_PWM_EN);
    gpio_set_dir(PIN_PWM_EN, GPIO_OUT);
    gpio_put(PIN_PWM_EN, 0); // Disable initially

    last_active_dir_ = Direction::OFF;
}

void MotorControl::apply_calibration(const CalibrationState &cal_state)
{
    cw_zero_pwm_ = cal_state.cw_zero_pwm.load(std::memory_order_relaxed);
    cw_active_range_ = PEAK_STALL_PWM - cw_zero_pwm_;
    ccw_zero_pwm_ = cal_state.ccw_zero_pwm.load(std::memory_order_relaxed);
    ccw_active_range_ = PEAK_STALL_PWM - ccw_zero_pwm_;
    force_scale_percent_ = cal_state.force_scale_percent.load(std::memory_order_relaxed);
    friction_fade_force_ = cal_state.friction_fade_force.load(std::memory_order_relaxed);
    dynamic_force_ = 10000 - friction_fade_force_;
}

void MotorControl::set_force(int16_t force, int32_t velocity)
{
    if (force == 0)
    {
        stop();
        return;
    }

    Direction dir = (force > 0) ? Direction::CW : Direction::CCW;
    uint16_t abs_force = (force > 0) ? force : -force;

    if (force_scale_percent_ != 100)
    {
        // Artificial "punch" boost to compress dynamic range and make weak forces feel stronger
        abs_force = static_cast<uint16_t>((static_cast<uint32_t>(abs_force) * force_scale_percent_) / 100);
    }

    if (abs_force > 10000)
        abs_force = 10000;

    // Determine the zero PWM offset based on direction
    uint16_t zero_pwm = (dir == Direction::CW) ? cw_zero_pwm_ : ccw_zero_pwm_;

    // Get the maximum safe PWM for this exact velocity
    uint16_t safe_max_pwm = get_safe_max_pwm(velocity);

    // Scale the requested force (0..10000) into the active safe range (zero_pwm..safe_max_pwm)
    uint16_t pwm = 0;

    if (safe_max_pwm <= zero_pwm)
    {
        pwm = (abs_force * safe_max_pwm) / friction_fade_force_;
    }
    else if (abs_force < friction_fade_force_)
    {
        // Static friction compensation
        pwm = (abs_force * zero_pwm) / friction_fade_force_;
    }
    else
    {
        // Dynamic force scaling, starting the scale at friction_fade_force
        uint16_t active_range = (dir == Direction::CW) ? cw_active_range_ : ccw_active_range_;
        pwm = zero_pwm + (((abs_force - friction_fade_force_) * active_range) / dynamic_force_);
    }

    // Ensure we never go above max safe PWM
    if (pwm > safe_max_pwm)
        pwm = safe_max_pwm;

    // Apply directly, bypassing set_pwm since we already scaled to safe limits
    apply_pwm(static_cast<uint16_t>(pwm), dir);
}

void MotorControl::set_pwm(uint16_t pwm, Direction dir, int32_t velocity)
{
    if (pwm == 0 || dir == Direction::OFF)
    {
        stop();
        return;
    }

    uint16_t max_allowed_pwm = get_safe_max_pwm(velocity);

    if (pwm > max_allowed_pwm)
    {
        pwm = max_allowed_pwm;
    }

    apply_pwm(pwm, dir);
}

uint16_t MotorControl::get_safe_max_pwm(int32_t velocity)
{

    uint16_t max_allowed_pwm = PEAK_STALL_PWM;

    if (remaining_peak_time_us_ <= 0)
    {
        max_allowed_pwm = CONT_STALL_PWM;
    }
    else if (remaining_peak_time_us_ >= PEAK_FALLOFF_TIME_US)
    {
        max_allowed_pwm = PEAK_STALL_PWM;
    }
    else
    {
        max_allowed_pwm = static_cast<uint16_t>(CONT_STALL_PWM + (static_cast<int64_t>(remaining_peak_time_us_) * (PEAK_STALL_PWM - CONT_STALL_PWM)) / PEAK_FALLOFF_TIME_US);
    }

    // --- Hardware Safety: Max Velocity Fading (Protection Envelope) ---
    // Fades out motor assistance if wheel is spinning too fast, protecting driver
    int32_t abs_velocity = (velocity >= 0) ? velocity : -velocity;

    if (abs_velocity > VELOCITY_FADE_START_CPS)
    {
        if (abs_velocity >= MAX_SAFE_VELOCITY_CPS)
        {
            max_allowed_pwm = 0;
        }
        else
        {
            int32_t overspeed = abs_velocity - VELOCITY_FADE_START_CPS;
            int32_t fade_range = MAX_SAFE_VELOCITY_CPS - VELOCITY_FADE_START_CPS;
            int32_t fade_factor_num = fade_range - overspeed;
            max_allowed_pwm = static_cast<uint16_t>((static_cast<uint32_t>(max_allowed_pwm) * fade_factor_num) / fade_range);
        }
    }

    return max_allowed_pwm;
}

void MotorControl::update_stall_time(uint16_t pwm)
{
    uint64_t now = time_us_64();
    int32_t elapsed_us = static_cast<int32_t>(now - last_stall_time_us_);
    last_stall_time_us_ = now;

    // Quick and dirty error check
    if (elapsed_us < 0) return;

    if (pwm <= CONT_STALL_PWM)
    {
        if (remaining_peak_time_us_ == PEAK_FALLOFF_TIME_US)
            return;

        remaining_peak_time_us_ += elapsed_us / PEAK_RECOVERY_PENALTY;

        if (remaining_peak_time_us_ > PEAK_FALLOFF_TIME_US || remaining_peak_time_us_ < 0)
            remaining_peak_time_us_ = PEAK_FALLOFF_TIME_US;
    }
    else
    {
        if (remaining_peak_time_us_ == 0)
            return;

        remaining_peak_time_us_ -= elapsed_us;

        if (remaining_peak_time_us_ < 0)
            remaining_peak_time_us_ = 0;
    }
}

void MotorControl::apply_pwm(uint16_t pwm, Direction dir)
{
    update_stall_time(pwm);

    if (dir == Direction::OFF)
    {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        gpio_put(PIN_PWM_EN, 0);
        return;
    }

    if (dir == Direction::BRAKE)
    {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        gpio_put(PIN_PWM_EN, 1);
        return;
    }

    if (dir != last_active_dir_)
    {
        // ---- Dead-Time Insertion ----
        // Before changing direction, turn both off and wait to prevent shoot-through
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);

        // Block tightly for DEAD_TIME_US (typically 50us)
        busy_wait_us_32(DEAD_TIME_US);
    }

    last_active_dir_ = dir;

    // Apply new duty cycle
    if (dir == Direction::CW)
    {
        pwm_set_gpio_level(PIN_PWM_LPWM, pwm);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
    }
    else
    {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, pwm);
    }
    gpio_put(PIN_PWM_EN, 1);
}
