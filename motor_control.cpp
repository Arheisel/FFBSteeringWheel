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

void MotorControl::init() {
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
    if (slice_l != slice_r) {
        pwm_init(slice_r, &config, true);
    }

    pwm_set_gpio_level(PIN_PWM_LPWM, 0);
    pwm_set_gpio_level(PIN_PWM_RPWM, 0);

    // Configure EN pin
    gpio_init(PIN_PWM_EN);
    gpio_set_dir(PIN_PWM_EN, GPIO_OUT);
    gpio_put(PIN_PWM_EN, 0); // Disable initially

    current_direction_ = Direction::OFF;
}

void MotorControl::apply_calibration(const CalibrationState& cal_state) {
    forward_max_pwm_ = cal_state.forward_max_pwm.load(std::memory_order_relaxed);
    cw_zero_pwm_ = cal_state.cw_zero_pwm.load(std::memory_order_relaxed);
    cw_active_range_ = forward_max_pwm_ - cw_zero_pwm_;
    ccw_zero_pwm_ = cal_state.ccw_zero_pwm.load(std::memory_order_relaxed);
    ccw_active_range_ = forward_max_pwm_ - ccw_zero_pwm_;
    force_scale_percent_ = cal_state.force_scale_percent.load(std::memory_order_relaxed);
    friction_fade_force_ = cal_state.friction_fade_force.load(std::memory_order_relaxed);
    dynamic_force_ = 10000 - friction_fade_force_;
}

void MotorControl::set_force(int16_t force, int32_t velocity) {
    if (force == 0) {
        stop();
        return;
    }

    Direction dir = (force > 0) ? Direction::CW : Direction::CCW;
    uint16_t abs_force = (force > 0) ? force : -force;

    if (force_scale_percent_ != 100){
        // Artificial "punch" boost to compress dynamic range and make weak forces feel stronger
         abs_force = static_cast<uint16_t>((static_cast<uint32_t>(abs_force) * force_scale_percent_) / 100);
    }

    if (abs_force > 10000) abs_force = 10000;

    // Determine the zero PWM offset based on direction
    uint16_t zero_pwm = (dir == Direction::CW) ? cw_zero_pwm_ : ccw_zero_pwm_;

    // Get the maximum safe PWM for this exact velocity
    uint16_t safe_max_pwm = get_safe_max_pwm(dir, velocity);

    // Scale the requested force (0..10000) into the active safe range (zero_pwm..safe_max_pwm)
    uint16_t pwm = 0;

    if (safe_max_pwm <= zero_pwm){
        // Backdriving force scaling
        pwm = (abs_force * safe_max_pwm) / friction_fade_force_;
    } else if (abs_force < friction_fade_force_){
        // Static friction compensation
        pwm = (abs_force * zero_pwm) / friction_fade_force_;
    } else {
        // Dynamic force scaling, starting the scale at friction_fade_force
        uint16_t active_range = (dir == Direction::CW) ? cw_active_range_ : ccw_active_range_;
        pwm = zero_pwm + (((abs_force - friction_fade_force_) * active_range) / dynamic_force_);
    }

    // Ensure we never go above max safe PWM
    if (pwm > safe_max_pwm) pwm = safe_max_pwm;
    
    // Apply directly, bypassing set_pwm since we already scaled to safe limits
    apply_pwm(static_cast<uint16_t>(pwm), dir);
}

uint16_t MotorControl::get_safe_max_pwm(Direction dir, int32_t velocity) {
    if (dir == Direction::OFF) return 0;

    bool is_forward = (dir == Direction::CW && velocity > 0) || (dir == Direction::CCW && velocity < 0);
    bool is_stalled = (velocity == 0);
    int32_t abs_velocity = (velocity >= 0) ? velocity : -velocity;
    
    uint16_t max_allowed_pwm = forward_max_pwm_;
    
    if (is_stalled) {
        max_allowed_pwm = STALL_PWM_MAX;
    } else if (is_forward) {
        if (abs_velocity < FORWARD_VELOCITY_THRESHOLD_CPS) {
            // Linearly increase from STALL_PWM_MAX to forward_max_pwm
            int16_t range = forward_max_pwm_ - STALL_PWM_MAX;
            max_allowed_pwm = STALL_PWM_MAX + static_cast<uint16_t>((abs_velocity * range) / FORWARD_VELOCITY_THRESHOLD_CPS);
        } else {
            max_allowed_pwm = forward_max_pwm_;
        }

        // --- Hardware Safety: Max Velocity Fading (Protection Envelope) ---
        // Fades out motor assistance if wheel is spinning too fast, protecting driver
        if (abs_velocity > VELOCITY_FADE_START_CPS) {
            if (abs_velocity >= MAX_SAFE_VELOCITY_CPS) {
                max_allowed_pwm = 0;
            } else {
                int32_t overspeed = abs_velocity - VELOCITY_FADE_START_CPS;
                int32_t fade_range = MAX_SAFE_VELOCITY_CPS - VELOCITY_FADE_START_CPS;
                int32_t fade_factor_num = fade_range - overspeed;
                max_allowed_pwm = static_cast<uint16_t>((static_cast<uint32_t>(max_allowed_pwm) * fade_factor_num) / fade_range);
            }
        }
    } else {
        // Moving backwards (user fighting the motor)
        if (abs_velocity < BACKWARDS_VELOCITY_THRESHOLD_CPS) {
            // Linearly decrease from STALL_PWM_MAX to BACKWARDS_PWM_MAX
            int16_t range = STALL_PWM_MAX - BACKWARDS_PWM_MAX;
            max_allowed_pwm = STALL_PWM_MAX - static_cast<uint16_t>((abs_velocity * range) / BACKWARDS_VELOCITY_THRESHOLD_CPS);
        } else {
            max_allowed_pwm = BACKWARDS_PWM_MAX;
        }
    }

    return max_allowed_pwm;
}

void MotorControl::set_pwm(uint16_t pwm, Direction dir, int32_t velocity) {
    if (pwm == 0 || dir == Direction::OFF) {
        stop();
        return;
    }

    uint16_t max_allowed_pwm = get_safe_max_pwm(dir, velocity);

    if (pwm > max_allowed_pwm) {
        pwm = max_allowed_pwm;
    }

    apply_pwm(pwm, dir);
}

void MotorControl::apply_pwm(uint16_t pwm, Direction dir) {

    if (dir == Direction::OFF) {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        gpio_put(PIN_PWM_EN, 0);
        // Do NOT set current_direction_ = OFF. This remembers the last active direction
        // so if the next non-zero force is in the opposite direction, dead-time fires correctly!
        return;
    }

    if (dir == Direction::BRAKE) {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        gpio_put(PIN_PWM_EN, 1);
        return;
    }

    if (dir != current_direction_) {
        // ---- Dead-Time Insertion ----
        // Before changing direction, turn both off and wait to prevent shoot-through
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        
        // Block tightly for DEAD_TIME_US (typically 50us)
        busy_wait_us_32(DEAD_TIME_US);
    }
    
    current_direction_ = dir;

    // Apply new duty cycle
    if (dir == Direction::CW) {
        pwm_set_gpio_level(PIN_PWM_LPWM, pwm);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
    } else {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, pwm);
    }
    gpio_put(PIN_PWM_EN, 1);
}
