// =========================================================================
// FFB Processor — Force Feedback Effect Calculation Engine
// =========================================================================
// Processes all active effects and returns a combined integer force.
// All calculations use integer math only — no floats.
// Force range: -10000 to +10000 (matches PID spec normalized range).
// =========================================================================

#include "ffb_processor.h"
#include "pico/time.h"

// =========================================================================
// Integer sine LUT — 91 entries for 0..90 degrees (1° steps)
// Values scaled to 0..10000
// =========================================================================

namespace {
    const int16_t sin_lut[91] = {
            0,   175,   349,   523,   698,   872,  1045,  1219,  1392,  1564,
        1736,  1908,  2079,  2250,  2419,  2588,  2756,  2924,  3090,  3256,
        3420,  3584,  3746,  3907,  4067,  4226,  4384,  4540,  4695,  4848,
        5000,  5150,  5299,  5446,  5592,  5736,  5878,  6018,  6157,  6293,
        6428,  6561,  6691,  6820,  6947,  7071,  7193,  7314,  7431,  7547,
        7660,  7771,  7880,  7986,  8090,  8192,  8290,  8387,  8480,  8572,
        8660,  8746,  8829,  8910,  8988,  9063,  9135,  9205,  9272,  9336,
        9397,  9455,  9511,  9563,  9613,  9659,  9703,  9744,  9781,  9816,
        9848,  9877,  9903,  9925,  9945,  9962,  9976,  9986,  9994,  9998,
        10000
    };
}

int32_t FFBProcessor::int_sin(uint32_t angle_centideg) {
    // Normalize to 0..35999
    angle_centideg = angle_centideg % 36000;

    // Convert centidegrees to degrees (integer, losing sub-degree precision)
    uint32_t deg = angle_centideg / 100;

    // Quadrant decomposition
    int32_t sign = 1;
    if (deg >= 180) {
        sign = -1;
        deg -= 180;
    }
    if (deg > 90) {
        deg = 180 - deg;
    }

    return sign * sin_lut[deg];
}

// =========================================================================
// Calibration
// =========================================================================

void FFBProcessor::apply_calibration(const CalibrationState& cal_state) {
    cal_lut_cw_ = &cal_state.cw_speed;
    cal_lut_ccw_ = &cal_state.ccw_speed;
    max_half_angle_counts_ = cal_state.max_half_angle_counts.load(std::memory_order_relaxed);
    system_damper_strength_ = cal_state.system_damper_strength.load(std::memory_order_relaxed);
}

// =========================================================================
// Main calculation
// =========================================================================

int16_t FFBProcessor::calculate(int32_t position, int32_t velocity, EffectState& effects) {
    int32_t total_force = 0;

    // ---- Electronic End-Stop (Early Exit) ----
    // If wheel is beyond physical limit, apply a proportional reverse spring
    // and skip ALL other effect processing.
    if (position > max_half_angle_counts_ || position < -max_half_angle_counts_) {
        int32_t overshoot = (position > 0)
            ? (position - max_half_angle_counts_)
            : (position + max_half_angle_counts_);

        // Proportional spring: force = -overshoot scaled to 10000 range
        // Scaling by 256 makes it extremely aggressive (hits a brick wall within ~11 degrees)
        int32_t force = (-overshoot * 10000) / 256;
        
        // Add viscous damping to prevent violent undamped oscillation (bouncing)
        // 200/1000 gives ~30% damping force at 15000 cps (15 counts/ms)
        force -= (velocity * 200) / 1000;
        
        if (force > 10000) force = 10000;
        if (force < -10000) force = -10000;

        return static_cast<int16_t>(force);
    }

    // ---- Check if actuators are enabled and not paused ----
    if (!effects.actuators_enabled || effects.device_paused) {
        return 0;
    }

    // ---- Accumulate forces from all active effects ----
    uint64_t now = time_us_64();

    int32_t scaled_pos = (position * 10000) / max_half_angle_counts_;
    int32_t scaled_vel = (velocity * 10000) / MAX_SAFE_VELOCITY_CPS;
    if (scaled_vel > 10000) scaled_vel = 10000;
    else if (scaled_vel < -10000) scaled_vel = -10000;

    // Take spinlock to safely read effects (Core 0 might be updating them via USB)
    uint32_t irq = spin_lock_blocking(effects.lock);

    for (uint8_t i = 0; i < MAX_EFFECTS; i++) {
        EffectSlot& e = effects.effects[i];
        if (e.state != EffectSlot::STATE_PLAYING) continue;

        // Calculate elapsed time in ms
        uint32_t elapsed_ms = static_cast<uint32_t>((now - e.start_time_us) / 1000);

        // Check duration (0xFFFF, 0x7FFF, or 0 = infinite)
        uint16_t duration = e.params.duration;
        if (duration != 0 && duration != 0xFFFF && duration != 0x7FFF && elapsed_ms > duration) {
            // Loop or expire
            if (e.loop_count == 0xFF) {
                // Infinite loop
                e.start_time_us = now;
                elapsed_ms = 0;
            } else if (e.loop_count > 1) {
                e.loop_count--;
                e.start_time_us = now;
                elapsed_ms = 0;
            } else {
                // Expired
                e.state = EffectSlot::STATE_ALLOCATED;
                continue;
            }
        }

        // Direction scaling: project force onto X-axis using USB PID / DirectInput standard.
        // 0 degrees = North (0 X-force)
        // 90 degrees = East (+X, full positive force)
        // Therefore, X-component = sin(θ).
        uint32_t dir_angle = static_cast<uint32_t>(e.params.directionX) * 36000 / 255;
        int32_t angle_ratio = int_sin(dir_angle);
        int32_t force = 0;

        switch (e.params.effectType) {
            case 1: force = calc_constant_force(e); break;              // Constant Force
            case 2: force = calc_ramp_force(e, elapsed_ms); break;      // Ramp Force
            case 3:                                                     // Square
            case 4:                                                     // Sine
            case 5:                                                     // Triangle
            case 6:                                                     // Sawtooth Up
            case 7: force = calc_periodic_force(e, elapsed_ms); break;  // Sawtooth Down
            case 8: force = calc_condition_force(e, scaled_pos, 0); break;// Spring (Position)
            case 9: {                                                   // Damper (Velocity)
                int32_t coeff_percent = calc_condition_force(e, scaled_vel, 0); 
                int32_t req_force = lookup_required_force(velocity);
                force = (coeff_percent * req_force) / 10000;
                break;
            }
            case 10: force = 0; break;                                  // Inertia (Unsupported)
            case 11: {                                                  // Friction (Velocity)
                int32_t coeff_percent = calc_condition_force(e, scaled_vel, 0);
                int32_t req_force = lookup_required_force(velocity);
                force = (coeff_percent * req_force) / 10000;
                break;
            }
            default: break;
        }

        if (force != 0) {
            if (e.params.effectType < 8 || e.params.effectType > 11) {
                force = apply_envelope(e, force, elapsed_ms);
                force = (force * angle_ratio) / 10000;
            }
            force = (force * e.params.gain) / 255;
            total_force += force;
        }
    }

    spin_unlock(effects.lock, irq);

    // Apply device gain, reversing direction beecause DirectInput seems to go against HID spec
    total_force = -(total_force * effects.device_gain) / 255;

    // ---- Firmware-Controlled Damper ----
    if (system_damper_strength_ > 0 && velocity != 0) {
        EffectSlot hw_e;
        hw_e.params.effectType = 9;
        hw_e.condition[0].positiveCoefficient = -system_damper_strength_;
        hw_e.condition[0].negativeCoefficient = -system_damper_strength_;
        hw_e.condition[0].positiveSaturation = 10000;
        hw_e.condition[0].negativeSaturation = 10000;
        hw_e.condition[0].deadBand = 0;
        hw_e.condition[0].cpOffset = 0;

        int32_t coeff_percent = calc_condition_force(hw_e, scaled_vel, 0); 
        int32_t req_force = lookup_required_force(velocity);
        total_force += (coeff_percent * req_force) / 10000;
    }

    // ---- Overpower Detection (Dynamic Damping) ----
    if (total_force != 0 && velocity != 0) {
        int32_t expected_vel = lookup_expected_speed(total_force);
        // Add a safety margin to avoid false positives from noise
        expected_vel += VELOCITY_MARGIN_CPS;
        
        if (total_force > 0 && velocity > expected_vel) {
            // User is throwing the wheel CW faster than the motor is pushing it
            int32_t excess = velocity - expected_vel;
            total_force -= (excess * DYNAMIC_DAMPING_FACTOR) / 1000;
        } 
        else if (total_force < 0 && velocity < -expected_vel) {
            // User is throwing the wheel CCW faster than the motor is pushing it
            int32_t excess = (-velocity) - expected_vel;
            total_force += (excess * DYNAMIC_DAMPING_FACTOR) / 1000;
        }
    }

    // Clamp to output range
    if (total_force > 10000) total_force = 10000;
    if (total_force < -10000) total_force = -10000;

    // Return unscaled output force
    return static_cast<int16_t>(total_force);
}

// =========================================================================
// Individual effect calculators
// =========================================================================

int32_t FFBProcessor::calc_constant_force(const EffectSlot& e) {
    // Magnitude: -10000..+10000 in the descriptor
    return static_cast<int32_t>(e.constant.magnitude);
}

int32_t FFBProcessor::calc_ramp_force(const EffectSlot& e, uint32_t elapsed_ms) {
    uint16_t duration = e.params.duration;
    if (duration == 0) return e.ramp.startMagnitude;

    int32_t start = static_cast<int32_t>(e.ramp.startMagnitude);
    int32_t end   = static_cast<int32_t>(e.ramp.endMagnitude);

    if (elapsed_ms >= duration) return end;

    // Linear interpolation: start + (end - start) * elapsed / duration
    return start + ((end - start) * static_cast<int32_t>(elapsed_ms)) / duration;
}

int32_t FFBProcessor::calc_periodic_force(const EffectSlot& e, uint32_t elapsed_ms) {
    int32_t  magnitude = static_cast<int32_t>(e.periodic.magnitude);
    int32_t  offset    = static_cast<int32_t>(e.periodic.offset);
    uint32_t phase     = static_cast<uint32_t>(e.periodic.phase);  // 0..35999 centidegrees
    uint32_t period    = static_cast<uint32_t>(e.periodic.period); // ms

    if (period == 0) return offset;

    // Calculate phase position within the period
    uint32_t phase_ms = (phase * period) / 36000;
    uint32_t t = (elapsed_ms + phase_ms) % period;

    int32_t force = 0;

    switch (e.params.effectType) {
        case 3: { // Square
            if (t < period / 2) {
                force = offset + magnitude;
            } else {
                force = offset - magnitude;
            }
            break;
        }

        case 4: { // Sine
            // angle = (t / period) * 36000 centidegrees
            uint32_t angle = (t * 36000) / period;
            force = offset + (magnitude * int_sin(angle)) / 10000;
            break;
        }

        case 5: { // Triangle
            int32_t half = static_cast<int32_t>(period / 2);
            int32_t ti = static_cast<int32_t>(t);
            if (half == 0) half = 1;
            if (ti < half) {
                // Rising: -magnitude to +magnitude
                force = offset - magnitude + (2 * magnitude * ti) / half;
            } else {
                // Falling: +magnitude to -magnitude
                force = offset + magnitude - (2 * magnitude * (ti - half)) / half;
            }
            break;
        }

        case 6: { // Sawtooth Up
            force = offset - magnitude +
                    (2 * magnitude * static_cast<int32_t>(t)) / static_cast<int32_t>(period);
            break;
        }

        case 7: { // Sawtooth Down
            force = offset + magnitude -
                    (2 * magnitude * static_cast<int32_t>(t)) / static_cast<int32_t>(period);
            break;
        }
    }

    return force;
}

int32_t FFBProcessor::calc_condition_force(const EffectSlot& e, int32_t metric, uint8_t axis) {
    if (axis >= 2) axis = 0;
    const auto& cond = e.condition[axis];

    int32_t cp_offset   = static_cast<int32_t>(cond.cpOffset);
    int32_t pos_coeff   = static_cast<int32_t>(cond.positiveCoefficient);
    int32_t neg_coeff   = static_cast<int32_t>(cond.negativeCoefficient);
    int32_t pos_sat     = static_cast<int32_t>(cond.positiveSaturation);
    int32_t neg_sat     = static_cast<int32_t>(cond.negativeSaturation);
    int32_t dead_band   = static_cast<int32_t>(cond.deadBand);

    int32_t force = 0;
    
    if (metric < (cp_offset - dead_band)) {
        // Negative side
        force = (neg_coeff * (metric - (cp_offset - dead_band))) / 10000;
        if (force < -neg_sat) force = -neg_sat;
    } else if (metric > (cp_offset + dead_band)) {
        // Positive side
        force = (pos_coeff * (metric - (cp_offset + dead_band))) / 10000;
        if (force > pos_sat) force = pos_sat;
    }
    // Inside dead band → force = 0

    // USB PID Standard: Force = Coefficient * Metric
    // If the game wants the force to resist movement (like a centering spring), 
    // it MUST provide a negative coefficient.
    return force;
}

int32_t FFBProcessor::apply_envelope(const EffectSlot& e, int32_t force,
                                     uint32_t elapsed_ms) {
    uint16_t duration = e.params.duration;
    uint32_t attack_time = e.envelope.attackTime;
    uint32_t fade_time   = e.envelope.fadeTime;
    int32_t  attack_level = static_cast<int32_t>(e.envelope.attackLevel);
    int32_t  fade_level   = static_cast<int32_t>(e.envelope.fadeLevel);

    // Normalize force magnitude for envelope scaling
    int32_t magnitude = (force >= 0) ? force : -force;

    if (elapsed_ms < attack_time && attack_time > 0) {
        // Attack phase: ramp from attack_level to magnitude
        int32_t level = attack_level +
            ((magnitude - attack_level) * static_cast<int32_t>(elapsed_ms)) /
            static_cast<int32_t>(attack_time);
        return (force >= 0) ? level : -level;
    }

    if (duration != 0xFFFF && duration != 0x7FFF &&
        fade_time > 0 && duration > fade_time &&
        elapsed_ms > (duration - fade_time)) {
        // Fade phase: ramp from magnitude to fade_level
        uint32_t fade_elapsed = elapsed_ms - (duration - fade_time);
        int32_t level = magnitude -
            ((magnitude - fade_level) * static_cast<int32_t>(fade_elapsed)) /
            static_cast<int32_t>(fade_time);
        return (force >= 0) ? level : -level;
    }

    return force;  // Sustain phase
}

int32_t FFBProcessor::lookup_expected_speed(int32_t force) const {

    bool is_cw = force > 0;
    auto* lut = is_cw ? *cal_lut_cw_ : *cal_lut_ccw_;
    if (!lut) return 0;

    int32_t abs_force = is_cw ? force : -force;
    
    // Find the two bracketing force levels and interpolate
    for (uint8_t i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        if (abs_force <= CAL_FORCE_LEVELS[i]) {
            if (i == 0) {
                if (CAL_FORCE_LEVELS[0] == 0) return 0;
                return (abs_force * lut[0].load(std::memory_order_relaxed)) / CAL_FORCE_LEVELS[0];
            }
            // Linear interpolation between lut[i-1] and lut[i]
            int32_t force_low  = CAL_FORCE_LEVELS[i - 1];
            int32_t force_high = CAL_FORCE_LEVELS[i];
            int32_t spd_low  = lut[i - 1].load(std::memory_order_relaxed);
            int32_t spd_high = lut[i].load(std::memory_order_relaxed);
            return spd_low + ((spd_high - spd_low) * (abs_force - force_low)) / (force_high - force_low);
        }
    }
    
    // Force exceeds the calibrated table, extrapolate using the last two points
    int32_t force_low  = CAL_FORCE_LEVELS[CAL_FORCE_LEVEL_COUNT - 2];
    int32_t force_high = CAL_FORCE_LEVELS[CAL_FORCE_LEVEL_COUNT - 1];
    int32_t spd_low  = lut[CAL_FORCE_LEVEL_COUNT - 2].load(std::memory_order_relaxed);
    int32_t spd_high = lut[CAL_FORCE_LEVEL_COUNT - 1].load(std::memory_order_relaxed);
    
    return spd_low + ((spd_high - spd_low) * (abs_force - force_low)) / (force_high - force_low);
}

int32_t FFBProcessor::lookup_required_force(int32_t velocity) const {
    if (velocity == 0) return 0;

    bool is_cw = velocity > 0;
    auto* lut = is_cw ? *cal_lut_cw_ : *cal_lut_ccw_;
    if (!lut) return 0;

    int32_t abs_vel = is_cw ? velocity : -velocity;

    // Find the two bracketing speed levels and interpolate the required force
    for (uint8_t i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        if (abs_vel <= lut[i].load(std::memory_order_relaxed)) {
            if (i == 0) {
                int32_t lut0 = lut[0].load(std::memory_order_relaxed);
                if (lut0 == 0) return CAL_FORCE_LEVELS[0];
                return (abs_vel * CAL_FORCE_LEVELS[0]) / lut0;
            }
            // Linear interpolation between lut[i-1] and lut[i]
            int32_t spd_low  = lut[i - 1].load(std::memory_order_relaxed);
            int32_t spd_high = lut[i].load(std::memory_order_relaxed);
            int32_t force_low  = CAL_FORCE_LEVELS[i - 1];
            int32_t force_high = CAL_FORCE_LEVELS[i];
            
            if (spd_high == spd_low) return force_high; // Prevent div by zero
            return force_low + ((force_high - force_low) * (abs_vel - spd_low)) / (spd_high - spd_low);
        }
    }
    
    // Velocity exceeds the calibrated table, extrapolate using the last two points
    int32_t spd_low  = lut[CAL_FORCE_LEVEL_COUNT - 2].load(std::memory_order_relaxed);
    int32_t spd_high = lut[CAL_FORCE_LEVEL_COUNT - 1].load(std::memory_order_relaxed);
    int32_t force_low  = CAL_FORCE_LEVELS[CAL_FORCE_LEVEL_COUNT - 2];
    int32_t force_high = CAL_FORCE_LEVELS[CAL_FORCE_LEVEL_COUNT - 1];
    
    if (spd_high == spd_low) return force_high; // Fallback
    
    int32_t force = force_low + ((force_high - force_low) * (abs_vel - spd_low)) / (spd_high - spd_low);
    if (force > 10000) force = 10000;
    return force;
}
