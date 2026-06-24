#pragma once
#include <cstdint>
#include "config.h"
#include "shared_state.h"

class PedalReader {
public:
    void init();
    void update();
    void read_raw_compensated(uint16_t &accel_comp, uint16_t &brake_comp);
    int16_t get_accel() const { return accel_filtered_; }
    int16_t get_brake() const { return brake_filtered_; }

    // Calibration: load min/max ADC values from CalibrationState
    void apply_calibration(const CalibrationState& state);

private:
    uint16_t trimmed_mean_filter(const uint16_t* history, uint8_t depth);
    int16_t scale_to_16bit(uint16_t raw, uint16_t cal_min, uint16_t cal_max);

    uint16_t accel_history_[ADC_FILTER_DEPTH] = {};
    uint16_t brake_history_[ADC_FILTER_DEPTH] = {};
    uint8_t  history_idx_ = 0;

    int16_t accel_filtered_ = -32767;
    int16_t brake_filtered_ = -32767;

    // Calibration ranges (ADC values)
    uint16_t accel_min_ = 0;
    uint16_t accel_max_ = 4095;
    uint16_t brake_min_ = 0;
    uint16_t brake_max_ = 4095;
};
