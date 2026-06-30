// =========================================================================
// Flash Storage — Persistent Calibration Data
// =========================================================================
// Saves the wheel center offset and pedal ADC ranges to the RP2040 flash.
// Uses the last sector of flash memory.
// flash_safe_execute is used to ensure Core 1 isn't accessing XIP memory
// while the flash is being erased/written.
// =========================================================================

#include "flash_storage.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include <cstring>

// Standard Pico flash starts at 0x10000000.
// Typical size is 2MB. We use the very last 4KB sector.
#define FLASH_TARGET_OFFSET (2048 * 1024 - FLASH_SECTOR_SIZE)

// Wrapper for flash erase/write to be run by flash_safe_execute
struct FlashCmdArgs
{
    uint32_t offset;
    const uint8_t *data;
    size_t length;
};

static void __not_in_flash_func(flash_write_wrapper)(void *param)
{
    auto *args = static_cast<FlashCmdArgs *>(param);
    flash_range_erase(args->offset, FLASH_SECTOR_SIZE);
    flash_range_program(args->offset, args->data, FLASH_PAGE_SIZE);
}

bool FlashStorage::load_internal(FlashCalibrationData &out_data)
{
    // Read directly from memory-mapped flash
    const FlashCalibrationData *flash_data_ptr = reinterpret_cast<const FlashCalibrationData *>(XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(&out_data, flash_data_ptr, sizeof(FlashCalibrationData));

    // Validate
    if (out_data.magic != FLASH_MAGIC_NUMBER)
    {
        return false;
    }
    if (out_data.version != FLASH_DATA_VERSION)
    {
        // Handle migration if needed in the future
        return false;
    }
    if (out_data.crc32 != calculate_crc(out_data))
    {
        return false; // Corrupted
    }

    return true;
}

bool FlashStorage::load(CalibrationState &out_state)
{
    FlashCalibrationData cal_data;
    bool valid = load_internal(cal_data);

    if (valid)
    {
        out_state.center_offset.store(cal_data.center_position);
        out_state.accel_min.store(cal_data.accel_min);
        out_state.accel_max.store(cal_data.accel_max);
        out_state.brake_min.store(cal_data.brake_min);
        out_state.brake_max.store(cal_data.brake_max);

        int32_t half_angle_deg = cal_data.wheel_angle_deg / 2;
        int32_t max_half_angle_counts = (half_angle_deg * WHEEL_COUNTS_PER_REV) / 360;
        out_state.max_half_angle_counts.store(max_half_angle_counts);
        out_state.wheel_angle_deg.store(cal_data.wheel_angle_deg);

        out_state.cw_zero_pwm.store(cal_data.cw_zero_pwm);
        out_state.ccw_zero_pwm.store(cal_data.ccw_zero_pwm);
        for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++)
        {
            out_state.cw_speed_lut[i].store(cal_data.cw_speed[i]);
            out_state.ccw_speed_lut[i].store(cal_data.ccw_speed[i]);
        }
        out_state.system_damper_strength.store(cal_data.system_damper_strength);
        out_state.force_scale_percent.store(cal_data.force_scale_percent);
        out_state.friction_fade_force.store(cal_data.friction_fade_force);
    }
    else
    {
        out_state.accel_min.store(100);
        out_state.accel_max.store(4000);
        out_state.brake_min.store(100);
        out_state.brake_max.store(4000);

        int32_t half_angle_deg = DEFAULT_MAX_WHEEL_ANGLE_DEG / 2;
        int32_t max_half_angle_counts = (half_angle_deg * WHEEL_COUNTS_PER_REV) / 360;
        out_state.max_half_angle_counts.store(max_half_angle_counts);
        out_state.wheel_angle_deg.store(DEFAULT_MAX_WHEEL_ANGLE_DEG);
        out_state.system_damper_strength.store(0);
        out_state.force_scale_percent.store(DEFAULT_FORCE_SCALE_PERCENT);
        out_state.friction_fade_force.store(DEFAULT_FRICTION_FADE_FORCE);
    }
    return valid;
}

bool FlashStorage::save_internal(const FlashCalibrationData &data, bool core1_running)
{
    // We need a page-aligned buffer to write to flash
    uint8_t page_buf[FLASH_PAGE_SIZE] = {0};

    // Copy data into buffer and update CRC
    FlashCalibrationData *p = reinterpret_cast<FlashCalibrationData *>(page_buf);
    memcpy(p, &data, sizeof(FlashCalibrationData));

    p->crc32 = calculate_crc(*p);

    FlashCmdArgs args;
    args.offset = FLASH_TARGET_OFFSET;
    args.data = page_buf;
    args.length = FLASH_PAGE_SIZE;

    if (core1_running)
    {
        // Use flash_safe_execute to pause Core 1 and safely write to flash
        int result = flash_safe_execute(flash_write_wrapper, &args, 50);
        return (result == PICO_OK);
    }
    else
    {
        // Core 1 is not running, safe to write directly with interrupts disabled
        uint32_t ints = save_and_disable_interrupts();
        flash_write_wrapper(&args);
        restore_interrupts(ints);
        return true;
    }
}

bool FlashStorage::save(const CalibrationState &state, bool core1_running)
{
    FlashCalibrationData data;
    data.center_position = state.center_offset.load();
    data.accel_min = state.accel_min.load();
    data.accel_max = state.accel_max.load();
    data.brake_min = state.brake_min.load();
    data.brake_max = state.brake_max.load();

    data.cw_zero_pwm = state.cw_zero_pwm.load();
    data.ccw_zero_pwm = state.ccw_zero_pwm.load();
    for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++)
    {
        data.cw_speed[i] = state.cw_speed_lut[i].load();
        data.ccw_speed[i] = state.ccw_speed_lut[i].load();
    }
    data.wheel_angle_deg = state.wheel_angle_deg.load();
    data.system_damper_strength = state.system_damper_strength.load();
    data.force_scale_percent = state.force_scale_percent.load();
    data.friction_fade_force = state.friction_fade_force.load();

    return save_internal(data, core1_running);
}

// Simple CRC32 implementation
uint32_t FlashStorage::calculate_crc(const FlashCalibrationData &data) const
{
    static_assert(offsetof(FlashCalibrationData, crc32) == sizeof(FlashCalibrationData) - sizeof(uint32_t),
                  "crc32 must be the last field in FlashCalibrationData");
    uint32_t crc = 0xFFFFFFFF;

    // Calculate CRC over everything EXCEPT the crc32 field itself
    size_t len = sizeof(FlashCalibrationData) - sizeof(uint32_t);
    const uint8_t *p = reinterpret_cast<const uint8_t *>(&data);

    for (size_t i = 0; i < len; i++)
    {
        crc ^= p[i];
        for (int j = 0; j < 8; j++)
        {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
        }
    }

    return ~crc;
}
