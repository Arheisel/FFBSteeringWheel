// =========================================================================
// USB HID — SET_REPORT Callback & Input Report Sending
// =========================================================================
// Parses incoming FFB output/feature reports by casting USB payloads
// directly into the packed structs from usb_ffb_descriptors.h.
// Routes parsed effects into the shared EffectState for Core 1.
// =========================================================================

#include <cstring>
#include "usb_hid.h"
#include "tusb.h"
#include "hid_device.h"
#include "pico/time.h"
#include "config.h"
#include "usb_ffb_descriptors.h"
#include "shared_state.h"

static void send_pid_status(uint8_t effect_idx, bool playing);

// Global pointer to shared state (set during init)
static SharedState *g_state = nullptr;

void usb_hid_init(SharedState &state)
{
    g_state = &state;
    g_state->ffb.init();
}

// =========================================================================
// Joystick Input Report Struct (matches descriptor in usb_descriptors.cpp)
// =========================================================================

struct __attribute__((packed)) JoystickInputReport
{
    uint8_t reportId; // = 1
    uint16_t buttons; // 16 buttons, bit-packed
    int16_t x;        // Steering axis
    int16_t accel;    // Accelerator pedal
    int16_t brake;    // Brake pedal
};

void usb_hid_send_input_report(SharedState &state)
{
    if (!tud_hid_ready())
        return;

    static uint64_t last_playing_mask = 0;
    static bool last_actuators_enabled = true;

    // 1. Read current playing state and actuators state under lock
    uint64_t current_playing_mask = 0;
    bool current_actuators_enabled = false;

    uint32_t irq = spin_lock_blocking(state.ffb.lock);
    for (uint8_t i = 0; i < MAX_EFFECTS; i++)
    {
        if (state.ffb.effects[i].state == EffectSlot::STATE_PLAYING)
        {
            current_playing_mask |= (1ULL << i);
        }
    }
    current_actuators_enabled = state.ffb.actuators_enabled;
    spin_unlock(state.ffb.lock, irq);

    // 2. Prioritize sending pending PID status reports over the joystick report
    uint64_t diff = current_playing_mask ^ last_playing_mask;
    if (diff != 0)
    {
        // Find the first changed bit
        for (uint8_t i = 0; i < MAX_EFFECTS; i++)
        {
            if (diff & (1ULL << i))
            {
                bool playing = (current_playing_mask & (1ULL << i)) != 0;
                send_pid_status(i + 1, playing);
                // Update our last playing mask for only this bit, so we send the rest in future ticks
                if (playing)
                {
                    last_playing_mask |= (1ULL << i);
                }
                else
                {
                    last_playing_mask &= ~(1ULL << i);
                }
                return; // Sent one report, wait for next tick
            }
        }
    }

    if (current_actuators_enabled != last_actuators_enabled)
    {
        // Find the first playing effect or use 0 for general status
        uint8_t report_idx = 0;
        bool playing = false;
        for (uint8_t i = 0; i < MAX_EFFECTS; i++)
        {
            if (current_playing_mask & (1ULL << i))
            {
                report_idx = i + 1;
                playing = true;
                break;
            }
        }
        send_pid_status(report_idx, playing);
        last_actuators_enabled = current_actuators_enabled;
        return;
    }

    // 3. Send normal joystick report
    JoystickInputReport report;
    report.reportId = 0x01;
    report.buttons = state.buttons.load();
    int32_t pos = state.sensor.wheel_position.load();
    int32_t max_half_angle_counts = state.cal_state.max_half_angle_counts.load();
    if (pos > max_half_angle_counts)
        pos = max_half_angle_counts;
    else if (pos < -max_half_angle_counts)
        pos = -max_half_angle_counts;

    // Scale to the HID descriptor's expected logical min/max (-32767 to 32767)
    pos = (pos * 32767) / max_half_angle_counts;

    report.x = static_cast<int16_t>(pos);
    report.accel = static_cast<int16_t>(state.pedal_accel.load());
    report.brake = static_cast<int16_t>(state.pedal_brake.load());

    // Send via Report ID 1 — TinyUSB strips the reportId from the struct
    // when using tud_hid_report(), so we pass the body after reportId.
    tud_hid_report(0x01, &report.buttons, sizeof(report) - 1);
}

// =========================================================================
// PID Status Input Report (Report ID 2)
// =========================================================================

static void send_pid_status(uint8_t effect_idx, bool playing)
{
    if (!g_state)
        return;

    USB_FFBReport_PIDStatus_Input_Data_t status_report;
    status_report.reportId = 0x02;

    // Build status byte
    status_report.status = 0;
    if (g_state->ffb.actuators_enabled)
    {
        status_report.status |= 0x02; // Bit 1: Actuators Enabled
    }
    status_report.status |= 0x04; // Bit 2: Safety Switch (always OK)
    status_report.status |= 0x10; // Bit 4: Actuator Power (always on)

    // Effect Block Index with playing bit
    status_report.effectBlockIndex = effect_idx;
    if (playing)
    {
        status_report.effectBlockIndex |= 0x80; // Bit 7: Effect Playing
    }

    tud_hid_report(0x02, &status_report.status, sizeof(status_report) - 1);
}

// =========================================================================
// Helper: Find or allocate an effect slot
// =========================================================================

static int8_t allocate_effect_slot()
{
    if (!g_state)
        return -1;

    uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
    for (uint8_t i = 0; i < MAX_EFFECTS; i++)
    {
        if (g_state->ffb.effects[i].state == EffectSlot::STATE_FREE)
        {
            g_state->ffb.effects[i].state = EffectSlot::STATE_ALLOCATED;
            spin_unlock(g_state->ffb.lock, irq);
            return static_cast<int8_t>(i);
        }
    }
    spin_unlock(g_state->ffb.lock, irq);
    return -1; // Pool full
}

// =========================================================================
// SET_REPORT Callback — Output & Feature reports from host
// =========================================================================

// Store the last allocated slot persistently across calls between SET and GET
static int8_t last_allocated_slot = -1;

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    if (!g_state)
        return;

    // =====================================================================
    // OUTPUT REPORTS (host → device)
    // =====================================================================
    if (report_type == HID_REPORT_TYPE_OUTPUT)
    {
        switch (report_id)
        {

        // -----------------------------------------------------------------
        // Set Effect Report (Output Report ID 1)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_SET_EFFECT:
        {
            if (bufsize < sizeof(USB_FFBReport_SetEffect_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_SetEffect_Output_Data_t *>(buffer - 1);
            uint8_t idx = data->effectBlockIndex;
            if (idx < 1 || idx > MAX_EFFECTS)
                break;
            idx--; // Convert 1-based to 0-based

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            memcpy(&g_state->ffb.effects[idx].params, data, sizeof(*data));
            if (g_state->ffb.effects[idx].state == EffectSlot::STATE_FREE)
            {
                g_state->ffb.effects[idx].state = EffectSlot::STATE_ALLOCATED;
            }
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        // -----------------------------------------------------------------
        // Set Envelope Report (Output Report ID 2)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_SET_ENVELOPE:
        {
            if (bufsize < sizeof(USB_FFBReport_SetEnvelope_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_SetEnvelope_Output_Data_t *>(buffer - 1);
            uint8_t idx = data->effectBlockIndex;
            if (idx < 1 || idx > MAX_EFFECTS)
                break;
            idx--;

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            memcpy(&g_state->ffb.effects[idx].envelope, data, sizeof(*data));
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        // -----------------------------------------------------------------
        // Set Condition Report (Output Report ID 3)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_SET_CONDITION:
        {
            if (bufsize < sizeof(USB_FFBReport_SetCondition_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_SetCondition_Output_Data_t *>(buffer - 1);
            uint8_t idx = data->effectBlockIndex;
            if (idx < 1 || idx > MAX_EFFECTS)
                break;
            idx--;

            // parameterBlockOffset lower 4 bits = axis index (0 or 1)
            uint8_t axis = data->parameterBlockOffset & 0x0F;
            if (axis > 1)
                axis = 0;

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            memcpy(&g_state->ffb.effects[idx].condition[axis], data, sizeof(*data));
            if (axis + 1 > g_state->ffb.effects[idx].conditionBlockCount)
            {
                g_state->ffb.effects[idx].conditionBlockCount = axis + 1;
            }
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        // -----------------------------------------------------------------
        // Set Periodic Report (Output Report ID 4)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_SET_PERIODIC:
        {
            if (bufsize < sizeof(USB_FFBReport_SetPeriodic_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_SetPeriodic_Output_Data_t *>(buffer - 1);
            uint8_t idx = data->effectBlockIndex;
            if (idx < 1 || idx > MAX_EFFECTS)
                break;
            idx--;

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            memcpy(&g_state->ffb.effects[idx].periodic, data, sizeof(*data));
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        // -----------------------------------------------------------------
        // Set Constant Force Report (Output Report ID 5)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_SET_CONSTANT_FORCE:
        {
            if (bufsize < sizeof(USB_FFBReport_SetConstantForce_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_SetConstantForce_Output_Data_t *>(buffer - 1);
            uint8_t idx = data->effectBlockIndex;
            if (idx < 1 || idx > MAX_EFFECTS)
                break;
            idx--;

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            memcpy(&g_state->ffb.effects[idx].constant, data, sizeof(*data));
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        // -----------------------------------------------------------------
        // Set Ramp Force Report (Output Report ID 6)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_SET_RAMP_FORCE:
        {
            if (bufsize < sizeof(USB_FFBReport_SetRampForce_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_SetRampForce_Output_Data_t *>(buffer - 1);
            uint8_t idx = data->effectBlockIndex;
            if (idx < 1 || idx > MAX_EFFECTS)
                break;
            idx--;

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            memcpy(&g_state->ffb.effects[idx].ramp, data, sizeof(*data));
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        // -----------------------------------------------------------------
        // Effect Operation Report (Output Report ID 10)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_EFFECT_OPERATION:
        {
            if (bufsize < sizeof(USB_FFBReport_EffectOperation_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_EffectOperation_Output_Data_t *>(buffer - 1);
            uint8_t idx = data->effectBlockIndex;
            if (idx < 1 || idx > MAX_EFFECTS)
                break;
            idx--;

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            switch (data->operation)
            {
            case 1: // Start
                g_state->ffb.effects[idx].state = EffectSlot::STATE_PLAYING;
                g_state->ffb.effects[idx].start_time_us = time_us_32();
                g_state->ffb.effects[idx].loop_count = data->loopCount;
                break;
            case 2: // Start Solo — stop all others first
                for (uint8_t i = 0; i < MAX_EFFECTS; i++)
                {
                    if (i != idx && g_state->ffb.effects[i].state == EffectSlot::STATE_PLAYING)
                    {
                        g_state->ffb.effects[i].state = EffectSlot::STATE_ALLOCATED;
                    }
                }
                g_state->ffb.effects[idx].state = EffectSlot::STATE_PLAYING;
                g_state->ffb.effects[idx].start_time_us = time_us_32();
                g_state->ffb.effects[idx].loop_count = data->loopCount;
                break;
            case 3: // Stop
                g_state->ffb.effects[idx].state = EffectSlot::STATE_ALLOCATED;
                break;
            }
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        // -----------------------------------------------------------------
        // PID Block Free Report (Output Report ID 11)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_PID_BLOCK_FREE:
        {
            if (bufsize < sizeof(USB_FFBReport_BlockFree_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_BlockFree_Output_Data_t *>(buffer - 1);
            uint8_t idx = data->effectBlockIndex;
            if (idx < 1 || idx > MAX_EFFECTS)
                break;
            idx--;

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            g_state->ffb.effects[idx].state = EffectSlot::STATE_FREE;
            g_state->ffb.effects[idx].conditionBlockCount = 0;
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        // -----------------------------------------------------------------
        // PID Device Control Report (Output Report ID 12)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_PID_DEVICE_CONTROL:
        {
            if (bufsize < sizeof(USB_FFBReport_DeviceControl_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_DeviceControl_Output_Data_t *>(buffer - 1);

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            switch (data->control)
            {
            case 1: // Enable Actuators
                g_state->ffb.actuators_enabled = true;
                break;
            case 2: // Disable Actuators
                g_state->ffb.actuators_enabled = false;
                break;
            case 4: // Stop All Effects
                for (auto &e : g_state->ffb.effects)
                {
                    if (e.state == EffectSlot::STATE_PLAYING)
                    {
                        e.state = EffectSlot::STATE_ALLOCATED;
                    }
                }
                break;
            case 8: // Device Reset
                for (auto &e : g_state->ffb.effects)
                {
                    e.state = EffectSlot::STATE_FREE;
                    e.conditionBlockCount = 0;
                }
                g_state->ffb.actuators_enabled = true;
                g_state->ffb.device_paused = false;
                break;
            case 16: // Pause
                g_state->ffb.device_paused = true;
                break;
            case 32: // Continue
                g_state->ffb.device_paused = false;
                break;
            }
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        // -----------------------------------------------------------------
        // Device Gain Report (Output Report ID 13)
        // -----------------------------------------------------------------
        case REPORT_ID_OUTPUT_DEVICE_GAIN:
        {
            if (bufsize < sizeof(USB_FFBReport_DeviceGain_Output_Data_t) - 1)
                break;
            auto *data = reinterpret_cast<const USB_FFBReport_DeviceGain_Output_Data_t *>(buffer - 1);

            uint32_t irq = spin_lock_blocking(g_state->ffb.lock);
            g_state->ffb.device_gain = data->gain;
            spin_unlock(g_state->ffb.lock, irq);
            break;
        }

        default:
            break;
        }
    }

    // =====================================================================
    // FEATURE REPORTS (host → device)
    // =====================================================================
    else if (report_type == HID_REPORT_TYPE_FEATURE)
    {
        switch (report_id)
        {

        // -----------------------------------------------------------------
        // Create New Effect Feature Report (Report ID 5)
        // -----------------------------------------------------------------
        case REPORT_ID_FEATURE_CREATE_NEW_EFFECT:
        {
            // Host wants to create a new effect. Allocate a slot.
            // DirectInput/Linux will query this slot later via GET_REPORT.
            last_allocated_slot = allocate_effect_slot();
            break;
        }

        default:
            break;
        }
    }
}

// =========================================================================
// GET_REPORT Callback — Feature reports device → host
// =========================================================================

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    if (!g_state)
        return 0;

    if (report_type == HID_REPORT_TYPE_FEATURE)
    {
        switch (report_id)
        {

        // -----------------------------------------------------------------
        // Create New Effect Feature Report (Report ID 5)
        // Host is creating a new effect — we allocate a slot.
        // -----------------------------------------------------------------
        case REPORT_ID_FEATURE_CREATE_NEW_EFFECT:
        {
            USB_FFBReport_CreateNewEffect_Feature_Data_t response;
            response.reportId = report_id;
            response.effectType = 0; // Echo back

            // byteCount = size of the parameter block. We report device-managed pool
            // with unlimited RAM, so this is always 0.
            response.byteCount = 0;

            uint16_t len = sizeof(response) - 1; // Exclude reportId
            if (len > reqlen)
                len = reqlen;
            memcpy(buffer, &response.effectType, len);
            return len;
        }

        // -----------------------------------------------------------------
        // PID Block Load Feature Report (Report ID 6)
        // -----------------------------------------------------------------
        case REPORT_ID_FEATURE_PID_BLOCK_LOAD:
        {
            // Return the most recently allocated slot
            int8_t slot = last_allocated_slot;

            USB_FFBReport_PIDBlockLoad_Feature_Data_t response;
            response.reportId = report_id;

            if (slot >= 0)
            {
                response.effectBlockIndex = slot + 1; // 1-based
                response.loadStatus = 1;              // Success
                response.ramPoolAvailable = 0xFFFF;
            }
            else
            {
                response.effectBlockIndex = 0;
                response.loadStatus = 2; // Full
                response.ramPoolAvailable = 0;
            }

            uint16_t len = sizeof(response) - 1;
            if (len > reqlen)
                len = reqlen;
            memcpy(buffer, &response.effectBlockIndex, len);
            return len;
        }

        // -----------------------------------------------------------------
        // PID Pool Feature Report (Report ID 7)
        // -----------------------------------------------------------------
        case REPORT_ID_FEATURE_PID_POOL:
        {
            USB_FFBReport_PIDPool_Feature_Data_t response;
            response.reportId = report_id;
            response.ramPoolSize = 0xFFFF;
            response.maxSimultaneousEffects = MAX_EFFECTS;
            // Bit 0: Device Managed Pool = 1, Bit 1: Shared Parameter Blocks = 1
            response.memoryManagement = 0x03;

            uint16_t len = sizeof(response) - 1;
            if (len > reqlen)
                len = reqlen;
            memcpy(buffer, &response.ramPoolSize, len);
            return len;
        }

        default:
            break;
        }
    }

    return 0;
}
