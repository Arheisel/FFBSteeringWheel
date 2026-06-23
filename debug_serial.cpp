// =========================================================================
// Debug Serial Console — CDC ACM Line-Buffered CLI
// =========================================================================
// Provides a simple debug console over USB CDC for diagnostics.
// =========================================================================

#include "debug_serial.h"
#include "tusb.h"
#include "config.h"
#include "shared_state.h"
#include "flash_storage.h"
#include "pedal_reader.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace {

    // =========================================================================
    // Error Log — Lock-free ring buffer (single writer from ISR, single reader)
    // =========================================================================

    struct ErrorLogEntry {
        uint64_t timestamp_us;
        SystemStatus error_code;
    };

    ErrorLogEntry g_error_log[ERROR_LOG_SIZE];
    volatile uint8_t g_error_log_write_idx = 0;
    uint8_t g_error_log_read_idx = 0;

    SharedState* g_state = nullptr;
    PedalReader* g_pedals = nullptr;
    FlashStorage* g_flash = nullptr;

    char g_line_buf[64];
    uint8_t g_line_len = 0;
    bool g_prompt_printed = false;
    char g_last_cmd_buf[64] = {0};
    uint8_t g_last_cmd_len = 0;
    uint8_t g_escape_state = 0;

    uint64_t g_connected_timestamp = 0;
    bool g_was_connected = false;

    // =========================================================================
    // Helper: Write a string to CDC
    // =========================================================================

    void cdc_print(const char* str) {
        if (!tud_cdc_connected()) return;
        uint32_t len = strlen(str);
        uint32_t sent = 0;
        while (sent < len) {
            if (!tud_cdc_connected()) return;  // Guard against mid-write disconnect
            uint32_t avail = tud_cdc_write_available();
            if (avail == 0) {
                tud_cdc_write_flush();
                // Brief yield to let USB send
                tud_task();
                continue;
            }
            uint32_t chunk = len - sent;
            if (chunk > avail) chunk = avail;
            tud_cdc_write(str + sent, chunk);
            sent += chunk;
        }
        tud_cdc_write_flush();
    }

    // =========================================================================
    // Helper: Integer to string (avoids printf/snprintf overhead)
    // =========================================================================

    char* int_to_str(int32_t val, char* buf) {
        if (val < 0) {
            *buf++ = '-';
            // Handle INT32_MIN edge case
            if (val == -2147483647 - 1) {
                strcpy(buf, "2147483648");
                return buf + 10;
            }
            val = -val;
        }
        // Write digits in reverse
        char tmp[11];
        int i = 0;
        if (val == 0) {
            tmp[i++] = '0';
        } else {
            while (val > 0) {
                tmp[i++] = '0' + (val % 10);
                val /= 10;
            }
        }
        // Reverse into buf
        for (int j = i - 1; j >= 0; j--) {
            *buf++ = tmp[j];
        }
        *buf = '\0';
        return buf;
    }

    char* uint_to_str(uint32_t val, char* buf) {
        char tmp[11];
        int i = 0;
        if (val == 0) {
            tmp[i++] = '0';
        } else {
            while (val > 0) {
                tmp[i++] = '0' + (val % 10);
                val /= 10;
            }
        }
        for (int j = i - 1; j >= 0; j--) {
            *buf++ = tmp[j];
        }
        *buf = '\0';
        return buf;
    }

    // =========================================================================
    // Command: Print live calibration data
    // =========================================================================

    void cmd_calibration() {
        if (!g_state || !g_pedals) {
            cdc_print("ERR: No state\r\n");
            return;
        }

        char buf[64];
        char* p;

        cdc_print("=== CALIBRATION DATA ===\r\n");

        p = buf; strcpy(p, "Center: "); p += 8;
        p = int_to_str(g_state->cal_state.center_offset.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);
        
        p = buf; strcpy(p, "Wheel Angle (deg): "); p += 19;
        p = uint_to_str(g_state->cal_state.wheel_angle_deg.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "System Damper: "); p += 15;
        p = uint_to_str(g_state->cal_state.system_damper_strength.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "Forward Max PWM: "); p += 17;
        p = uint_to_str(g_state->cal_state.forward_max_pwm.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "Force Scale %: "); p += 15;
        p = uint_to_str(g_state->cal_state.force_scale_percent.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "Friction Fade Force: "); p += 21;
        p = uint_to_str(g_state->cal_state.friction_fade_force.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        uint16_t amin, amax, bmin, bmax;
        g_pedals->get_calibration(amin, amax, bmin, bmax);

        p = buf; strcpy(p, "Accel: "); p += 7;
        p = uint_to_str(amin, p); *p++ = '-';
        p = uint_to_str(amax, p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "Brake: "); p += 7;
        p = uint_to_str(bmin, p); *p++ = '-';
        p = uint_to_str(bmax, p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "CW Zero PWM: "); p += 13;
        p = uint_to_str(g_state->cal_state.cw_zero_pwm.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "CCW Zero PWM: "); p += 14;
        p = uint_to_str(g_state->cal_state.ccw_zero_pwm.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        cdc_print("CW Speed LUT:");
        for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
            p = buf; *p++ = ' ';
            p = int_to_str(g_state->cal_state.cw_speed[i].load(), p);
            *p = '\0';
            cdc_print(buf);
        }
        cdc_print("\r\n");

        cdc_print("CCW Speed LUT:");
        for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
            p = buf; *p++ = ' ';
            p = int_to_str(g_state->cal_state.ccw_speed[i].load(), p);
            *p = '\0';
            cdc_print(buf);
        }
        cdc_print("\r\n");
    }

    // =========================================================================
    // Command: Save calibration data
    // =========================================================================

    void cmd_save_calibration() {
        if (!g_state || !g_pedals || !g_flash) return;

        cdc_print("Saving to flash...\r\n");
        FlashCalibrationData data;
        data.center_position = g_state->cal_state.center_offset.load();
        data.wheel_angle_deg = g_state->cal_state.wheel_angle_deg.load();
        data.system_damper_strength = g_state->cal_state.system_damper_strength.load();
        data.forward_max_pwm = g_state->cal_state.forward_max_pwm.load();
        data.force_scale_percent = g_state->cal_state.force_scale_percent.load();
        data.friction_fade_force = g_state->cal_state.friction_fade_force.load();
        g_pedals->get_calibration(data.accel_min, data.accel_max, data.brake_min, data.brake_max);
        data.cw_zero_pwm = g_state->cal_state.cw_zero_pwm.load();
        data.ccw_zero_pwm = g_state->cal_state.ccw_zero_pwm.load();
        for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
            data.cw_speed[i] = g_state->cal_state.cw_speed[i].load();
            data.ccw_speed[i] = g_state->cal_state.ccw_speed[i].load();
        }

        // Core 1 is spinning, but we still pass core1_running=true so flash_safe_execute 
        // puts Core 1 into a lockout state cleanly before pausing Core 0 interrupts.
        bool ok = g_flash->save(data, true);

        cdc_print(ok ? "Save OK\r\n" : "Save FAILED\r\n");
    }

    // =========================================================================
    // Command: Print live status
    // =========================================================================

    void print_error_flags(uint8_t flags) {
        char buf[128];
        char* p = buf;
        strcpy(p, "Err flags: "); p += 11;
        p = uint_to_str(flags, p);
        
        if (flags == 0) {
            strcpy(p, " (None)\r\n");
        } else {
            strcpy(p, " ("); p += 2;
            bool first = true;
            if (flags & SensorState::ERR_MAGNET_HIGH) { strcpy(p, "MagnetHigh"); p += 10; first = false; }
            if (flags & SensorState::ERR_MAGNET_LOW) { if (!first) { *p++ = ','; *p++ = ' '; } strcpy(p, "MagnetLow"); p += 9; first = false; }
            if (flags & SensorState::ERR_MAGNET_MISSING) { if (!first) { *p++ = ','; *p++ = ' '; } strcpy(p, "MagnetMissing"); p += 13; first = false; }
            if (flags & SensorState::ERR_I2C_WATCHDOG) { if (!first) { *p++ = ','; *p++ = ' '; } strcpy(p, "I2CWatchdog"); p += 11; first = false; }
            if (flags & SensorState::ERR_DESYNC) { if (!first) { *p++ = ','; *p++ = ' '; } strcpy(p, "Desync"); p += 6; first = false; }
            if (flags & SensorState::ERR_RECOVERY_DESYNC) { if (!first) { *p++ = ','; *p++ = ' '; } strcpy(p, "RecDesync"); p += 9; first = false; }
            strcpy(p, ")\r\n");
        }
        cdc_print(buf);
    }

    void cmd_status() {
        if (!g_state) {
            cdc_print("ERR: No state\r\n");
            return;
        }

        char buf[128];
        char* p;

        cdc_print("=== LIVE STATUS ===\r\n");

        p = buf; strcpy(p, "Position: "); p += 10;
        p = int_to_str(g_state->sensor.wheel_position.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        int32_t vel_cps = g_state->sensor.wheel_velocity.load();
        p = buf; strcpy(p, "Velocity (cps): "); p += 16;
        p = int_to_str(vel_cps, p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "Buttons: 0x"); p += 11;
        uint16_t btns = g_state->buttons.load();
        const char hex[] = "0123456789ABCDEF";
        *p++ = hex[(btns >> 12) & 0xF];
        *p++ = hex[(btns >> 8) & 0xF];
        *p++ = hex[(btns >> 4) & 0xF];
        *p++ = hex[btns & 0xF];
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "Accel: "); p += 7;
        p = int_to_str(g_state->pedal_accel.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "Brake: "); p += 7;
        p = int_to_str(g_state->pedal_brake.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        uint8_t flags = g_state->sensor.error_flags.load();
        print_error_flags(flags);

        p = buf; strcpy(p, "LED status: "); p += 12;
        p = uint_to_str(static_cast<uint8_t>(g_state->led_status.get()), p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        uint8_t err_count = (g_error_log_write_idx - g_error_log_read_idx + ERROR_LOG_SIZE) % ERROR_LOG_SIZE;
        p = buf; strcpy(p, "Logged errors: "); p += 15;
        p = uint_to_str(err_count, p);
        strcpy(p, "\r\n");
        cdc_print(buf);

        p = buf; strcpy(p, "Loop time (EMA): "); p += 17;
        p = uint_to_str(g_state->loop_time_avg_us.load(), p);
        strcpy(p, " us\r\n");
        cdc_print(buf);

        // Request AGC register read from Core 1
        g_state->request_agc_read.store(true);

        // Poll for completion (timeout after 10ms)
        uint64_t start = time_us_64();
        while (g_state->request_agc_read.load()) {
            if (time_us_64() - start > 10000) {
                cdc_print("AGC: timeout\r\n");
                g_state->request_agc_read.store(false);
                return;
            }
            tud_task(); // Keep USB alive while waiting
        }

        p = buf; strcpy(p, "AGC: "); p += 5;
        p = uint_to_str(g_state->sensor.agc_value.load(), p);
        strcpy(p, "\r\n");
        cdc_print(buf);
    }

    // =========================================================================
    // Command: Print and clear error log
    // =========================================================================

    const char* error_name(SystemStatus code) {
        switch (code) {
            case SystemStatus::FlashCalMissing: return "FlashCalMissing";
            case SystemStatus::MagnetHigh:      return "MagnetHigh";
            case SystemStatus::MagnetLow:       return "MagnetLow";
            case SystemStatus::MagnetMissing:   return "MagnetMissing";
            case SystemStatus::I2CWatchdogFired:return "I2CWatchdog";
            case SystemStatus::EncoderDesync:   return "Desync";
            case SystemStatus::DesyncAfterRecovery: return "RecoveryDesync";
            case SystemStatus::FlashWriteFailed:return "FlashWriteFailed";
            default: return "Unknown";
        }
    }

    void cmd_errors() {
        uint8_t write_idx = g_error_log_write_idx;
        uint8_t read_idx = g_error_log_read_idx;

        if (read_idx == write_idx) {
            cdc_print("=== ERROR LOG: Empty ===\r\n");
            return;
        }

        cdc_print("=== ERROR LOG ===\r\n");

        char buf[64];
        char* p;
        int count = 0;

        while (read_idx != write_idx && count < ERROR_LOG_SIZE) {
            ErrorLogEntry& e = g_error_log[read_idx];
            p = buf;
            *p++ = '[';
            p = uint_to_str(static_cast<uint32_t>(e.timestamp_us / 1000), p);  // uint to avoid sign wrap after 24d
            strcpy(p, "ms | Code "); p += 10;
            p = uint_to_str(static_cast<uint8_t>(e.error_code), p);
            strcpy(p, " ("); p += 2;
            strcpy(p, error_name(e.error_code)); p += strlen(error_name(e.error_code));
            strcpy(p, ")\r\n"); p += 3;
            cdc_print(buf);

            read_idx = (read_idx + 1) % ERROR_LOG_SIZE;
            count++;
        }

        // Clear the log
        g_error_log_read_idx = write_idx;

        p = buf; strcpy(p, "Total: "); p += 7;
        p = int_to_str(count, p);
        strcpy(p, " entries (cleared)\r\n");
        cdc_print(buf);
    }

    // =========================================================================
    // Command Parser
    // =========================================================================

    void print_help() {
        cdc_print("Commands:\r\n");
        cdc_print("  s   - Print live status\r\n");
        cdc_print("  c   - Print live calibration data\r\n");
        cdc_print("  e   - Print error log\r\n");
        cdc_print("  cw  - Write live calibration data to flash\r\n");
        cdc_print("  cs  - Configuration Settings:\r\n");
        cdc_print("        cs <lut> <idx> <val> - Set LUT value (lut: cwl, ccl; idx: 0-4)\r\n");
        cdc_print("        cs amin/amax <val>   - Set accelerator min/max\r\n");
        cdc_print("        cs bmin/bmax <val>   - Set brake min/max\r\n");
        cdc_print("        cs cwz/ccz <val>     - Set CW/CCW zero PWM\r\n");
        cdc_print("        cs center <val>      - Set wheel center offset\r\n");
        cdc_print("        cs angle <val>       - Set max wheel angle (>=180)\r\n");
        cdc_print("        cs damper <val>      - Set system damper (0-10000)\r\n");
        cdc_print("        cs maxpwm <val>      - Set forward max PWM (0-6249)\r\n");
        cdc_print("        cs scale <val>       - Set force scale %\r\n");
        cdc_print("        cs friction <val>    - Set friction fade force (0-10000)\r\n");
        cdc_print("  help - Print this help\r\n");
    }

    void cmd_cs(int argc, char** argv) {
        if (argc < 3) {
            cdc_print("ERR: Usage: cs <var> <val>\r\n");
            print_help();
            return;
        }
        const char* var = argv[1];
        if (strcmp(var, "cwl") == 0 || strcmp(var, "ccl") == 0) {
            if (argc == 4) {
                int idx = atoi(argv[2]);
                int32_t val = atoi(argv[3]);
                if (idx >= 0 && idx < CAL_FORCE_LEVEL_COUNT) {
                    if (strcmp(var, "cwl") == 0) g_state->cal_state.cw_speed[idx].store(val);
                    else g_state->cal_state.ccw_speed[idx].store(val);
                    cdc_print("LUT updated.\r\n");
                    // Signal Core 1 to re-apply the new calibration values
                    g_state->calibration_reload.store(true);
                } else {
                    cdc_print("ERR: Invalid index\r\n");
                }
            } else {
                cdc_print("ERR: Usage: cs <lut> <idx> <val>\r\n");
                print_help();
            }
        } else {
            int32_t val = atoi(argv[2]);
            if (strcmp(var, "cwz") == 0) g_state->cal_state.cw_zero_pwm.store(static_cast<uint16_t>(val));
            else if (strcmp(var, "ccz") == 0) g_state->cal_state.ccw_zero_pwm.store(static_cast<uint16_t>(val));
            else if (strcmp(var, "center") == 0) {
                g_state->cal_state.center_offset.store(val);
            } else if (strcmp(var, "angle") == 0) {
                if (val < 180) {
                    cdc_print("ERR: angle must be >= 180 degrees\r\n");
                    return;
                }
                g_state->cal_state.wheel_angle_deg.store(val);
                int32_t half_deg = val / 2;
                int32_t max_half_angle_counts = (half_deg * WHEEL_COUNTS_PER_REV) / 360;
                g_state->cal_state.max_half_angle_counts.store(max_half_angle_counts);
            } else if (strcmp(var, "damper") == 0) {
                if (val < 0 || val > 10000) {
                    cdc_print("ERR: damper must be 0 to 10000\r\n");
                    return;
                }
                g_state->cal_state.system_damper_strength.store(val);
            } else if (strcmp(var, "maxpwm") == 0) {
                if (val < 0 || val > PWM_WRAP) {
                    cdc_print("ERR: maxpwm out of range\r\n");
                    return;
                }
                g_state->cal_state.forward_max_pwm.store(val);
            } else if (strcmp(var, "scale") == 0) {
                if (val < 0) {
                    cdc_print("ERR: scale must be >= 0\r\n");
                    return;
                }
                g_state->cal_state.force_scale_percent.store(val);
            } else if (strcmp(var, "friction") == 0) {
                if (val < 0 || val > 10000) {
                    cdc_print("ERR: friction must be 0 to 10000\r\n");
                    return;
                }
                g_state->cal_state.friction_fade_force.store(val);
            } else if (strcmp(var, "amin") == 0 || strcmp(var, "amax") == 0 || 
                    strcmp(var, "bmin") == 0 || strcmp(var, "bmax") == 0) {
                uint16_t amin, amax, bmin, bmax;
                g_pedals->get_calibration(amin, amax, bmin, bmax);
                if (strcmp(var, "amin") == 0) amin = val;
                else if (strcmp(var, "amax") == 0) amax = val;
                else if (strcmp(var, "bmin") == 0) bmin = val;
                else if (strcmp(var, "bmax") == 0) bmax = val;
                g_pedals->set_calibration(amin, amax, bmin, bmax);
            } else {
                cdc_print("ERR: Unknown variable\r\n");
                print_help();
                return;
            }
            cdc_print("Updated.\r\n");
            // Signal Core 1 to re-apply the new calibration values
            g_state->calibration_reload.store(true);
        }
    }

    void process_command(char* cmd) {
        char* argv[5];
        int argc = 0;
        char* p = cmd;
        while (*p) {
            while (*p == ' ') p++;
            if (!*p) break;
            argv[argc++] = p;
            if (argc >= 5) break;
            while (*p && *p != ' ') p++;
            if (*p) {
                *p = '\0';
                p++;
            }
        }

        if (argc == 0) return;

        if (strcmp(argv[0], "s") == 0) {
            cmd_status();
        } else if (strcmp(argv[0], "c") == 0) {
            cmd_calibration();
        } else if (strcmp(argv[0], "e") == 0) {
            cmd_errors();
        } else if (strcmp(argv[0], "help") == 0) {
            print_help();
        } else if (strcmp(argv[0], "cw") == 0) {
            cmd_save_calibration();
        } else if (strcmp(argv[0], "cs") == 0) {
            cmd_cs(argc, argv);
        } else {
            cdc_print("ERR: Unknown command.\r\n");
            print_help();
        }
    }

}

// =========================================================================
// Public API
// =========================================================================

void debug_serial_init(SharedState& state, PedalReader& pedals, FlashStorage& flash) {
    g_state = &state;
    g_pedals = &pedals;
    g_flash = &flash;
    memset(g_error_log, 0, sizeof(g_error_log));
    g_error_log_write_idx = 0;
    g_error_log_read_idx = 0;
}

void debug_log_error(SystemStatus error_code) {
    uint8_t idx = g_error_log_write_idx;
    uint8_t next_idx = (idx + 1) % ERROR_LOG_SIZE;

    // When full, silently overwrite the current slot and do NOT advance read_idx.
    // read_idx is only ever written by Core 0 (cmd_errors), eliminating the cross-core race.
    g_error_log[idx].timestamp_us = time_us_64();
    g_error_log[idx].error_code = error_code;
    g_error_log_write_idx = next_idx;
}

// =========================================================================
// Main update — called from Core 0 main loop
// =========================================================================

void debug_serial_update() {
    bool connected = tud_cdc_connected();
    if (!connected) {
        g_prompt_printed = false;
        g_line_len = 0;
        g_escape_state = 0;
        g_was_connected = false;
        return;
    }

    if (!g_was_connected) {
        g_was_connected = true;
        g_connected_timestamp = time_us_64();
    }

    if (!g_prompt_printed) {
        // Wait 500ms after connection before sending the first prompt
        // to give the host terminal software time to initialize.
        if (time_us_64() - g_connected_timestamp > 500000) {
            cdc_print("\r\nffbserial: ");
            g_prompt_printed = true;
        }
    }

    if (!tud_cdc_available()) return;

    char c = (char)tud_cdc_read_char();

    if (g_escape_state == 1) {
        if (c == '[') g_escape_state = 2;
        else g_escape_state = 0;
        return;
    } else if (g_escape_state == 2) {
        if (c == 'A') { // Arrow Up
            // Clear current line
            while (g_line_len > 0) {
                cdc_print("\b \b");
                g_line_len--;
            }
            // Copy last command
            strcpy(g_line_buf, g_last_cmd_buf);
            g_line_len = g_last_cmd_len;
            g_line_buf[g_line_len] = '\0';
            cdc_print(g_line_buf);
        }
        g_escape_state = 0;
        return;
    } else if (c == 0x1B) { // ESC
        g_escape_state = 1;
        return;
    }

    if (c == '\r' || c == '\n') {
        cdc_print("\r\n");
        g_line_buf[g_line_len] = '\0';
        if (g_line_len > 0) {
            strcpy(g_last_cmd_buf, g_line_buf);
            g_last_cmd_len = g_line_len;
            process_command(g_line_buf);
            g_line_len = 0;
        }
        if (g_prompt_printed) {
            cdc_print("ffbserial: ");
        }
    } else if (c == 0x03) { // Ctrl-C
        g_line_len = 0;
        cdc_print("^C\r\n");
        if (g_prompt_printed) {
            cdc_print("ffbserial: ");
        }
    } else if (c == '\b' || c == 0x7F) { // Backspace or DEL
        if (g_line_len > 0) {
            g_line_len--;
            cdc_print("\b \b");
        }
    } else if (g_line_len < sizeof(g_line_buf) - 1) {
        // Allow alphanumeric, space, minus
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || 
            (c >= '0' && c <= '9') || c == ' ' || c == '-') {
            g_line_buf[g_line_len++] = c;
            char echo[2] = {c, '\0'};
            cdc_print(echo);
        }
    }
}
