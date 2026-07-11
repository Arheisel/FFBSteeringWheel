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
#include <type_traits>

namespace
{

    // =========================================================================
    // Error Log — Lock-free ring buffer (single writer from ISR, single reader)
    // =========================================================================

    struct ErrorLogEntry
    {
        uint64_t timestamp_us;
        SystemStatus error_code;
    };

    ErrorLogEntry g_error_log[ERROR_LOG_SIZE];
    volatile uint8_t g_error_log_write_idx = 0;
    uint8_t g_error_log_read_idx = 0;

    SharedState *g_state = nullptr;
    PedalReader *g_pedals = nullptr;
    FlashStorage *g_flash = nullptr;

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

    void cdc_print(const char *str)
    {
        if (!tud_cdc_connected())
            return;
        uint32_t len = strlen(str);
        uint32_t sent = 0;
        while (sent < len)
        {
            if (!tud_cdc_connected())
                return; // Guard against mid-write disconnect
            uint32_t avail = tud_cdc_write_available();
            if (avail == 0)
            {
                tud_cdc_write_flush();
                // Brief yield to let USB send
                tud_task();
                continue;
            }
            uint32_t chunk = len - sent;
            if (chunk > avail)
                chunk = avail;
            tud_cdc_write(str + sent, chunk);
            sent += chunk;
        }
        tud_cdc_write_flush();
    }

    void cdc_print_char(char c)
    {
        if (!tud_cdc_connected())
            return;
        while (tud_cdc_write_available() == 0)
        {
            if (!tud_cdc_connected())
                return;
            tud_cdc_write_flush();
            tud_task();
        }
        tud_cdc_write(&c, 1);
        tud_cdc_write_flush();
    }

    // =========================================================================
    // SafeBufferWriter & Print Helpers
    // =========================================================================

    template <size_t Size = 128>
    class SafeBufferWriter
    {
    public:
        SafeBufferWriter() : write_idx_(0)
        {
            buf_[0] = '\0';
        }

        void reset()
        {
            write_idx_ = 0;
            if (Size > 0)
            {
                buf_[0] = '\0';
            }
        }

        void append_str(const char *str)
        {
            if (!str || write_idx_ >= Size - 1)
                return;
            while (*str && write_idx_ < Size - 1)
            {
                buf_[write_idx_++] = *str++;
            }
            buf_[write_idx_] = '\0';
        }

        void append_int(int32_t val)
        {
            char temp[12];
            int_to_str(val, temp);
            append_str(temp);
        }

        void append_uint(uint32_t val)
        {
            char temp[12];
            uint_to_str(val, temp);
            append_str(temp);
        }

        void append_hex16(uint16_t val)
        {
            const char hex_chars[] = "0123456789ABCDEF";
            char temp[5];
            temp[0] = hex_chars[(val >> 12) & 0xF];
            temp[1] = hex_chars[(val >> 8) & 0xF];
            temp[2] = hex_chars[(val >> 4) & 0xF];
            temp[3] = hex_chars[val & 0xF];
            temp[4] = '\0';
            append_str(temp);
        }

        const char *c_str() const
        {
            return buf_;
        }

    private:
        char buf_[Size];
        size_t write_idx_;

        static char *int_to_str(int32_t val, char *buf)
        {
            if (val < 0)
            {
                *buf++ = '-';
                // Handle INT32_MIN edge case
                if (val == -2147483647 - 1)
                {
                    strcpy(buf, "2147483648");
                    return buf + 10;
                }
                val = -val;
            }
            // Write digits in reverse
            char tmp[11];
            int i = 0;
            if (val == 0)
            {
                tmp[i++] = '0';
            }
            else
            {
                while (val > 0)
                {
                    tmp[i++] = '0' + (val % 10);
                    val /= 10;
                }
            }
            // Reverse into buf
            for (int j = i - 1; j >= 0; j--)
            {
                *buf++ = tmp[j];
            }
            *buf = '\0';
            return buf;
        }

        static char *uint_to_str(uint32_t val, char *buf)
        {
            char tmp[11];
            int i = 0;
            if (val == 0)
            {
                tmp[i++] = '0';
            }
            else
            {
                while (val > 0)
                {
                    tmp[i++] = '0' + (val % 10);
                    val /= 10;
                }
            }
            for (int j = i - 1; j >= 0; j--)
            {
                *buf++ = tmp[j];
            }
            *buf = '\0';
            return buf;
        }
    };

    template <typename T, typename = typename std::enable_if<std::is_integral<T>::value>::type>
    void cdc_print_line(const char *label, T value)
    {
        SafeBufferWriter<> writer;
        writer.append_str(label);
        if constexpr (std::is_signed<T>::value)
        {
            writer.append_int(static_cast<int32_t>(value));
        }
        else
        {
            writer.append_uint(static_cast<uint32_t>(value));
        }
        writer.append_str("\r\n");
        cdc_print(writer.c_str());
    }

    void cdc_print_line(const char *label, const char *value)
    {
        SafeBufferWriter<> writer;
        writer.append_str(label);
        writer.append_str(value);
        writer.append_str("\r\n");
        cdc_print(writer.c_str());
    }

    // =========================================================================
    // Command: Print live calibration data
    // =========================================================================

    void cmd_calibration()
    {
        if (!g_state || !g_pedals)
        {
            cdc_print("ERR: No state\r\n");
            return;
        }

        cdc_print("=== CALIBRATION DATA ===\r\n");

        cdc_print_line("Center: ", g_state->cal_state.center_offset.load());
        cdc_print_line("Wheel Angle (deg): ", g_state->cal_state.wheel_angle_deg.load());
        cdc_print_line("System Damper: ", g_state->cal_state.system_damper_strength.load());
        cdc_print_line("Force Gain %: ", g_state->cal_state.force_gain_percent.load());
        cdc_print_line("Friction Fade Force: ", g_state->cal_state.friction_fade_force.load());
        cdc_print_line("CW Zero PWM: ", g_state->cal_state.cw_zero_pwm.load());
        cdc_print_line("CCW Zero PWM: ", g_state->cal_state.ccw_zero_pwm.load());

        uint16_t amin = g_state->cal_state.accel_min.load();
        uint16_t amax = g_state->cal_state.accel_max.load();
        uint16_t bmin = g_state->cal_state.brake_min.load();
        uint16_t bmax = g_state->cal_state.brake_max.load();

        SafeBufferWriter<> writer;

        writer.append_str("Accel: ");
        writer.append_uint(amin);
        writer.append_str("-");
        writer.append_uint(amax);
        writer.append_str("\r\n");
        cdc_print(writer.c_str());

        writer.reset();
        writer.append_str("Brake: ");
        writer.append_uint(bmin);
        writer.append_str("-");
        writer.append_uint(bmax);
        writer.append_str("\r\n");
        cdc_print(writer.c_str());

        writer.reset();
        writer.append_str("CW Speed LUT:");
        for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++)
        {
            writer.append_str(" ");
            writer.append_int(g_state->cal_state.cw_speed_lut[i].load());
        }
        writer.append_str("\r\n");
        cdc_print(writer.c_str());

        writer.reset();
        writer.append_str("CCW Speed LUT:");
        for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++)
        {
            writer.append_str(" ");
            writer.append_int(g_state->cal_state.ccw_speed_lut[i].load());
        }
        writer.append_str("\r\n");
        cdc_print(writer.c_str());
    }

    // =========================================================================
    // Command: Save calibration data
    // =========================================================================

    void cmd_save_calibration()
    {
        if (!g_state || !g_pedals || !g_flash)
            return;

        cdc_print("Saving to flash...\r\n");

        // Core 1 is spinning, but we still pass core1_running=true so flash_safe_execute
        // puts Core 1 into a lockout state cleanly before pausing Core 0 interrupts.
        bool ok = g_flash->save(g_state->cal_state, true);

        cdc_print(ok ? "Save OK\r\n" : "Save FAILED\r\n");
    }

    // =========================================================================
    // Command: Print live status
    // =========================================================================

    void print_error_flags(uint8_t flags)
    {
        SafeBufferWriter<128> writer;
        writer.append_str("Err flags: ");
        writer.append_uint(flags);

        if (flags == 0)
        {
            writer.append_str(" (None)\r\n");
        }
        else
        {
            writer.append_str(" (");
            bool first = true;
            if (flags & SensorState::ERR_MAGNET_HIGH)
            {
                writer.append_str("MagnetHigh");
                first = false;
            }
            if (flags & SensorState::ERR_MAGNET_LOW)
            {
                if (!first)
                    writer.append_str(", ");
                writer.append_str("MagnetLow");
                first = false;
            }
            if (flags & SensorState::ERR_MAGNET_MISSING)
            {
                if (!first)
                    writer.append_str(", ");
                writer.append_str("MagnetMissing");
                first = false;
            }
            if (flags & SensorState::ERR_I2C_WATCHDOG)
            {
                if (!first)
                    writer.append_str(", ");
                writer.append_str("I2CWatchdog");
                first = false;
            }
            if (flags & SensorState::ERR_DESYNC)
            {
                if (!first)
                    writer.append_str(", ");
                writer.append_str("Desync");
                first = false;
            }
            if (flags & SensorState::ERR_RECOVERY_DESYNC)
            {
                if (!first)
                    writer.append_str(", ");
                writer.append_str("RecDesync");
                first = false;
            }
            writer.append_str(")\r\n");
        }
        cdc_print(writer.c_str());
    }

    void cmd_status()
    {
        if (!g_state)
        {
            cdc_print("ERR: No state\r\n");
            return;
        }

        cdc_print("=== LIVE STATUS ===\r\n");

        cdc_print_line("Position: ", g_state->sensor.wheel_position.load());
        cdc_print_line("Velocity (cps): ", g_state->sensor.wheel_velocity.load());
        cdc_print_line("Accel: ", g_state->pedal_accel.load());
        cdc_print_line("Brake: ", g_state->pedal_brake.load());

        uint8_t flags = g_state->sensor.error_flags.load();
        print_error_flags(flags);

        cdc_print_line("LED status: ", static_cast<uint8_t>(g_state->led_status.get()));

        uint8_t err_count = (g_error_log_write_idx - g_error_log_read_idx + ERROR_LOG_SIZE) % ERROR_LOG_SIZE;
        cdc_print_line("Logged errors: ", err_count);

        // Request AGC register read from Core 1
        g_state->request_agc_read.store(true);

        // Poll for completion (timeout after 10ms)
        uint32_t start = time_us_32();
        while (g_state->request_agc_read.load())
        {
            if (time_us_32() - start > 10000)
            {
                cdc_print("AGC: timeout\r\n");
                g_state->request_agc_read.store(false);
                return;
            }
            tud_task(); // Keep USB alive while waiting
        }

        cdc_print_line("AGC: ", g_state->sensor.agc_value.load());

        SafeBufferWriter<> writer;

        writer.append_str("Buttons: 0x");
        writer.append_hex16(g_state->buttons.load());
        writer.append_str("\r\n");
        cdc_print(writer.c_str());

        writer.reset();
        writer.append_str("Loop time (EMA): ");
        writer.append_uint(g_state->loop_time_avg_us.load());
        writer.append_str(" us\r\n");
        cdc_print(writer.c_str());
    }

    // =========================================================================
    // Command: Print and clear error log
    // =========================================================================

    const char *error_name(SystemStatus code)
    {
        switch (code)
        {
        case SystemStatus::FlashCalMissing:
            return "FlashCalMissing";
        case SystemStatus::MagnetHigh:
            return "MagnetHigh";
        case SystemStatus::MagnetLow:
            return "MagnetLow";
        case SystemStatus::MagnetMissing:
            return "MagnetMissing";
        case SystemStatus::I2CWatchdogFired:
            return "I2CWatchdog";
        case SystemStatus::EncoderDesync:
            return "Desync";
        case SystemStatus::DesyncAfterRecovery:
            return "RecoveryDesync";
        case SystemStatus::FlashWriteFailed:
            return "FlashWriteFailed";
        default:
            return "Unknown";
        }
    }

    void cmd_errors()
    {
        uint8_t write_idx = g_error_log_write_idx;
        uint8_t read_idx = g_error_log_read_idx;

        if (read_idx == write_idx)
        {
            cdc_print("=== ERROR LOG: Empty ===\r\n");
            return;
        }

        cdc_print("=== ERROR LOG ===\r\n");

        int count = 0;
        SafeBufferWriter<> writer;

        while (read_idx != write_idx && count < ERROR_LOG_SIZE)
        {
            ErrorLogEntry &e = g_error_log[read_idx];
            writer.reset();
            writer.append_str("[");
            writer.append_uint(static_cast<uint32_t>(e.timestamp_us / 1000));
            writer.append_str("ms | Code ");
            writer.append_uint(static_cast<uint8_t>(e.error_code));
            writer.append_str(" (");
            writer.append_str(error_name(e.error_code));
            writer.append_str(")\r\n");
            cdc_print(writer.c_str());

            read_idx = (read_idx + 1) % ERROR_LOG_SIZE;
            count++;
        }

        // Clear the log
        g_error_log_read_idx = write_idx;

        writer.reset();
        writer.append_str("Total: ");
        writer.append_int(count);
        writer.append_str(" entries (cleared)\r\n");
        cdc_print(writer.c_str());
    }

    // =========================================================================
    // Command Parser
    // =========================================================================

    // =========================================================================
    // Command Parser Settings Registries & Setters
    // =========================================================================

    void set_amin(int32_t val) { g_state->cal_state.accel_min.store(val); g_pedals->apply_calibration(g_state->cal_state); }
    int32_t get_amin() { return g_state->cal_state.accel_min.load(); }

    void set_amax(int32_t val) { g_state->cal_state.accel_max.store(val); g_pedals->apply_calibration(g_state->cal_state); }
    int32_t get_amax() { return g_state->cal_state.accel_max.load(); }

    void set_bmin(int32_t val) { g_state->cal_state.brake_min.store(val); g_pedals->apply_calibration(g_state->cal_state); }
    int32_t get_bmin() { return g_state->cal_state.brake_min.load(); }

    void set_bmax(int32_t val) { g_state->cal_state.brake_max.store(val); g_pedals->apply_calibration(g_state->cal_state); }
    int32_t get_bmax() { return g_state->cal_state.brake_max.load(); }

    void set_cwz(int32_t val) { g_state->cal_state.cw_zero_pwm.store(static_cast<uint16_t>(val)); g_state->calibration_reload.store(true); }
    int32_t get_cwz() { return g_state->cal_state.cw_zero_pwm.load(); }

    void set_ccz(int32_t val) { g_state->cal_state.ccw_zero_pwm.store(static_cast<uint16_t>(val)); g_state->calibration_reload.store(true); }
    int32_t get_ccz() { return g_state->cal_state.ccw_zero_pwm.load(); }

    void set_center(int32_t val) { g_state->cal_state.center_offset.store(val); g_state->calibration_reload.store(true); }
    int32_t get_center() { return g_state->cal_state.center_offset.load(); }

    void set_angle(int32_t val) {
        g_state->cal_state.wheel_angle_deg.store(val);
        int32_t half_deg = val / 2;
        int32_t max_half_angle_counts = (half_deg * WHEEL_COUNTS_PER_REV) / 360;
        g_state->cal_state.max_half_angle_counts.store(max_half_angle_counts);
        g_state->calibration_reload.store(true);
    }
    int32_t get_angle() { return g_state->cal_state.wheel_angle_deg.load(); }

    void set_damper(int32_t val) { g_state->cal_state.system_damper_strength.store(val); g_state->calibration_reload.store(true); }
    int32_t get_damper() { return g_state->cal_state.system_damper_strength.load(); }

    void set_gain(int32_t val) { g_state->cal_state.force_gain_percent.store(val); g_state->calibration_reload.store(true); }
    int32_t get_gain() { return g_state->cal_state.force_gain_percent.load(); }

    void set_friction(int32_t val) { g_state->cal_state.friction_fade_force.store(val); g_state->calibration_reload.store(true); }
    int32_t get_friction() { return g_state->cal_state.friction_fade_force.load(); }

    // Registries structures
    struct CommandDefinition
    {
        const char *name;
        void (*handler)(int argc, char **argv);
        const char *help_desc;
        bool show_in_list;
    };

    struct SettingDefinition
    {
        const char *name;
        const char *help_desc;
        int32_t min_val;
        int32_t max_val;
        void (*set_fn)(int32_t val);
        int32_t (*get_fn)();
    };

    // Forward declarations of handlers so they can be placed in COMMANDS table
    void handle_status(int argc, char **argv) { cmd_status(); }
    void handle_calibration(int argc, char **argv) { cmd_calibration(); }
    void handle_errors(int argc, char **argv) { cmd_errors(); }
    void handle_save(int argc, char **argv) { cmd_save_calibration(); }
    void print_help();
    void handle_help(int argc, char **argv) { print_help(); }
    void cmd_cs(int argc, char **argv);
    void handle_cs(int argc, char **argv) { cmd_cs(argc, argv); }

    constexpr CommandDefinition COMMANDS[] = {
        {"s",    handle_status,      "Print live status",                     true},
        {"c",    handle_calibration, "Print live calibration data",           true},
        {"e",    handle_errors,      "Print error log",                       true},
        {"cw",   handle_save,        "Write live calibration data to flash",  true},
        {"cs",   handle_cs,          "Configuration Settings",                false},
        {"help", handle_help,        "Print this help",                       true}
    };

    constexpr SettingDefinition SETTINGS[] = {
        {"amin",     "Set accelerator min",      0, 4095,  set_amin,     get_amin},
        {"amax",     "Set accelerator max",      0, 4095,  set_amax,     get_amax},
        {"bmin",     "Set brake min",            0, 4095,  set_bmin,     get_bmin},
        {"bmax",     "Set brake max",            0, 4095,  set_bmax,     get_bmax},
        {"cwz",      "Set CW zero PWM",          0, 6249,  set_cwz,      get_cwz},
        {"ccz",      "Set CCW zero PWM",         0, 6249,  set_ccz,      get_ccz},
        {"center",   "Set wheel center offset",  0, 4095,  set_center,   get_center},
        {"angle",    "Set max wheel angle",      180, 1080,set_angle,    get_angle},
        {"damper",   "Set system damper",        0, 10000, set_damper,   get_damper},
        {"gain",     "Set force gain %",         0, 100,   set_gain,     get_gain},
        {"friction", "Set friction fade force",  1, 9999,  set_friction, get_friction}
    };

    // Compile-time string builder helper
    template <size_t N>
    struct CompileTimeString
    {
        char data[N]{};
        size_t length = 0;

        constexpr void append(const char *str)
        {
            while (*str)
            {
                if (length < N - 1)
                {
                    data[length++] = *str;
                }
                str++;
            }
        }

        constexpr void append_int(int32_t val)
        {
            if (val == 0)
            {
                if (length < N - 1) data[length++] = '0';
                return;
            }
            if (val < 0)
            {
                if (length < N - 1) data[length++] = '-';
                val = -val;
            }
            char buf[12]{};
            int i = 0;
            while (val > 0)
            {
                buf[i++] = '0' + (val % 10);
                val /= 10;
            }
            for (int j = i - 1; j >= 0; j--)
            {
                if (length < N - 1) data[length++] = buf[j];
            }
        }

        constexpr const char *c_str() const
        {
            return data;
        }
    };

    constexpr size_t constexpr_strlen(const char *str)
    {
        size_t len = 0;
        while (str[len]) len++;
        return len;
    }

    template <size_t N>
    constexpr CompileTimeString<N> generate_help_string()
    {
        CompileTimeString<N> s;
        s.append("Commands:\r\n");

        for (size_t i = 0; i < sizeof(COMMANDS) / sizeof(COMMANDS[0]); ++i)
        {
            if (!COMMANDS[i].show_in_list)
                continue;
            s.append("  ");
            s.append(COMMANDS[i].name);
            
            size_t name_len = constexpr_strlen(COMMANDS[i].name);
            size_t spaces = (name_len < 6) ? (6 - name_len) : 0;
            for (size_t k = 0; k < spaces; ++k)
            {
                s.append(" ");
            }
            s.append("- ");
            s.append(COMMANDS[i].help_desc);
            s.append("\r\n");
        }

        s.append("  cs     - Configuration Settings:\r\n");
        s.append("           cs <lut> <idx> <val> - Set LUT value (lut: cwl, ccl; idx: 0-4)\r\n");

        for (size_t i = 0; i < sizeof(SETTINGS) / sizeof(SETTINGS[0]); ++i)
        {
            s.append("           cs ");
            s.append(SETTINGS[i].name);
            s.append(" <val> ");

            size_t name_len = constexpr_strlen(SETTINGS[i].name);
            size_t spaces = (name_len < 14) ? (14 - name_len) : 0;
            for (size_t k = 0; k < spaces; ++k)
            {
                s.append(" ");
            }
            s.append("- ");
            s.append(SETTINGS[i].help_desc);
            s.append(" (");
            s.append_int(SETTINGS[i].min_val);
            s.append("-");
            s.append_int(SETTINGS[i].max_val);
            s.append(")\r\n");
        }

        return s;
    }

    constexpr auto HELP_STRING = generate_help_string<2048>();

    void print_help()
    {
        cdc_print(HELP_STRING.c_str());
    }

    void cmd_cs(int argc, char **argv)
    {
        if (argc < 3)
        {
            cdc_print("ERR: Usage: cs <var> <val>\r\n");
            print_help();
            return;
        }

        const char *var = argv[1];

        // 1. Handle special array/LUT variables requiring index (cwl, ccl)
        if (strcmp(var, "cwl") == 0 || strcmp(var, "ccl") == 0)
        {
            if (argc == 4)
            {
                int idx = atoi(argv[2]);
                int32_t val = atoi(argv[3]);
                if (idx >= 0 && idx < CAL_FORCE_LEVEL_COUNT)
                {
                    if (strcmp(var, "cwl") == 0)
                        g_state->cal_state.cw_speed_lut[idx].store(val);
                    else
                        g_state->cal_state.ccw_speed_lut[idx].store(val);
                    cdc_print("LUT updated.\r\n");
                    g_state->calibration_reload.store(true);
                }
                else
                {
                    cdc_print("ERR: Invalid index\r\n");
                }
            }
            else
            {
                cdc_print("ERR: Usage: cs <lut> <idx> <val>\r\n");
                print_help();
            }
            return;
        }

        // 2. Handle generic setting table variables
        int32_t val = atoi(argv[2]);
        for (size_t i = 0; i < sizeof(SETTINGS) / sizeof(SETTINGS[0]); ++i)
        {
            if (strcmp(var, SETTINGS[i].name) == 0)
            {
                if (val < SETTINGS[i].min_val || val > SETTINGS[i].max_val)
                {
                    SafeBufferWriter<> writer;
                    writer.append_str("ERR: Value must be ");
                    writer.append_int(SETTINGS[i].min_val);
                    writer.append_str(" to ");
                    writer.append_int(SETTINGS[i].max_val);
                    writer.append_str("\r\n");
                    cdc_print(writer.c_str());
                    return;
                }
                SETTINGS[i].set_fn(val);
                cdc_print("Updated.\r\n");
                return;
            }
        }

        cdc_print("ERR: Unknown variable\r\n");
        print_help();
    }

    void process_command(char *cmd)
    {
        char *argv[5];
        int argc = 0;
        char *p = cmd;
        while (*p)
        {
            while (*p == ' ')
                p++;
            if (!*p)
                break;
            argv[argc++] = p;
            if (argc >= 5)
                break;
            while (*p && *p != ' ')
                p++;
            if (*p)
            {
                *p = '\0';
                p++;
            }
        }

        if (argc == 0)
            return;

        for (size_t i = 0; i < sizeof(COMMANDS) / sizeof(COMMANDS[0]); ++i)
        {
            if (strcmp(argv[0], COMMANDS[i].name) == 0)
            {
                COMMANDS[i].handler(argc, argv);
                return;
            }
        }

        cdc_print("ERR: Unknown command.\r\n");
        print_help();
    }

}

// =========================================================================
// Public API
// =========================================================================

void debug_serial_init(SharedState &state, PedalReader &pedals, FlashStorage &flash)
{
    g_state = &state;
    g_pedals = &pedals;
    g_flash = &flash;
    memset(g_error_log, 0, sizeof(g_error_log));
    g_error_log_write_idx = 0;
    g_error_log_read_idx = 0;
}

void debug_log_error(SystemStatus error_code)
{
    uint8_t idx = g_error_log_write_idx;
    uint8_t next_idx = (idx + 1) % ERROR_LOG_SIZE;

    g_error_log[idx].timestamp_us = time_us_64();
    g_error_log[idx].error_code = error_code;

    // If buffer is full, advance read pointer to drop oldest entry
    if (next_idx == g_error_log_read_idx)
    {
        g_error_log_read_idx = (g_error_log_read_idx + 1) % ERROR_LOG_SIZE;
    }

    g_error_log_write_idx = next_idx;
}

// =========================================================================
// Main update — called from Core 0 main loop
// =========================================================================

void debug_serial_update()
{
    bool connected = tud_cdc_connected();
    if (!connected)
    {
        g_prompt_printed = false;
        g_line_len = 0;
        g_escape_state = 0;
        g_was_connected = false;
        return;
    }

    if (!g_was_connected)
    {
        g_was_connected = true;
        g_connected_timestamp = time_us_64();
    }

    if (!g_prompt_printed)
    {
        // Wait 500ms after connection before sending the first prompt
        // to give the host terminal software time to initialize.
        if (time_us_64() - g_connected_timestamp > 500000)
        {
            cdc_print("\r\nffbserial: ");
            g_prompt_printed = true;
        }
    }

    while (tud_cdc_available() > 0)
    {
        char c = (char)tud_cdc_read_char();

        if (g_escape_state == 1)
        {
            if (c == '[')
                g_escape_state = 2;
            else
                g_escape_state = 0;
            continue;
        }
        else if (g_escape_state == 2)
        {
            if (c == 'A')
            { // Arrow Up
                // Batch backspaces to clear the current line
                if (g_line_len > 0)
                {
                    char clear_buf[193];
                    size_t idx = 0;
                    for (uint8_t i = 0; i < g_line_len; i++)
                    {
                        clear_buf[idx++] = '\b';
                        clear_buf[idx++] = ' ';
                        clear_buf[idx++] = '\b';
                    }
                    clear_buf[idx] = '\0';
                    cdc_print(clear_buf);
                }
                // Copy last command safely
                memcpy(g_line_buf, g_last_cmd_buf, g_last_cmd_len);
                g_line_len = g_last_cmd_len;
                g_line_buf[g_line_len] = '\0';
                cdc_print(g_line_buf);
            }
            g_escape_state = 0;
            continue;
        }
        else if (c == 0x1B)
        { // ESC
            g_escape_state = 1;
            continue;
        }

        if (c == '\r' || c == '\n')
        {
            cdc_print("\r\n");
            g_line_buf[g_line_len] = '\0';
            if (g_line_len > 0)
            {
                memcpy(g_last_cmd_buf, g_line_buf, g_line_len);
                g_last_cmd_len = g_line_len;
                g_last_cmd_buf[g_last_cmd_len] = '\0';
                process_command(g_line_buf);
                g_line_len = 0;
            }
            if (g_prompt_printed)
            {
                cdc_print("ffbserial: ");
            }
        }
        else if (c == 0x03)
        { // Ctrl-C
            g_line_len = 0;
            cdc_print("^C\r\n");
            if (g_prompt_printed)
            {
                cdc_print("ffbserial: ");
            }
        }
        else if (c == '\b' || c == 0x7F)
        { // Backspace or DEL
            if (g_line_len > 0)
            {
                g_line_len--;
                cdc_print("\b \b");
            }
        }
        else if (g_line_len < sizeof(g_line_buf) - 1)
        {
            // Allow alphanumeric, space, minus
            if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                (c >= '0' && c <= '9') || c == ' ' || c == '-')
            {
                g_line_buf[g_line_len++] = c;
                cdc_print_char(c);
            }
        }
    }
}
