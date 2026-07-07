#pragma once
#include <cstdint>

// =========================================================================
// GPIO PIN ASSIGNMENTS & PERIPHERAL MAPPING
// =========================================================================
// RP2040 peripheral instances are fixed by pin selection.
// Derivation formulas (from RP2040 GPIO function table):
//   I2C instance = (SDA_pin / 2) % 2
//   SPI instance = (RX_pin / 8) % 2
//   ADC channel  = pin - 26

// I2C — AS5600 magnetic encoder
constexpr uint8_t PIN_I2C_SDA          = 14;
constexpr uint8_t PIN_I2C_SCL          = 15;

// SPI — HCF4021B shift registers (buttons)
constexpr uint8_t PIN_SPI_SCK          = 18;
constexpr uint8_t PIN_SPI_RX           = 16;   // MISO
constexpr uint8_t PIN_SPI_LATCH        = 19;   // Directly wired parallel/serial control

// PWM — BTS7960 H-Bridge
constexpr uint8_t PIN_PWM_LPWM        = 7;
constexpr uint8_t PIN_PWM_RPWM        = 6;
constexpr uint8_t PIN_PWM_EN          = 5;     // Bridged left/right enable

// ADC — Analog pedals (0–3V)
constexpr uint8_t PIN_ADC_ACCEL       = 26;
constexpr uint8_t PIN_ADC_BRAKE       = 27;
constexpr uint8_t PIN_ADC_VBUS        = 28;

// Status LED
constexpr uint8_t PIN_LED             = 22;

// =========================================================================
// WHEEL GEOMETRY (Raw Integer Units)
// =========================================================================
// AS5600: 12-bit → 4096 counts per encoder revolution
// Gear ratio 1:2 → 8192 encoder counts per wheel revolution
// All angles expressed in raw encoder counts, NOT degrees.

constexpr uint16_t ENCODER_COUNTS_PER_REV  = 4096;
constexpr uint16_t WHEEL_COUNTS_PER_REV    = ENCODER_COUNTS_PER_REV * 2;  // 8192

// Physical wheel range default: 1800° total → ±900° from center
constexpr uint16_t DEFAULT_MAX_WHEEL_ANGLE_DEG = 1800;

// =========================================================================
// CALIBRATION & PHYSICS TUNING
// =========================================================================

constexpr uint32_t LONG_PRESS_US         = 5'000'000; // Hold > 5s for Flash cal

// Overpower Detection
constexpr int32_t DYNAMIC_DAMPING_FACTOR = 50;      // Tuning parameter for overpower opposition
// Increased margin from 2000 to 8000 to prevent closed-loop oscillation caused by imperfect LUTs
constexpr int32_t VELOCITY_MARGIN_CPS    = 2000;       // Safety margin (counts/sec) for imperfect LUT readings

// Force levels for speed LUT calibration sweeps (raw -10000 to +10000 scale)
// Scaled to stay under the 140 RPM speed limiter (to get an accurate curve)
constexpr int32_t  CAL_FORCE_LEVELS[]    = {
    500,
    800,
    1000,
    1500,
    2500,
};
constexpr uint8_t  CAL_FORCE_LEVEL_COUNT = sizeof(CAL_FORCE_LEVELS) / sizeof(CAL_FORCE_LEVELS[0]);

// Timeouts and Minimum sweep distance during calibration
constexpr uint32_t CAL_I2C_TIMEOUT_US    = 5000;
constexpr uint32_t CAL_SWEEP_TIMEOUT_US  = 10'000'000;
constexpr uint32_t CAL_SETTLE_TIMEOUT_US = 5'000'000;
constexpr int16_t  CAL_ZERO_MIN_SWEEP_COUNTS = WHEEL_COUNTS_PER_REV / 16;
constexpr int16_t  CAL_FORCE_MIN_SWEEP_COUNTS = WHEEL_COUNTS_PER_REV;

// =========================================================================
// MOTOR CONFIGURATION
// =========================================================================
// Target: 20kHz PWM
// RP2040 system clock: 125 MHz
// Edge-aligned: TOP = 125_000_000 / 20_000 - 1 = 6249
// Duty cycle range: 0 to PWM_WRAP (6249)

constexpr uint16_t PWM_WRAP              = 6249;   // TOP value, duty 0..6249

// Flash Default values
constexpr uint32_t DEFAULT_FORCE_GAIN_PERCENT   = 100;
constexpr uint16_t DEFAULT_FRICTION_FADE_FORCE   = 50;

// =========================================================================
// MOTOR SAFETY (Stall Governor)
// =========================================================================

// Dead-time between direction changes to prevent shoot-through (microseconds)
constexpr uint16_t DEAD_TIME_US             = 50;

// Stall protection governor:
// Tuned for 12V 775 DC motor to limit current to a maximum of 15A
constexpr uint16_t PEAK_STALL_PWM           = 3749;  // ~60% duty cycle
constexpr uint16_t CONT_STALL_PWM           = 1874;  // ~30% duty cycle
constexpr uint32_t PEAK_FALLOFF_TIME_US      = 10'000'000; // 10s
constexpr uint32_t PEAK_RECOVERY_PENALTY     = 3;  // How much slower the timer recharges

// =========================================================================
// PROTECTION ENVELOPE (Speed Limiter)
// =========================================================================
// Soft speed limiter to protect the user from dangerously fast wheel spins.
// Limits forward accelerating torque, but allows braking/damping forces.
constexpr int32_t VELOCITY_FADE_START_CPS    = 15'000; // ~110 RPM. Start reducing max PWM.
constexpr int32_t MAX_SAFE_VELOCITY_CPS      = 19'000; // ~140 RPM. PWM reduced to 0.

// =========================================================================
// AS5600 SENSOR
// =========================================================================

constexpr uint8_t  I2C_INSTANCE         = (PIN_I2C_SDA / 2) % 2;
constexpr uint32_t I2C_FREQ_HZ           = 100'000;  // 100 kHz
constexpr uint32_t I2C_READ_INTERVAL_US  = 1000;    // 1 ms
constexpr uint32_t I2C_WATCHDOG_TIMEOUT_US = 5000;  // 5 ms — motor kill if missed

constexpr uint8_t  AS5600_I2C_ADDR       = 0x36;
constexpr uint8_t  AS5600_REG_CONF       = 0x07;
constexpr uint8_t  AS5600_REG_STATUS     = 0x0B;
constexpr uint8_t  AS5600_REG_RAW_ANGLE_H = 0x0C;
constexpr uint8_t  AS5600_REG_RAW_ANGLE_L = 0x0D;

// Initial CONF register values for AS5600 (PM=00, SF=10, FTH=100, WD=0)
constexpr uint8_t  AS5600_CONF_VALUE_H   = 0x12;
constexpr uint8_t  AS5600_CONF_VALUE_L   = 0x00;
constexpr uint8_t  AS5600_CONF_RETRY_COUNT = 3;

// AS5600 status register bits
constexpr uint8_t  AS5600_STATUS_MH      = 0x08;    // Magnet too strong
constexpr uint8_t  AS5600_STATUS_ML      = 0x10;    // Magnet too weak
constexpr uint8_t  AS5600_STATUS_MD      = 0x20;    // Magnet detected

// Maximum physically possible delta between reads (raw counts).
// Any delta exceeding this is a sensor glitch and must be discarded.
// 30000 counts/sec = ~220 RPM. The soft limiter kicks in at 19000 counts/sec.
constexpr int32_t  MAX_PHYSICAL_DELTA_CPS     = 30'000;

// Number of consecutive bad magnet reads before motor is killed.
// Allows the motor to coast through 1-2ms EMI-induced sensor glitches.
constexpr uint8_t  MAGNET_ERROR_TOLERANCE_FRAMES = 3;

// Velocity EMA filter depth for motor governor noise suppression.
// filtered += (raw - filtered) / N  — N=8 gives ~8ms time constant at 1ms sample rate.
constexpr int32_t VELOCITY_EMA_N         = 8;

// =========================================================================
// BUTTON READING (SPI)
// =========================================================================

constexpr uint8_t  SPI_INSTANCE         = (PIN_SPI_RX / 8) % 2;
constexpr uint32_t SPI_FREQ_HZ           = 100'000;  // 100 kHz (Matches I2C for better motor noise immunity)
constexpr uint8_t  BUTTON_COUNT          = 16;       // 2x HCF4021B = 16 bits
constexpr uint8_t  DEBOUNCE_READS        = 4;        // Rolling buffer depth
constexpr uint32_t BUTTON_UPDATE_INTERVAL_US = 2000; // 500 Hz

// =========================================================================
// PEDAL READING (ADC)
// =========================================================================

constexpr uint8_t  ADC_CHANNEL_ACCEL     = PIN_ADC_ACCEL - 26;
constexpr uint8_t  ADC_CHANNEL_BRAKE     = PIN_ADC_BRAKE - 26;
constexpr uint8_t  ADC_CHANNEL_VBUS      = PIN_ADC_VBUS - 26;
constexpr uint8_t  ADC_FILTER_DEPTH      = 20;      // 20 samples, discard top/bottom 2, average 16
constexpr uint32_t PEDAL_UPDATE_INTERVAL_US = 500;  // 0.5ms (2000 Hz)

// =========================================================================
// LED STATUS CONTROLLER
// =========================================================================

constexpr uint32_t LED_UPDATE_INTERVAL_US = 10'000; // 100 Hz (10ms)
constexpr uint32_t LED_UPDATE_INTERVAL_MS = LED_UPDATE_INTERVAL_US / 1000;
constexpr uint32_t LED_FLASH_ON_MS       = 200;
constexpr uint32_t LED_FLASH_OFF_MS      = 500;
constexpr uint32_t LED_PAUSE_MS          = 2000;
constexpr uint32_t LED_RAPID_FLASH_MS    = 50;
// Minimum number of complete flash cycles before a clear() takes effect.
// Ensures transient errors are visible long enough to read.
constexpr uint8_t  LED_MIN_DISPLAY_CYCLES = 2;

// =========================================================================
// USB & FFB CONFIGURATION
// =========================================================================

constexpr uint16_t USB_VID                = 0xCAFE;
constexpr uint16_t USB_PID                = 0x4003;
constexpr uint32_t USB_REPORT_INTERVAL_US = 1000;
constexpr uint8_t  MAX_EFFECTS            = 40;

// =========================================================================
// STATUS / ERROR CODES  (LED flash code = enum value)
// =========================================================================
// Priority: highest numeric value wins.

enum class SystemStatus : uint8_t {
    Normal            = 0,   // Solid ON — no flash code
    BootWait          = 1,   // Awaiting button press to boot or enter calibration
    MotorSweepsActive = 2,  // Flash calibration: Motor sweeps in progress
    PedalCalActive    = 3,   // Flash calibration: Pedal calibration in progress
    FlashCalMissing   = 5,   // No valid flash calibration data
    FlashWriteFailed  = 6,  // Failed to save calibration data to flash
    MagnetHigh        = 7,   // AS5600: magnet too strong (MH=1)
    MagnetLow         = 8,   // AS5600: magnet too weak (ML=1)
    MagnetMissing     = 9,   // AS5600: no magnet detected (MD=0)
    I2CWatchdogFired  = 10,  // I2C DMA watchdog expired — motor killed
    EncoderDesync     = 11,  // Repeated impossible jumps — motor killed
    DesyncAfterRecovery = 12, // Impossible jump on first read after watchdog recovery
    EncoderConfWriteFailed  = 13,  // AS5600 CONF register write failed after retries
    RapidFlash        = 254  // Special mode: rapid continuous flashing, clearable immediately
};

constexpr uint8_t ERROR_LOG_SIZE = 16;