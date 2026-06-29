// =========================================================================
// USB Descriptors — TinyUSB Callbacks
// =========================================================================
// Provides Device, Configuration, HID Report, and String descriptors.
// The HID Report descriptor is assembled by prepending a Joystick Input
// Report (Report ID 1) and Application Collection wrapper around the
// existing PID descriptor from usb_ffb_descriptors.h.
// =========================================================================

#include <cstring>
#include "tusb.h"
#include "bsp/board_api.h"
#include "hid_device.h"
#include "config.h"
#include "usb_ffb_descriptors.h"

#define EPNUM_HID         0x81
#define EPNUM_CDC_NOTIF   0x82
#define EPNUM_CDC_OUT     0x03
#define EPNUM_CDC_IN      0x83

// TUD_HID_DESCRIPTOR uses sizeof(desc_hid_report) for wDescriptorLength.
// We need to use COMBINED_REPORT_DESC_SIZE instead, so we build manually.
// TUD_HID_DESCRIPTOR(itf, str, proto, report_len, epin, bufsize, poll)
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_CDC_DESC_LEN)

// =========================================================================
// Joystick Input Report Descriptor Fragment (Report ID 1)
// =========================================================================
// This is the joystick input portion that the Arduino library built
// dynamically at runtime. Here we define it statically.
//
// Layout:
//   - 16 buttons (16 bits, no padding needed)
//   - X axis (steering):  16-bit signed (-32767..32767)
//   - Accelerator:        16-bit signed (-32767..32767)
//   - Brake:              16-bit signed (-32767..32767)

static const uint8_t joystick_input_descriptor[] = {
    // Usage Page (Generic Desktop)
    0x05, 0x01,
    // Usage (Joystick)
    0x09, 0x04,
    // Collection (Application) — wraps EVERYTHING (input + PID)
    0xA1, 0x01,

        // Usage (Pointer)
        0x09, 0x01,

        // Report ID (1)
        0x85, 0x01,

        // Collection (Physical)
        0xA1, 0x00,

            // ---- Buttons (16) ----
            // Usage Page (Button)
            0x05, 0x09,
            // Usage Minimum (Button 1)
            0x19, 0x01,
            // Usage Maximum (Button 16)
            0x29, 0x10,
            // Logical Minimum (0)
            0x15, 0x00,
            // Logical Maximum (1)
            0x25, 0x01,
            // Report Size (1)
            0x75, 0x01,
            // Report Count (16)
            0x95, 0x10,
            // Unit Exponent (0)
            0x55, 0x00,
            // Unit (None)
            0x65, 0x00,
            // Input (Data, Var, Abs)
            0x81, 0x02,

            // ---- Axes ----
            // Usage Page (Generic Desktop)
            0x05, 0x01,
            // Usage (Pointer)
            0x09, 0x01,
            // Logical Minimum (-32767)
            0x16, 0x01, 0x80,
            // Logical Maximum (+32767)
            0x26, 0xFF, 0x7F,
            // Report Size (16)
            0x75, 0x10,
            // Report Count (1)
            0x95, 0x01,

            // Collection (Physical)
            0xA1, 0x00,
                // Usage (X) — Steering
                0x09, 0x30,
                // Input (Data, Var, Abs)
                0x81, 0x02,
            // End Collection (Physical)
            0xC0,

            // ---- Simulation Controls (Pedals) ----
            // Usage Page (Simulation Controls)
            0x05, 0x02,
            // Logical Minimum (-32767)
            0x16, 0x01, 0x80,
            // Logical Maximum (+32767)
            0x26, 0xFF, 0x7F,
            // Report Size (16)
            0x75, 0x10,
            // Report Count (2)
            0x95, 0x02,

            // Collection (Physical)
            0xA1, 0x00,
                // Usage (Accelerator)
                0x09, 0xC4,
                // Usage (Brake)
                0x09, 0xC5,
                // Input (Data, Var, Abs)
                0x81, 0x02,
            // End Collection (Physical)
            0xC0,

        // End Collection (Physical — outer)
        0xC0,

    // NOTE: The Application Collection (0xA1, 0x01) is NOT closed here.
    // It continues into the PID descriptor and is closed by the final
    // 0xC0 in hid_report_descriptor[].
};

// =========================================================================
// Combined HID Report Descriptor
// =========================================================================
// Concatenates: joystick_input_descriptor + hid_report_descriptor
// The joystick_input_descriptor opens the Application Collection.
// The hid_report_descriptor (from usb_ffb_descriptors.h) contains all
// PID reports and closes the Application Collection with its final 0xC0.

static const uint16_t COMBINED_REPORT_DESC_SIZE =
    sizeof(joystick_input_descriptor) + sizeof(hid_report_descriptor);

static uint8_t combined_hid_report_descriptor[
    sizeof(joystick_input_descriptor) + sizeof(hid_report_descriptor)
];

static bool combined_descriptor_built = false;

static void build_combined_descriptor() {
    if (combined_descriptor_built) return;
    memcpy(combined_hid_report_descriptor,
           joystick_input_descriptor,
           sizeof(joystick_input_descriptor));
    memcpy(combined_hid_report_descriptor + sizeof(joystick_input_descriptor),
           hid_report_descriptor,
           sizeof(hid_report_descriptor));
    combined_descriptor_built = true;
}

// =========================================================================
// Device Descriptor
// =========================================================================

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

// =========================================================================
// Configuration Descriptor
// =========================================================================
// Single HID interface with IN endpoint (for input reports)
// OUT reports come via control pipe (SET_REPORT), which TinyUSB handles
// through tud_hid_set_report_cb without needing an OUT endpoint descriptor.

enum {
    ITF_NUM_HID = 0,
    ITF_NUM_CDC = 1,
    ITF_NUM_CDC_DATA = 2,
    ITF_NUM_TOTAL = 3
};


static uint8_t desc_configuration[CONFIG_TOTAL_LEN];
static bool config_built = false;

static void build_config_descriptor() {
    if (config_built) return;
    build_combined_descriptor();

    // Config descriptor
    uint8_t const config_desc[] = {
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                              TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
        TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE,
                           COMBINED_REPORT_DESC_SIZE, EPNUM_HID,
                           CFG_TUD_HID_EP_BUFSIZE, (uint8_t)(USB_REPORT_INTERVAL_US / 1000)),
        TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8,
                           EPNUM_CDC_OUT, EPNUM_CDC_IN, 64)
    };

    static_assert(sizeof(config_desc) == CONFIG_TOTAL_LEN, "Config descriptor size mismatch");
    memcpy(desc_configuration, config_desc, sizeof(config_desc));
    config_built = true;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    build_config_descriptor();
    return desc_configuration;
}

// =========================================================================
// HID Report Descriptor Callback
// =========================================================================

uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    build_combined_descriptor();
    return combined_hid_report_descriptor;
}

// =========================================================================
// String Descriptors
// =========================================================================

static const char* string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },   // 0: Supported language (English 0x0409)
    "Arheisel",                      // 1: Manufacturer
    "FFB Steering Wheel",            // 2: Product
    nullptr,                         // 3: Serial (uses board unique ID)
    "FFBSerial Debug",                // 4: CDC interface string
};

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t desc_str[32 + 1];

    size_t chr_count;

    switch (index) {
        case 0:
            memcpy(&desc_str[1], string_desc_arr[0], 2);
            chr_count = 1;
            break;

        case 3:
            chr_count = board_usb_get_serial(desc_str + 1, 32);
            break;

        default:
            if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
                return nullptr;
            }
            const char* str = string_desc_arr[index];
            if (!str) return nullptr;
            chr_count = strlen(str);
            if (chr_count > 31) chr_count = 31;
            for (size_t i = 0; i < chr_count; i++) {
                desc_str[1 + i] = str[i];
            }
            break;
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}
