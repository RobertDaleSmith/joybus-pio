#ifndef _JOYBUS_N64_DEFINITIONS_H
#define _JOYBUS_N64_DEFINITIONS_H

#include <pico/stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// N64 Joybus Commands
typedef enum {
    N64Command_PROBE = 0x00,
    N64Command_RESET = 0xFF,
    N64Command_POLL = 0x01,
    N64Command_READ_EXPANSION_BUS = 0x02,
    N64Command_WRITE_EXPANSION_BUS = 0x03,
} N64Command;

// N64 Controller Pak / Rumble Pak addresses
#define N64_RUMBLE_PAK_ADDR     0xC000  // Rumble pak control register
#define N64_PAK_DATA_SIZE       32      // Controller pak read/write size

// N64 Pak status bits (from probe response status byte)
#define N64_STATUS_PAK_PRESENT  0x01    // Controller/Rumble pak inserted
#define N64_STATUS_PAK_CHANGED  0x02    // Pak was removed/inserted

// N64 Controller Report (4 bytes)
typedef struct __attribute__((packed)) {
    bool dpad_right : 1;
    bool dpad_left : 1;
    bool dpad_down : 1;
    bool dpad_up : 1;
    bool start : 1;
    bool z : 1;
    bool b : 1;
    bool a : 1;

    bool c_right : 1;
    bool c_left : 1;
    bool c_down : 1;
    bool c_up : 1;
    bool r : 1;
    bool l : 1;
    uint8_t reserved1 : 1;
    uint8_t reserved0 : 1;

    int8_t stick_x;   // Signed: -128 to +127 (center = 0)
    int8_t stick_y;   // Signed: -128 to +127 (center = 0)
} n64_report_t;

// N64 Controller Status (response to PROBE command)
typedef struct __attribute__((packed)) {
    uint16_t device;
    uint8_t status;
} n64_status_t;

// Default report initializer (all buttons released, stick centered)
#define DEFAULT_N64_REPORT_INITIALIZER { \
    .dpad_right = 0, \
    .dpad_left = 0, \
    .dpad_down = 0, \
    .dpad_up = 0, \
    .start = 0, \
    .z = 0, \
    .b = 0, \
    .a = 0, \
    .c_right = 0, \
    .c_left = 0, \
    .c_down = 0, \
    .c_up = 0, \
    .r = 0, \
    .l = 0, \
    .reserved1 = 0, \
    .reserved0 = 0, \
    .stick_x = 0, \
    .stick_y = 0, \
}

// Default status (standard controller, pak present)
// status bit 0 (0x01): accessory/pak is connected
// status bit 1 (0x02): address CRC was reset (pak changed since last check)
// 0x01 = pak present, stable. Console will read pak to identify type.
#define DEFAULT_N64_STATUS_INITIALIZER { \
    .device = 0x0005, \
    .status = 0x01, \
}

#ifdef __cplusplus
}
#endif

#endif // _JOYBUS_N64_DEFINITIONS_H
