#ifndef _JOYBUS_GAMECUBE_DEFINITIONS_H
#define _JOYBUS_GAMECUBE_DEFINITIONS_H

#include <pico/stdlib.h>

typedef enum {
    GamecubeCommand_PROBE = 0x00,
    GamecubeCommand_RESET = 0xFF,
    GamecubeCommand_ORIGIN = 0x41,
    GamecubeCommand_RECALIBRATE = 0x42,
    GamecubeCommand_POLL = 0x40,
    GamecubeCommand_KEYBOARD = 0x54, // poll keyboard report
    GamecubeCommand_GAME_ID = 0x1D,
} GamecubeCommand;

typedef union{
    // 8 bytes of datareport that we get from the controller
    uint8_t raw8[8];
    uint16_t raw16[0];
    uint32_t raw32[0];

    struct{
        uint8_t buttons0;
        union{
            uint8_t buttons1;
            uint8_t dpad : 4;
        };
    };

    struct {
        // first data byte (bitfields are sorted in LSB order)
        uint8_t a : 1;
        uint8_t b : 1;
        uint8_t x : 1;
        uint8_t y : 1;
        uint8_t start : 1;
        uint8_t origin : 1; // Indicates if GetOrigin(0x41) was called (LOW)
        uint8_t errlatch : 1;
        uint8_t errstat : 1;

        // second data byte
        uint8_t dpad_left : 1;
        uint8_t dpad_right : 1;
        uint8_t dpad_down : 1;
        uint8_t dpad_up : 1;
        uint8_t z : 1;
        uint8_t r : 1;
        uint8_t l : 1;
        uint8_t high1 : 1;

        // 3rd-8th data byte
        uint8_t stick_x;
        uint8_t stick_y;
        uint8_t cstick_x;
        uint8_t cstick_y;
        uint8_t l_analog;
        uint8_t r_analog;
    }; // mode3 (default reading mode)

    struct {
        // first data byte (bitfields are sorted in LSB order)
        uint8_t a : 1;
        uint8_t b : 1;
        uint8_t x : 1;
        uint8_t y : 1;
        uint8_t start : 1;
        uint8_t origin : 1; // Indicates if GetOrigin(0x41) was called (LOW)
        uint8_t errlatch : 1;
        uint8_t errstat : 1;

        // second data byte
        uint8_t dpad_left : 1;
        uint8_t dpad_right : 1;
        uint8_t dpad_down : 1;
        uint8_t dpad_up : 1;
        uint8_t z : 1;
        uint8_t r : 1;
        uint8_t l : 1;
        uint8_t high1 : 1;

        // 3rd-8th data byte
        uint8_t stick_x;
        uint8_t stick_y;
        uint8_t cstick_x;
        uint8_t cstick_y;
        /*
        uint8_t l_analog;
        uint8_t right;
        */
        uint8_t l_analog : 4;
        uint8_t r_analog : 4;
        uint8_t analogA : 4;
        uint8_t analogB : 4;
    } mode0;

    struct {
        // first data byte (bitfields are sorted in LSB order)
        uint8_t a : 1;
        uint8_t b : 1;
        uint8_t x : 1;
        uint8_t y : 1;
        uint8_t start : 1;
        uint8_t origin : 1; // Indicates if GetOrigin(0x41) was called (LOW)
        uint8_t errlatch : 1;
        uint8_t errstat : 1;

        // second data byte
        uint8_t dpad_left : 1;
        uint8_t dpad_right : 1;
        uint8_t dpad_down : 1;
        uint8_t dpad_up : 1;
        uint8_t z : 1;
        uint8_t r : 1;
        uint8_t l : 1;
        uint8_t high1 : 1;

        // 3rd-8th data byte
        uint8_t stick_x;
        uint8_t stick_y;
        uint8_t cstick_x : 4;
        uint8_t cstick_y : 4;
        uint8_t l_analog;
        uint8_t r_analog;
        uint8_t analogA : 4;
        uint8_t analogB : 4;
    } mode1;

    struct {
        // first data byte (bitfields are sorted in LSB order)
        uint8_t a : 1;
        uint8_t b : 1;
        uint8_t x : 1;
        uint8_t y : 1;
        uint8_t start : 1;
        uint8_t origin : 1; // Indicates if GetOrigin(0x41) was called (LOW)
        uint8_t errlatch : 1;
        uint8_t errstat : 1;

        // second data byte
        uint8_t dpad_left : 1;
        uint8_t dpad_right : 1;
        uint8_t dpad_down : 1;
        uint8_t dpad_up : 1;
        uint8_t z : 1;
        uint8_t r : 1;
        uint8_t l : 1;
        uint8_t high1 : 1;

        // 3rd-8th data byte
        uint8_t stick_x;
        uint8_t stick_y;
        uint8_t cstick_x : 4;
        uint8_t cstick_y : 4;
        uint8_t l_analog : 4;
        uint8_t r_analog : 4;
        uint8_t analogA;
        uint8_t analogB;
    } mode2;

    struct {
        // first data byte (bitfields are sorted in LSB order)
        uint8_t a : 1;
        uint8_t b : 1;
        uint8_t x : 1;
        uint8_t y : 1;
        uint8_t start : 1;
        uint8_t origin : 1; // Indicates if GetOrigin(0x41) was called (LOW)
        uint8_t errlatch : 1;
        uint8_t errstat : 1;

        // second data byte
        uint8_t dpad_left : 1;
        uint8_t dpad_right : 1;
        uint8_t dpad_down : 1;
        uint8_t dpad_up : 1;
        uint8_t z : 1;
        uint8_t r : 1;
        uint8_t l : 1;
        uint8_t high1 : 1;

        // 3rd-8th data byte
        uint8_t stick_x;
        uint8_t stick_y;
        uint8_t cstick_x;
        uint8_t cstick_y;
        uint8_t analogA;
        uint8_t analogB;
    } mode4;

    struct {
        // first data byte (bitfields are sorted in LSB order)
        uint8_t salts : 4;
        uint8_t unknown1 : 2;
        uint8_t errlatch : 1;
        uint8_t errstat : 1;

        uint16_t unknown2 : 16;
        uint8_t  unknown3;

        uint8_t keypress[3];

        uint8_t checksum;
    } keyboard;

} gc_report_t;

typedef struct __attribute__((packed)) {
    gc_report_t initial_inputs;
    uint8_t reserved0;
    uint8_t reserved1;
} gc_origin_t;

typedef struct __attribute__((packed)) {
    uint16_t device;
    uint8_t status;
} gc_status_t;

#define DEFAULT_GC_REPORT_INITIALIZER {    \
    .a = 0,                                \
    .b = 0,                                \
    .x = 0,                                \
    .y = 0,                                \
    .start = 0,                            \
    .origin = 0,                           \
    .errlatch = 0,                         \
    .errstat = 0,                          \
    .dpad_left = 0,                        \
    .dpad_right = 0,                       \
    .dpad_down = 0,                        \
    .dpad_up = 0,                          \
    .z = 0,                                \
    .r = 0,                                \
    .l = 0,                                \
    .high1 = 1,                            \
    .stick_x = 128,                        \
    .stick_y = 128,                        \
    .cstick_x = 128,                       \
    .cstick_y = 128,                       \
    .l_analog = 0,                         \
    .r_analog = 0                          \
}

extern gc_report_t default_gc_report;

#define DEFAULT_GC_KB_REPORT_INITIALIZER {  \
    .keyboard.keypress = 0,                 \
}
extern gc_report_t default_gc_kb_report;

#define DEFAULT_GC_ORIGIN_INITIALIZER {    \
    .initial_inputs = DEFAULT_GC_REPORT_INITIALIZER, \
    .reserved0 = 0,                        \
    .reserved1 = 0                         \
}

extern gc_origin_t default_gc_origin;

// #define DEFAULT_GC_STATUS_INITIALIZER {    \
//     .device = 0x0009,                      \ // 0x0900
//     .status = 0x03                         \
// }

#define DEFAULT_GC_STATUS_INITIALIZER {    \
    .device = 0x2008,                      \
    .status = 0x03                         \
}

extern gc_status_t default_gc_status;

#endif
