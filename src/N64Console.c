// N64Console.c - C API for N64 Console (Device Mode)
//
// Emulates an N64 controller connected to an N64 console.
// C implementation following the pattern from N64Console.cpp and GamecubeConsole.c

#include "N64Console.h"
#include "n64_definitions.h"
#include "joybus.h"

#include <hardware/pio.h>
#include <hardware/timer.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Disable debug printf to avoid timing issues with joybus protocol
#define printf(...)

// N64 joybus timing constants
#define n64_incoming_bit_length_us   5
#define n64_max_command_bytes        1
#define n64_receive_timeout_us       (n64_incoming_bit_length_us * 10)
#define n64_reset_wait_period_us     ((n64_incoming_bit_length_us * 8) * (n64_max_command_bytes - 1) + n64_receive_timeout_us)
#define n64_reply_delay              (n64_incoming_bit_length_us - 1)

// Default instances
n64_report_t default_n64_report = DEFAULT_N64_REPORT_INITIALIZER;
n64_status_t default_n64_status = DEFAULT_N64_STATUS_INITIALIZER;

// Initialization
void N64Console_init(N64Console_t* console, uint pin, PIO pio, int sm, int offset) {
    joybus_port_init(&console->_port, pin, pio, sm, offset);
}

// Termination
void N64Console_terminate(N64Console_t* console) {
    joybus_port_terminate(&console->_port);
}

// Detect if an N64 console is connected
bool __no_inline_not_in_flash_func(N64Console_Detect)(N64Console_t* console) {
    uint8_t received[1];

    // Attempt to receive and respond to PROBE/RESET commands
    // N64 polls at 60Hz, so 60 attempts ≈ 1 second
    for (uint8_t attempts = 0; attempts < 60; attempts++) {
        if (joybus_receive_bytes(&console->_port, received, 1, 20000, true) != 1) {
            continue;
        }

        switch ((N64Command)received[0]) {
            case N64Command_RESET:
            case N64Command_PROBE:
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, (uint8_t *)&default_n64_status, sizeof(n64_status_t));
                attempts = 0;
                break;
            case N64Command_POLL:
                // Console sent a poll — we're detected
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, (uint8_t *)&default_n64_report, sizeof(n64_report_t));
                // Wait for report to finish sending
                busy_wait_us(40 * sizeof(n64_report_t));
                return true;
            default:
                // Invalid command — wait and reset
                busy_wait_us(n64_reset_wait_period_us);
                joybus_port_reset(&console->_port);
        }
    }

    return false;
}

// Wait for N64 console to poll, handle probe/reset automatically
// Returns true if rumble is active (from expansion bus write)
bool __no_inline_not_in_flash_func(N64Console_WaitForPoll)(N64Console_t* console) {
    uint8_t received[1];

    while (true) {
        joybus_receive_bytes(&console->_port, received, 1, n64_receive_timeout_us, false);

        switch ((N64Command)received[0]) {
            case N64Command_RESET:
            case N64Command_PROBE:
                // Respond with controller status
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, (uint8_t *)&default_n64_status, sizeof(n64_status_t));
                break;

            case N64Command_POLL:
                // Set timeout for reply delay
                console->_receive_end = make_timeout_time_us(n64_reply_delay);
                return false;  // No rumble info from basic poll

            case N64Command_READ_EXPANSION_BUS: {
                // Console wants to read expansion bus (controller pak / rumble pak)
                // Read 2 address bytes
                uint8_t addr_bytes[2];
                joybus_receive_bytes(&console->_port, addr_bytes, 2, n64_receive_timeout_us, true);

                // Respond with 32 bytes of data + 1 CRC byte
                uint8_t response[33];
                memset(response, 0x00, sizeof(response));

                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, response, sizeof(response));
                break;
            }

            case N64Command_WRITE_EXPANSION_BUS: {
                // Console writing to expansion bus (rumble pak control)
                // Read 2 address bytes + 32 data bytes
                uint8_t write_data[34];
                joybus_receive_bytes(&console->_port, write_data, 34, n64_receive_timeout_us, true);

                // Respond with CRC byte
                uint8_t crc = 0x00;
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, &crc, 1);

                // Check if this is a rumble pak write (address 0xC000)
                uint16_t addr = ((uint16_t)write_data[0] << 8) | write_data[1];
                if ((addr & 0xFFE0) == N64_RUMBLE_PAK_ADDR) {
                    // Rumble is on if any data byte is non-zero
                    // Set timeout and return rumble state
                    console->_receive_end = make_timeout_time_us(n64_reply_delay);
                    // Don't return here — wait for next POLL
                }
                break;
            }

            default:
                // Unknown command — wait and reset
                printf("N64 COMMAND: 0x%x\n", received[0]);
                busy_wait_us(n64_reset_wait_period_us);
                joybus_port_reset(&console->_port);
        }
    }
}

// Send controller report to N64 console (call after WaitForPoll returns)
void __no_inline_not_in_flash_func(N64Console_SendReport)(N64Console_t* console, n64_report_t *report) {
    // Wait for receive timeout to end before responding
    while (!time_reached(console->_receive_end)) {
        tight_loop_contents();
    }

    joybus_send_bytes(&console->_port, (uint8_t *)report, sizeof(n64_report_t));
}

// Get PIO program offset
int N64Console_GetOffset(N64Console_t* console) {
    return console->_port.offset;
}
