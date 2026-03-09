// N64Console.c - C API for N64 Console (Device Mode)
//
// Emulates an N64 controller connected to an N64 console.
// C implementation following the pattern from N64Console.cpp and GamecubeConsole.c

#include "N64Console.h"
#include "n64_definitions.h"
#include "joybus.h"
#include "joybus.pio.h"

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

// Diagnostic counters (read by Core 0 for serial output)
volatile uint32_t n64_diag_poll_count = 0;
volatile uint8_t n64_diag_last_cmd = 0;
volatile uint32_t n64_diag_probe_count = 0;
volatile uint32_t n64_diag_rx_count = 0;
volatile uint32_t n64_diag_pak_read_count = 0;
volatile uint8_t n64_diag_last_rx = 0xFF;   // Last received byte (always stored)
volatile uint8_t n64_diag_phase = 0;        // 0=waiting, 1=got byte, 2=sending, 3=sent

// Default instances
n64_report_t default_n64_report = DEFAULT_N64_REPORT_INITIALIZER;
extern n64_report_t n64_report;  // Live report updated by Core 1's update_output()
n64_status_t default_n64_status = DEFAULT_N64_STATUS_INITIALIZER;

// Zero-filled response for pak reads (32 data bytes + 1 CRC byte)
static uint8_t pak_read_response[N64_PAK_DATA_SIZE + 1];

// ============================================================================
// RAM-only helpers for Core 1 (RP2350: flash may be locked by Core 0's CYW43)
// ============================================================================

// Accurate delay without calling busy_wait_us (which is in flash).
// make_timeout_time_us and time_reached are static inline, so they compile
// into this function's RAM section.
static void __no_inline_not_in_flash_func(n64_delay_us)(uint32_t us) {
    absolute_time_t target = make_timeout_time_us(us);
    while (!time_reached(target)) {
        tight_loop_contents();
    }
}

// Send bytes without calling pio_sm_init (which is in flash).
// All PIO functions used here are static inline (register writes).
static void __no_inline_not_in_flash_func(n64_send_bytes)(
    joybus_port_t *port, uint8_t *bytes, uint len
) {
    while (!gpio_get(port->pin)) { tight_loop_contents(); }
    pio_sm_set_enabled(port->pio, port->sm, false);
    pio_sm_clear_fifos(port->pio, port->sm);
    pio_sm_restart(port->pio, port->sm);
    pio_sm_exec(port->pio, port->sm,
                pio_encode_jmp(port->offset + joybus_offset_write));
    pio_sm_set_enabled(port->pio, port->sm, true);
    for (uint i = 0; i < len; i++) {
        joybus_send_byte(port, bytes[i], i == len - 1);
    }
}

// Reset SM to receive mode without calling pio_sm_init.
static void __no_inline_not_in_flash_func(n64_reset_to_receive)(joybus_port_t *port) {
    pio_sm_set_enabled(port->pio, port->sm, false);
    pio_sm_clear_fifos(port->pio, port->sm);
    pio_sm_restart(port->pio, port->sm);
    pio_sm_exec(port->pio, port->sm,
                pio_encode_jmp(port->offset + joybus_offset_read));
    // Left disabled — joybus_receive_bytes will enable it
}

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
            case N64Command_READ_EXPANSION_BUS: {
                // READ: 0x02 + 2 addr bytes → respond with 32 data + 1 CRC
                uint8_t addr[2];
                joybus_receive_bytes(&console->_port, addr, 2, n64_receive_timeout_us, true);
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, pak_read_response, sizeof(pak_read_response));
                attempts = 0;
                break;
            }
            case N64Command_WRITE_EXPANSION_BUS: {
                // WRITE: 0x03 + 2 addr + 32 data → respond with 1 CRC
                uint8_t buf[2 + N64_PAK_DATA_SIZE];
                joybus_receive_bytes(&console->_port, buf, sizeof(buf), n64_receive_timeout_us, true);
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, pak_read_response, 1); // CRC = 0
                attempts = 0;
                break;
            }
            default:
                // Unknown command — wait for it to finish, then reset
                busy_wait_us(n64_reset_wait_period_us);
                joybus_port_reset(&console->_port);
        }
    }

    return false;
}

// Wait for N64 console to poll, handle probe/reset automatically.
// Follows the same pattern as GamecubeConsole: no SM cleanup between
// sends and receives — PIO naturally transitions write→read via jmp.
//
// IMPORTANT: This function runs on Core 1 and must be entirely flash-free.
// On RP2350 (Pico 2 W), Core 0's CYW43 driver periodically locks flash for
// BT bonding storage, making flash-resident functions (busy_wait_us, pio_sm_init)
// inaccessible to Core 1. All operations use RAM-only helpers above.
bool __no_inline_not_in_flash_func(N64Console_WaitForPoll)(N64Console_t* console) {
    uint8_t received[1];

    while (true) {
        n64_diag_phase = 0;
        joybus_receive_bytes(&console->_port, received, 1, n64_receive_timeout_us, false);
        n64_diag_rx_count++;
        n64_diag_last_rx = received[0];
        n64_diag_phase = 1;

        switch ((N64Command)received[0]) {
            case N64Command_RESET:
            case N64Command_PROBE:
                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, (uint8_t *)&default_n64_status, sizeof(n64_status_t));
                n64_diag_probe_count++;
                break;

            case N64Command_POLL:
                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, (uint8_t *)&n64_report, sizeof(n64_report_t));
                n64_diag_poll_count++;
                return false;

            case N64Command_READ_EXPANSION_BUS: {
                uint8_t addr[2];
                joybus_receive_bytes(&console->_port, addr, 2, n64_receive_timeout_us, true);
                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, pak_read_response, sizeof(pak_read_response));
                n64_diag_pak_read_count++;
                break;
            }

            case N64Command_WRITE_EXPANSION_BUS: {
                uint8_t buf[2 + N64_PAK_DATA_SIZE];
                joybus_receive_bytes(&console->_port, buf, sizeof(buf), n64_receive_timeout_us, true);
                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, pak_read_response, 1);
                break;
            }

            default:
                n64_diag_last_cmd = received[0];
                n64_delay_us(n64_reset_wait_period_us);
                n64_reset_to_receive(&console->_port);
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
