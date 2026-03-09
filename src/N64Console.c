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
#include <string.h>
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

// Rumble state (written by Core 1, read by Core 0 via OutputInterface)
volatile uint8_t n64_rumble_state = 0;

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

// CRC-8 lookup table for N64 pak operations (polynomial 0x85)
// Placed in RAM (__not_in_flash) because Core 1 accesses it from WaitForPoll,
// and Core 0 may lock flash during BT bonding writes (flash_safe_execute).
static const uint8_t __not_in_flash("n64_pak") pak_crc_table[256] = {
    0x00, 0x85, 0x8F, 0x0A, 0x9B, 0x1E, 0x14, 0x91,
    0xB3, 0x36, 0x3C, 0xB9, 0x28, 0xAD, 0xA7, 0x22,
    0xE3, 0x66, 0x6C, 0xE9, 0x78, 0xFD, 0xF7, 0x72,
    0x50, 0xD5, 0xDF, 0x5A, 0xCB, 0x4E, 0x44, 0xC1,
    0x43, 0xC6, 0xCC, 0x49, 0xD8, 0x5D, 0x57, 0xD2,
    0xF0, 0x75, 0x7F, 0xFA, 0x6B, 0xEE, 0xE4, 0x61,
    0xA0, 0x25, 0x2F, 0xAA, 0x3B, 0xBE, 0xB4, 0x31,
    0x13, 0x96, 0x9C, 0x19, 0x88, 0x0D, 0x07, 0x82,
    0x86, 0x03, 0x09, 0x8C, 0x1D, 0x98, 0x92, 0x17,
    0x35, 0xB0, 0xBA, 0x3F, 0xAE, 0x2B, 0x21, 0xA4,
    0x65, 0xE0, 0xEA, 0x6F, 0xFE, 0x7B, 0x71, 0xF4,
    0xD6, 0x53, 0x59, 0xDC, 0x4D, 0xC8, 0xC2, 0x47,
    0xC5, 0x40, 0x4A, 0xCF, 0x5E, 0xDB, 0xD1, 0x54,
    0x76, 0xF3, 0xF9, 0x7C, 0xED, 0x68, 0x62, 0xE7,
    0x26, 0xA3, 0xA9, 0x2C, 0xBD, 0x38, 0x32, 0xB7,
    0x95, 0x10, 0x1A, 0x9F, 0x0E, 0x8B, 0x81, 0x04,
    0x89, 0x0C, 0x06, 0x83, 0x12, 0x97, 0x9D, 0x18,
    0x3A, 0xBF, 0xB5, 0x30, 0xA1, 0x24, 0x2E, 0xAB,
    0x6A, 0xEF, 0xE5, 0x60, 0xF1, 0x74, 0x7E, 0xFB,
    0xD9, 0x5C, 0x56, 0xD3, 0x42, 0xC7, 0xCD, 0x48,
    0xCA, 0x4F, 0x45, 0xC0, 0x51, 0xD4, 0xDE, 0x5B,
    0x79, 0xFC, 0xF6, 0x73, 0xE2, 0x67, 0x6D, 0xE8,
    0x29, 0xAC, 0xA6, 0x23, 0xB2, 0x37, 0x3D, 0xB8,
    0x9A, 0x1F, 0x15, 0x90, 0x01, 0x84, 0x8E, 0x0B,
    0x0F, 0x8A, 0x80, 0x05, 0x94, 0x11, 0x1B, 0x9E,
    0xBC, 0x39, 0x33, 0xB6, 0x27, 0xA2, 0xA8, 0x2D,
    0xEC, 0x69, 0x63, 0xE6, 0x77, 0xF2, 0xF8, 0x7D,
    0x5F, 0xDA, 0xD0, 0x55, 0xC4, 0x41, 0x4B, 0xCE,
    0x4C, 0xC9, 0xC3, 0x46, 0xD7, 0x52, 0x58, 0xDD,
    0xFF, 0x7A, 0x70, 0xF5, 0x64, 0xE1, 0xEB, 0x6E,
    0xAF, 0x2A, 0x20, 0xA5, 0x34, 0xB1, 0xBB, 0x3E,
    0x1C, 0x99, 0x93, 0x16, 0x87, 0x02, 0x08, 0x8D,
};

static uint8_t __no_inline_not_in_flash_func(pak_calc_crc)(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = pak_crc_table[crc ^ data[i]];
    }
    return crc;
}

// Rumble pak identification: reads to 0x8000 return 0x80-filled data
// This is how the N64 console identifies a rumble pak vs controller pak
#define RUMBLE_PAK_ID_ADDR  0x8000

// Pre-built pak read responses (32 data + 1 CRC)
static uint8_t pak_response_rumble_id[N64_PAK_DATA_SIZE + 1];  // 0x80-filled + CRC (rumble pak ID)
static uint8_t pak_response_zeros[N64_PAK_DATA_SIZE + 1];      // 0x00-filled + CRC (rumble motor off)

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

    // Pre-build rumble pak ID response: 0x80-filled + CRC
    // N64 console reads address 0x8000 to identify pak type.
    // Rumble pak returns 0x80 for all 32 bytes.
    memset(pak_response_rumble_id, 0x80, N64_PAK_DATA_SIZE);
    pak_response_rumble_id[N64_PAK_DATA_SIZE] = pak_calc_crc(pak_response_rumble_id, N64_PAK_DATA_SIZE);

    // Pre-build zero response: 0x00-filled + CRC
    // Used for rumble motor status reads (0xC000) and other addresses.
    memset(pak_response_zeros, 0x00, N64_PAK_DATA_SIZE);
    pak_response_zeros[N64_PAK_DATA_SIZE] = pak_calc_crc(pak_response_zeros, N64_PAK_DATA_SIZE);
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
                uint8_t addr_bytes[2];
                joybus_receive_bytes(&console->_port, addr_bytes, 2, n64_receive_timeout_us, true);
                uint16_t read_addr = ((uint16_t)addr_bytes[0] << 8) | addr_bytes[1];
                uint16_t real_addr = read_addr & 0xFFE0;
                uint8_t *response = (real_addr == RUMBLE_PAK_ID_ADDR)
                    ? pak_response_rumble_id
                    : pak_response_zeros;
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, response, N64_PAK_DATA_SIZE + 1);
                attempts = 0;
                break;
            }
            case N64Command_WRITE_EXPANSION_BUS: {
                uint8_t buf[2 + N64_PAK_DATA_SIZE];
                joybus_receive_bytes(&console->_port, buf, sizeof(buf), n64_receive_timeout_us, true);
                uint8_t crc = pak_calc_crc(&buf[2], N64_PAK_DATA_SIZE);
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, &crc, 1);
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
//
// Also serves as initial detection: sets n64_console_active on first received
// command. This eliminates the need for a separate Detect phase that uses
// flash-resident functions and can miss probes during BT init.
bool __no_inline_not_in_flash_func(N64Console_WaitForPoll)(N64Console_t* console) {
    uint8_t received[1];

    while (true) {
        n64_diag_phase = 0;
        joybus_receive_bytes(&console->_port, received, 1, n64_receive_timeout_us, false);
        n64_diag_rx_count++;
        n64_diag_last_rx = received[0];
        n64_diag_phase = 1;

        // Signal console detection on first valid receive
        extern volatile bool n64_console_active;
        if (!n64_console_active) {
            n64_console_active = true;
        }

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
                uint8_t addr_bytes[2];
                joybus_receive_bytes(&console->_port, addr_bytes, 2, n64_receive_timeout_us, true);
                uint16_t addr = ((uint16_t)addr_bytes[0] << 8) | addr_bytes[1];
                uint16_t real_addr = addr & 0xFFE0;  // mask off 5-bit address CRC

                // Select response based on address:
                // 0x8000: pak identification — return 0x80-filled (rumble pak signature)
                // 0xC000: rumble motor status — return 0x00-filled
                // Other: return 0x00-filled
                uint8_t *response = (real_addr == RUMBLE_PAK_ID_ADDR)
                    ? pak_response_rumble_id
                    : pak_response_zeros;

                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, response, N64_PAK_DATA_SIZE + 1);
                n64_diag_pak_read_count++;
                break;
            }

            case N64Command_WRITE_EXPANSION_BUS: {
                uint8_t buf[2 + N64_PAK_DATA_SIZE];
                joybus_receive_bytes(&console->_port, buf, sizeof(buf), n64_receive_timeout_us, true);

                // Calculate CRC of the 32 data bytes written by console
                uint8_t crc = pak_calc_crc(&buf[2], N64_PAK_DATA_SIZE);

                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, &crc, 1);

                // Check for rumble pak write (address 0xC000)
                uint16_t addr = ((uint16_t)buf[0] << 8) | buf[1];
                if ((addr & 0xFFE0) == N64_RUMBLE_PAK_ADDR) {
                    uint8_t rumble = 0;
                    for (int i = 2; i < 2 + N64_PAK_DATA_SIZE; i++) {
                        if (buf[i]) { rumble = 255; break; }
                    }
                    n64_rumble_state = rumble;
                }
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
