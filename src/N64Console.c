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
volatile uint32_t n64_diag_pak_write_count = 0;
volatile uint16_t n64_diag_last_read_addr = 0;
volatile uint16_t n64_diag_last_write_addr = 0;
volatile uint8_t n64_diag_write_crc = 0;        // CRC we responded with for last WRITE
volatile uint8_t n64_diag_write_data[4] = {0};   // First 4 data bytes of last WRITE
volatile uint16_t n64_diag_first_write_addr = 0xFFFF;  // First write address seen
volatile uint8_t n64_diag_first_write_crc = 0;         // CRC for first write
volatile uint8_t n64_diag_first_write_data[4] = {0};   // First 4 data bytes of first write
volatile uint16_t n64_diag_first_read_addr = 0xFFFF;   // First read address seen
volatile uint8_t n64_diag_last_rx = 0xFF;   // Last received byte (always stored)
volatile uint8_t n64_diag_phase = 0;        // 0=waiting, 1=got byte, 2=sending, 3=sent
volatile uint32_t n64_diag_listen_time_us = 0;  // μs from boot when Core 1 starts listening
volatile uint32_t n64_diag_first_rx_us = 0;     // μs from boot when first byte received

// Persistent counters (never reset — survives probe resets)
volatile uint32_t n64_diag_total_reads = 0;
volatile uint32_t n64_diag_total_writes = 0;
volatile uint32_t n64_diag_total_probes = 0;
volatile uint32_t n64_diag_total_unknown = 0;
volatile uint32_t n64_diag_read_addr_fail = 0;  // Failed to receive address bytes on READ
volatile uint32_t n64_diag_write_buf_fail = 0;  // Failed to receive data on WRITE

// Boot sequence capture: first 20 command bytes received (resets on probe)
#define N64_BOOT_CAPTURE_SIZE 20
volatile uint8_t n64_boot_cmds[N64_BOOT_CAPTURE_SIZE];

// Full command history: never resets, captures all commands
#define N64_CMD_HISTORY_SIZE 40
volatile uint8_t n64_cmd_history[N64_CMD_HISTORY_SIZE];
volatile uint8_t n64_cmd_history_count = 0;
volatile uint8_t n64_boot_cmd_count = 0;

// Default instances
n64_report_t default_n64_report = DEFAULT_N64_REPORT_INITIALIZER;
extern n64_report_t n64_report;  // Live report updated by Core 1's update_output()
n64_status_t default_n64_status = DEFAULT_N64_STATUS_INITIALIZER;

// N64 controller pak data CRC (polynomial 0x85).
// Unlike standard CRC-8, N64 shifts data bits INTO the CRC register (not XOR),
// then finalizes with 8 zero-bit shifts. This is the algorithm used by the PIF
// to verify pak READ/WRITE responses.
// Placed in RAM for Core 1 safety (flash may be locked by CYW43 on RP2350).
static uint8_t __no_inline_not_in_flash_func(pak_calc_crc)(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        for (int j = 7; j >= 0; j--) {
            uint8_t xor_tap = (crc & 0x80) ? 0x85 : 0x00;
            crc <<= 1;
            if (data[i] & (1 << j)) crc |= 1;
            crc ^= xor_tap;
        }
    }
    // Finalization: process 8 zero bits
    for (int j = 7; j >= 0; j--) {
        uint8_t xor_tap = (crc & 0x80) ? 0x85 : 0x00;
        crc <<= 1;
        crc ^= xor_tap;
    }
    return crc;
}

// Pak type identification at address 0x8000:
//   0x80-filled = Rumble Pak
//   0x00-filled = Controller Pak (memory)
//   0x84-filled = Transfer Pak
#define PAK_ID_ADDR  0x8000

// Pre-built pak read responses (32 data + 1 CRC)
static uint8_t pak_response_rumble_id[N64_PAK_DATA_SIZE + 1];  // 0x80-filled + CRC (rumble pak ID)
static uint8_t pak_response_zeros[N64_PAK_DATA_SIZE + 1];      // 0x00-filled + CRC (empty data)

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

// Exact RAM replica of pio_sm_init + joybus_send_bytes.
// All functions called here are static inline (register writes only).
// Matches pio_sm_init step-for-step: disable, config, clear, fdebug, restart,
// clkdiv_restart, jmp, addr sync. Then enable and send like joybus_send_bytes.
static void __no_inline_not_in_flash_func(n64_send_bytes)(
    joybus_port_t *port, uint8_t *bytes, uint len
) {
    while (!gpio_get(port->pin)) { tight_loop_contents(); }

    // --- Exact pio_sm_init equivalent (all inline, flash-safe) ---
    pio_sm_set_enabled(port->pio, port->sm, false);
    pio_sm_set_config(port->pio, port->sm, &port->config);
    pio_sm_clear_fifos(port->pio, port->sm);
    // Clear FDEBUG sticky flags (same as pio_sm_init)
    const uint32_t fdebug_sm_mask =
        (1u << PIO_FDEBUG_TXOVER_LSB) |
        (1u << PIO_FDEBUG_RXUNDER_LSB) |
        (1u << PIO_FDEBUG_TXSTALL_LSB) |
        (1u << PIO_FDEBUG_RXSTALL_LSB);
    port->pio->fdebug = fdebug_sm_mask << port->sm;
    pio_sm_restart(port->pio, port->sm);
    pio_sm_clkdiv_restart(port->pio, port->sm);
    pio_sm_exec(port->pio, port->sm,
                pio_encode_jmp(port->offset + joybus_offset_write));
    (void)port->pio->sm[port->sm].addr;  // sync: ensure JMP takes effect

    // --- Now send (same as joybus_send_bytes after pio_sm_init) ---
    pio_sm_set_enabled(port->pio, port->sm, true);
    for (uint i = 0; i < len; i++) {
        joybus_send_byte(port, bytes[i], i == len - 1);
    }

    // Wait for PIO to finish transmitting before returning.
    // joybus_send_byte only puts data in the TX FIFO; the PIO sends asynchronously.
    // If we return early, the caller's gpio_get() check can see the PIO driving
    // the line LOW (during a joybus bit), misinterpret it as "N64 off", and reset
    // the SM mid-send — destroying the response.
    while (!pio_sm_is_tx_fifo_empty(port->pio, port->sm)) {
        tight_loop_contents();
    }
    // TX FIFO is empty but PIO is still sending the last byte + stop bit.
    // Wait for PIO to transition back to read mode (set pindirs 0 = input).
    while (gpio_is_dir_out(port->pin)) {
        tight_loop_contents();
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
    memset(pak_response_rumble_id, 0x80, N64_PAK_DATA_SIZE);
    pak_response_rumble_id[N64_PAK_DATA_SIZE] = pak_calc_crc(pak_response_rumble_id, N64_PAK_DATA_SIZE);

    // Pre-build zero response: 0x00-filled + CRC
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
            case N64Command_RESET: {
                n64_status_t reset_status = default_n64_status;
                reset_status.status |= N64_STATUS_PAK_CHANGED;
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, (uint8_t *)&reset_status, sizeof(n64_status_t));
                attempts = 0;
                break;
            }
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
                uint8_t *det_response = (real_addr == PAK_ID_ADDR)
                    ? pak_response_rumble_id
                    : pak_response_zeros;
                busy_wait_us(n64_reply_delay);
                joybus_send_bytes(&console->_port, det_response, N64_PAK_DATA_SIZE + 1);
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

    // Record when we start listening (μs since boot)
    if (n64_diag_listen_time_us == 0) {
        n64_diag_listen_time_us = timer_hw->timelr;
    }

    while (true) {
        n64_diag_phase = 0;

        // Receive with timeout so we can detect N64 power-off/on.
        // Without timeout, pio_sm_get_blocking blocks forever when N64 is off,
        // and we never reach the line-state check to reset phantom ISR bits.
        // 50ms is long enough to avoid unnecessary resets during normal 60Hz polling
        // (~16.7ms between polls) but short enough to detect power cycles promptly.
        uint bytes = joybus_receive_bytes(&console->_port, received, 1, 50000, true);

        if (bytes == 0) {
            // Timeout — no command received. Check if N64 is off (line low).
            // When N64 is off, the joybus line drops low (no pull-up power).
            // PIO in read mode sees this as data, injecting phantom bits into ISR.
            if (!gpio_get(console->_port.pin)) {
                // N64 is off — wait for power-on
                while (!gpio_get(console->_port.pin)) {
                    tight_loop_contents();
                }
                // Line just went high — power is ramping up.
                // Wait briefly for line to stabilize.
                n64_delay_us(100);
            }
            // Reset SM to clear any phantom ISR bits from power-off/bounce
            n64_reset_to_receive(&console->_port);
            continue;
        }
        n64_diag_phase = 1;

        // Capture non-POLL command history (never resets)
        if (received[0] != 0x01 && n64_cmd_history_count < N64_CMD_HISTORY_SIZE) {
            n64_cmd_history[n64_cmd_history_count++] = received[0];
        }

        // Record first receive timestamp
        if (n64_diag_first_rx_us == 0) {
            n64_diag_first_rx_us = timer_hw->timelr;
        }

        // On RESET or PROBE, clear boot capture for fresh cold-boot trace
        if ((N64Command)received[0] == N64Command_RESET ||
            (N64Command)received[0] == N64Command_PROBE) {
            n64_boot_cmd_count = 0;
            n64_diag_probe_count = 0;
            n64_diag_poll_count = 0;
            n64_diag_pak_read_count = 0;
            n64_diag_pak_write_count = 0;
            n64_diag_rx_count = 0;
            n64_diag_first_write_addr = 0xFFFF;
            n64_diag_first_read_addr = 0xFFFF;
        }

        // Capture boot sequence
        if (n64_boot_cmd_count < N64_BOOT_CAPTURE_SIZE) {
            n64_boot_cmds[n64_boot_cmd_count++] = received[0];
        }

        // Deferred pak advertisement: report no pak during initial boot probes
        // to avoid games that can't handle our pak emulation (e.g., Cruisin' USA).
        // After first POLL, switch to pak present so games can detect rumble pak.
        // RESET restarts the deferred sequence (pak hidden again until next POLL).
        static bool pak_advertised = false;

        // Respond with reply delay matching real controller timing (~4μs after stop bit).
        // n64_send_bytes waits for line-high first, then n64_delay_us adds the gap.
        switch ((N64Command)received[0]) {
            case N64Command_RESET: {
                n64_status_t reset_status = default_n64_status;
                // Hide pak during boot — game probes after RESET
                reset_status.status = pak_advertised ? (default_n64_status.status | N64_STATUS_PAK_CHANGED) : 0x00;
                pak_advertised = false;
                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, (uint8_t *)&reset_status, sizeof(n64_status_t));
                n64_diag_probe_count++;
                n64_diag_total_probes++;
                break;
            }
            case N64Command_PROBE: {
                n64_status_t probe_status = default_n64_status;
                if (!pak_advertised) probe_status.status = 0x00;
                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, (uint8_t *)&probe_status, sizeof(n64_status_t));
                n64_diag_probe_count++;
                n64_diag_total_probes++;
                break;
            }

            case N64Command_POLL: {
                // Set console active before sending — POLL may be the first
                // command seen (e.g., Everdrive skips PROBE after game boot)
                extern volatile bool n64_console_active;
                if (!n64_console_active) n64_console_active = true;
                // After first poll, advertise pak on subsequent probes
                pak_advertised = true;

                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, (uint8_t *)&n64_report, sizeof(n64_report_t));
                n64_diag_poll_count++;
                return false;
            }

            case N64Command_READ_EXPANSION_BUS: {
                uint8_t addr_bytes[2];
                uint got_addr = joybus_receive_bytes(&console->_port, addr_bytes, 2, n64_receive_timeout_us, true);
                if (got_addr < 2) { n64_diag_read_addr_fail++; break; }
                uint16_t addr = ((uint16_t)addr_bytes[0] << 8) | addr_bytes[1];
                uint16_t real_addr = addr & 0xFFE0;

                // Rumble pak returns 0x80 for all addresses except 0xC000 (rumble control)
                uint8_t *response = (real_addr == N64_RUMBLE_PAK_ADDR)
                    ? pak_response_zeros
                    : pak_response_rumble_id;

                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, response, N64_PAK_DATA_SIZE + 1);
                n64_diag_pak_read_count++;
                n64_diag_total_reads++;
                if (n64_diag_first_read_addr == 0xFFFF) n64_diag_first_read_addr = real_addr;
                n64_diag_last_read_addr = real_addr;
                break;
            }

            case N64Command_WRITE_EXPANSION_BUS: {
                uint8_t buf[2 + N64_PAK_DATA_SIZE];
                uint got_wr = joybus_receive_bytes(&console->_port, buf, sizeof(buf), n64_receive_timeout_us, true);
                if (got_wr < sizeof(buf)) { n64_diag_write_buf_fail++; break; }

                uint8_t crc = pak_calc_crc(&buf[2], N64_PAK_DATA_SIZE);

                n64_delay_us(n64_reply_delay);
                n64_send_bytes(&console->_port, &crc, 1);

                uint16_t addr = ((uint16_t)buf[0] << 8) | buf[1];
                uint16_t write_addr = addr & 0xFFE0;
                n64_diag_last_write_addr = write_addr;
                n64_diag_write_crc = crc;
                n64_diag_write_data[0] = buf[2];
                n64_diag_write_data[1] = buf[3];
                n64_diag_write_data[2] = buf[4];
                n64_diag_write_data[3] = buf[5];
                if (n64_diag_first_write_addr == 0xFFFF) {
                    n64_diag_first_write_addr = write_addr;
                    n64_diag_first_write_crc = crc;
                    n64_diag_first_write_data[0] = buf[2];
                    n64_diag_first_write_data[1] = buf[3];
                    n64_diag_first_write_data[2] = buf[4];
                    n64_diag_first_write_data[3] = buf[5];
                }
                n64_diag_pak_write_count++;
                n64_diag_total_writes++;

                if (write_addr == N64_RUMBLE_PAK_ADDR) {
                    uint8_t rumble = 0;
                    for (int i = 2; i < 2 + N64_PAK_DATA_SIZE; i++) {
                        if (buf[i]) { rumble = 255; break; }
                    }
                    n64_rumble_state = rumble;
                }
                break;
            }

            case 0x1D: {
                // PixelFX N64 Game ID: 1 command + 10 data bytes, no response.
                // Sent by game software for N64Digital/Retro GEM per-game settings.
                // Fire-and-forget — absorb payload and return to receive.
                uint8_t game_id[10];
                joybus_receive_bytes(&console->_port, game_id, 10, n64_receive_timeout_us, true);
                break;
            }

            default:
                n64_diag_last_cmd = received[0];
                n64_diag_total_unknown++;
                // Unknown commands: wait for trailing data then reset SM
                n64_delay_us(n64_reset_wait_period_us);
                n64_reset_to_receive(&console->_port);
                break;
        }

        // Update diagnostics AFTER response (not timing-critical)
        n64_diag_rx_count++;
        n64_diag_last_rx = received[0];

        // Signal console detection on first valid receive
        extern volatile bool n64_console_active;
        if (!n64_console_active) {
            n64_console_active = true;
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
