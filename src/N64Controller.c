/**
 * N64Controller.c - N64 Controller Reader (C port)
 *
 * Reads Nintendo 64 controllers using the joybus protocol over PIO.
 * Ported from N64Controller.cpp for use in C projects.
 *
 * Based on joybus-pio library by:
 * https://github.com/JonnyHaystack/joybus-pio
 */

#include "N64Controller.h"
#include "n64_definitions.h"
#include "joybus.h"

#include <stdio.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <string.h>

// Timing constants
#define N64_INCOMING_BIT_LENGTH_US 4
// Give 5 bytes of leniency for controller processing time,
// plus another byte because timeout starts as we begin sending
#define N64_RECEIVE_TIMEOUT_US (N64_INCOMING_BIT_LENGTH_US * 10 * 6)

// Default status for initialization
static n64_status_t default_n64_status = DEFAULT_N64_STATUS_INITIALIZER;

// ============================================================================
// Internal Functions
// ============================================================================

static void __no_inline_not_in_flash_func(N64Controller_wait_poll_cooldown)(N64Controller* controller)
{
    // Wait for start of next polling period
    while (!time_reached(controller->_next_poll)) {
        tight_loop_contents();
    }

    // Reset poll cooldown
    controller->_next_poll = make_timeout_time_us(controller->_polling_period_us);
}

static bool __no_inline_not_in_flash_func(N64Controller_do_init)(N64Controller* controller)
{
    // Send probe command
    uint8_t probe_cmd[] = { N64Command_PROBE };
    joybus_send_bytes(&controller->_port, probe_cmd, sizeof(probe_cmd));

    // Read and validate probe response
    uint received_len = joybus_receive_bytes(
        &controller->_port,
        (uint8_t*)&controller->_status,
        sizeof(n64_status_t),
        N64_RECEIVE_TIMEOUT_US,
        true
    );

    // If response is invalid, return false
    if (received_len != sizeof(n64_status_t) || controller->_status.device == 0) {
        return false;
    }

    // Wait until start of next polling period before sending first poll
    N64Controller_wait_poll_cooldown(controller);

    // Send initial poll command (acts as "origin" calibration)
    uint8_t poll_cmd[] = { N64Command_POLL };
    joybus_send_bytes(&controller->_port, poll_cmd, sizeof(poll_cmd));

    // Read and validate poll response
    n64_report_t report;
    received_len = joybus_receive_bytes(
        &controller->_port,
        (uint8_t*)&report,
        sizeof(n64_report_t),
        N64_RECEIVE_TIMEOUT_US,
        true
    );

    // If response is invalid, return false
    if (received_len != sizeof(n64_report_t)) {
        return false;
    }

    return true;
}

// ============================================================================
// Public API
// ============================================================================

void N64Controller_init(N64Controller* controller, uint pin, uint polling_rate,
                        PIO pio, int sm, int offset)
{
    memset(controller, 0, sizeof(N64Controller));

    joybus_port_init(&controller->_port, pin, pio, sm, offset);
    controller->_polling_period_us = 1000000 / polling_rate;
    controller->_next_poll = get_absolute_time();
    controller->_initialized = false;

    // Initialize status with defaults
    controller->_status = default_n64_status;
}

void N64Controller_terminate(N64Controller* controller)
{
    joybus_port_terminate(&controller->_port);
}

bool __no_inline_not_in_flash_func(N64Controller_Poll)(N64Controller* controller,
                                                        n64_report_t* report, bool rumble)
{
    static uint8_t init_fail_count = 0;

    // If controller is uninitialized, do probe/origin sequence
    if (!controller->_initialized) {
        // When not initialized, use non-blocking check instead of blocking wait
        // This prevents starving other tasks (e.g., DC maple bus) when no N64 controller is connected
        if (!time_reached(controller->_next_poll)) {
            return false;  // Not time to retry yet, return immediately
        }

        // Short backoff between init retries (no controller connected)
        uint32_t backoff_ms = 500;  // 500ms between retries
        controller->_next_poll = make_timeout_time_ms(backoff_ms);

        controller->_initialized = N64Controller_do_init(controller);
        if (!controller->_initialized) {
            if (init_fail_count < 255) init_fail_count++;
            return false;
        }
        init_fail_count = 0;  // Reset on success
    }

    // Non-blocking cooldown check for regular polling too
    // This prevents starving other tasks (e.g., DC maple bus)
    if (!time_reached(controller->_next_poll)) {
        return false;  // Not time to poll yet, return immediately
    }
    controller->_next_poll = make_timeout_time_us(controller->_polling_period_us);

    // Send poll command with rumble (matches C++ version: 0x01, 0x03, rumble)
    uint8_t poll_cmd[] = { N64Command_POLL, 0x03, rumble };
    joybus_send_bytes(&controller->_port, poll_cmd, sizeof(poll_cmd));

    // Read and validate report
    uint8_t received_len = joybus_receive_bytes(
        &controller->_port,
        (uint8_t*)report,
        sizeof(n64_report_t),
        N64_RECEIVE_TIMEOUT_US,
        true
    );

    // If response is invalid, restart the initialization process
    if (received_len != sizeof(n64_report_t)) {
        controller->_initialized = false;
        return false;
    }

    return true;
}

int N64Controller_GetOffset(N64Controller* controller)
{
    return controller->_port.offset;
}

bool N64Controller_IsInitialized(N64Controller* controller)
{
    return controller->_initialized;
}

const n64_status_t* N64Controller_GetStatus(N64Controller* controller)
{
    return &controller->_status;
}

bool N64Controller_HasPak(N64Controller* controller)
{
    if (!controller->_initialized) return false;
    return (controller->_status.status & N64_STATUS_PAK_PRESENT) != 0;
}

// Calculate address CRC5 for controller pak commands
// CRC is calculated over address bits [15:5] (11 bits), polynomial 0x15
static uint8_t calc_address_crc(uint16_t addr)
{
    // Only use upper 11 bits (addr[15:5])
    uint16_t addr_bits = addr >> 5;
    uint8_t crc = 0;
    for (int i = 0; i < 11; i++) {
        uint8_t bit = (addr_bits >> (10 - i)) & 1;
        uint8_t xor_tap = ((crc >> 4) ^ bit) & 1;
        crc = ((crc << 1) | xor_tap) ^ (xor_tap ? 0x15 : 0);
        crc &= 0x1F;
    }
    return crc;
}

// Initialize rumble pak - must be called before SetRumble will work
bool N64Controller_InitRumblePak(N64Controller* controller)
{
    if (!controller->_initialized) return false;

    // Write 32 bytes of 0xFE to address 0x8000 for initialization
    // Address 0x8000 encodes to: 0x80, 0x01
    uint8_t cmd[35];
    cmd[0] = N64Command_WRITE_EXPANSION_BUS;  // 0x03
    cmd[1] = 0x80;  // Address 0x8000 high byte
    cmd[2] = 0x01;  // Address 0x8000 low byte + CRC

    // Fill with 0xFE for initialization
    for (int i = 0; i < N64_PAK_DATA_SIZE; i++) {
        cmd[3 + i] = 0xFE;
    }

    joybus_send_bytes(&controller->_port, cmd, sizeof(cmd));

    // Read response (1 byte CRC)
    uint8_t response;
    uint received = joybus_receive_bytes(
        &controller->_port,
        &response,
        1,
        N64_RECEIVE_TIMEOUT_US,
        true
    );

    return received == 1;
}

bool N64Controller_SetRumble(N64Controller* controller, bool enabled)
{
    if (!controller->_initialized) return false;

    // Build the write command: 0x03 + address (2 bytes) + data (32 bytes) = 35 bytes
    uint8_t cmd[35];
    cmd[0] = N64Command_WRITE_EXPANSION_BUS;  // 0x03

    // Address 0xC000 for rumble pak control
    // Hardcoded known working values: 0xC0, 0x1B
    cmd[1] = 0xC0;
    cmd[2] = 0x1B;

    // Fill data bytes: 0x01 for rumble on, 0x00 for off
    uint8_t rumble_byte = enabled ? 0x01 : 0x00;
    for (int i = 0; i < N64_PAK_DATA_SIZE; i++) {
        cmd[3 + i] = rumble_byte;
    }

    // Send the command
    joybus_send_bytes(&controller->_port, cmd, sizeof(cmd));

    // Read response (1 byte CRC)
    uint8_t response;
    uint received = joybus_receive_bytes(
        &controller->_port,
        &response,
        1,
        N64_RECEIVE_TIMEOUT_US,
        true
    );

    return received == 1;
}
