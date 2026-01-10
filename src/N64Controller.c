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
    // If controller is uninitialized, do probe/origin sequence
    if (!controller->_initialized) {
        N64Controller_wait_poll_cooldown(controller);

        controller->_initialized = N64Controller_do_init(controller);
        if (!controller->_initialized) {
            return false;
        }
    }

    N64Controller_wait_poll_cooldown(controller);

    // Send poll command
    // Byte 0: POLL command (0x01)
    // Byte 1: 0x03 (controller pak address high byte)
    // Byte 2: rumble (0x00 = off, 0x01 = on)
    uint8_t poll_cmd[] = { N64Command_POLL, 0x03, rumble ? 0x01 : 0x00 };
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
