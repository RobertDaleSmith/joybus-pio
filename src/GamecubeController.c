/**
 * GamecubeController.c - GameCube Controller Reader (C port)
 *
 * Reads Nintendo GameCube controllers using the joybus protocol over PIO.
 * Ported from GamecubeController.cpp for use in C projects.
 *
 * Based on joybus-pio library by:
 * https://github.com/JonnyHaystack/joybus-pio
 */

#include "GamecubeController.h"
#include "gamecube_definitions.h"
#include "joybus.h"

#include <stdio.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <string.h>

// Timing constants
#define GC_INCOMING_BIT_LENGTH_US 4
// Give a whole 5 bytes of leniency on receive so the controller has time to do processing
// before responding. And another byte of leniency because the joybus_receive_bytes (and thus
// the timeout) starts soon after the PIO starts sending our first byte.
#define GC_RECEIVE_TIMEOUT_US (5 * GC_INCOMING_BIT_LENGTH_US * 10)  // 200μs

// ============================================================================
// Internal Functions
// ============================================================================

static void __no_inline_not_in_flash_func(GamecubeController_wait_poll_cooldown)(GamecubeController* controller)
{
    // Wait for start of next polling period
    while (!time_reached(controller->_next_poll)) {
        tight_loop_contents();
    }

    // Reset poll cooldown
    controller->_next_poll = make_timeout_time_us(controller->_polling_period_us);
}

static bool __no_inline_not_in_flash_func(GamecubeController_do_init)(GamecubeController* controller)
{
    // Send probe command
    uint8_t probe_cmd[] = { GamecubeCommand_PROBE };
    joybus_send_bytes(&controller->_port, probe_cmd, sizeof(probe_cmd));

    // Read and validate probe response (3 bytes: device ID + status)
    uint received_len = joybus_receive_bytes(
        &controller->_port,
        (uint8_t*)&controller->_status,
        sizeof(gc_status_t),
        GC_RECEIVE_TIMEOUT_US,
        true
    );

    // If response is invalid, return false
    if (received_len != sizeof(gc_status_t) || controller->_status.device == 0) {
        return false;
    }

    // Wait until start of next polling period before sending origin
    GamecubeController_wait_poll_cooldown(controller);

    // Send origin command (0x41)
    uint8_t origin_cmd[] = { GamecubeCommand_ORIGIN };
    joybus_send_bytes(&controller->_port, origin_cmd, sizeof(origin_cmd));

    // Read and validate origin response (10 bytes: gc_origin_t)
    gc_origin_t origin;
    received_len = joybus_receive_bytes(
        &controller->_port,
        (uint8_t*)&origin,
        sizeof(gc_origin_t),
        GC_RECEIVE_TIMEOUT_US,
        true
    );

    // If response is invalid, return false
    if (received_len != sizeof(gc_origin_t)) {
        return false;
    }

    return true;
}

// ============================================================================
// Public API
// ============================================================================

void GamecubeController_init(GamecubeController* controller, uint pin, uint polling_rate,
                              PIO pio, int sm, int offset)
{
    memset(controller, 0, sizeof(GamecubeController));

    joybus_port_init(&controller->_port, pin, pio, sm, offset);
    controller->_polling_period_us = 1000000 / polling_rate;
    controller->_next_poll = get_absolute_time();
    controller->_initialized = false;

    // Initialize status to zero
    memset(&controller->_status, 0, sizeof(gc_status_t));
}

void GamecubeController_terminate(GamecubeController* controller)
{
    joybus_port_terminate(&controller->_port);
}

bool __no_inline_not_in_flash_func(GamecubeController_Poll)(GamecubeController* controller,
                                                             gc_report_t* report, bool rumble)
{
    static uint8_t init_fail_count = 0;

    // If controller is uninitialized, do probe/origin sequence
    if (!controller->_initialized) {
        // When not initialized, use non-blocking check instead of blocking wait
        // This prevents starving other tasks when no GC controller is connected
        if (!time_reached(controller->_next_poll)) {
            return false;  // Not time to retry yet, return immediately
        }

        // Backoff: short for reconnection attempts, longer if no controller
        // First 5 retries: 16ms (quick reconnect after brief glitch)
        // After that: 500ms (controller likely not connected)
        uint32_t backoff_ms = (init_fail_count < 5) ? 16 : 500;
        controller->_next_poll = make_timeout_time_ms(backoff_ms);

        controller->_initialized = GamecubeController_do_init(controller);
        if (!controller->_initialized) {
            if (init_fail_count < 255) init_fail_count++;
            return false;
        }
        init_fail_count = 0;  // Reset on success
    }

    // Non-blocking cooldown check for regular polling
    // This prevents starving other tasks
    if (!time_reached(controller->_next_poll)) {
        return false;  // Not time to poll yet, return immediately
    }
    controller->_next_poll = make_timeout_time_us(controller->_polling_period_us);

    // Send poll command: 0x40 + mode (0x03) + rumble byte
    // Mode 0x03 returns full 8-byte report with analog sticks and triggers
    uint8_t poll_cmd[] = { GamecubeCommand_POLL, 0x03, rumble ? 1 : 0 };
    joybus_send_bytes(&controller->_port, poll_cmd, sizeof(poll_cmd));

    // Read and validate report (8 bytes)
    uint8_t received_len = joybus_receive_bytes(
        &controller->_port,
        (uint8_t*)report,
        sizeof(gc_report_t),
        GC_RECEIVE_TIMEOUT_US,
        true
    );

    // If report origin bit is 1, it indicates that the controller is not initialized properly,
    // so we want to restart the initialization process.
    if (received_len != sizeof(gc_report_t) || report->origin) {
        controller->_initialized = false;
        return false;
    }

    return true;
}

int GamecubeController_GetOffset(GamecubeController* controller)
{
    return controller->_port.offset;
}

bool GamecubeController_IsInitialized(GamecubeController* controller)
{
    return controller->_initialized;
}

const gc_status_t* GamecubeController_GetStatus(GamecubeController* controller)
{
    return &controller->_status;
}
