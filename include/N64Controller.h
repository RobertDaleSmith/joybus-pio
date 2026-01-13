#ifndef _JOYBUS_N64CONTROLLER_H
#define _JOYBUS_N64CONTROLLER_H

#include "joybus.h"
#include "n64_definitions.h"

#include <hardware/pio.h>
#include <pico/stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// N64 Controller Reader context
typedef struct {
    joybus_port_t _port;
    uint _polling_period_us;
    n64_status_t _status;
    bool _initialized;
    absolute_time_t _next_poll;
} N64Controller;

/**
 * @brief Initialize an N64 controller reader
 *
 * @param controller Pointer to N64Controller context
 * @param pin The GPIO pin connected to the N64 controller's data line
 * @param polling_rate The frequency (in Hz) at which to poll the controller (typically 60)
 * @param pio The PIO instance; either pio0 or pio1
 * @param sm The PIO state machine to use (-1 for auto-claim)
 * @param offset The instruction memory offset (-1 for auto-allocate)
 */
void N64Controller_init(N64Controller* controller, uint pin, uint polling_rate,
                        PIO pio, int sm, int offset);

/**
 * @brief Cleanly terminate the N64 controller reader
 *
 * @param controller Pointer to N64Controller context
 */
void N64Controller_terminate(N64Controller* controller);

/**
 * @brief Poll the N64 controller for its current state
 *
 * This function handles the full polling sequence including:
 * - Initial probe/origin sequence if not yet initialized
 * - Rate limiting to respect polling_rate
 * - Reading the controller report
 *
 * @param controller Pointer to N64Controller context
 * @param report Pointer to buffer where controller state will be written
 * @param rumble True to enable rumble pak, false to disable
 * @return true if poll was successful and report is valid
 * @return false if controller not responding (will retry init on next poll)
 */
bool N64Controller_Poll(N64Controller* controller, n64_report_t* report, bool rumble);

/**
 * @brief Get the PIO program offset
 *
 * Useful for sharing the joybus PIO program across multiple controllers
 *
 * @param controller Pointer to N64Controller context
 * @return The instruction memory offset where the PIO program is loaded
 */
int N64Controller_GetOffset(N64Controller* controller);

/**
 * @brief Check if controller is initialized
 *
 * @param controller Pointer to N64Controller context
 * @return true if controller has been successfully probed
 */
bool N64Controller_IsInitialized(N64Controller* controller);

/**
 * @brief Get the controller's device type from probe response
 *
 * @param controller Pointer to N64Controller context
 * @return Pointer to the status struct (valid only if initialized)
 */
const n64_status_t* N64Controller_GetStatus(N64Controller* controller);

/**
 * @brief Check if a Rumble Pak is detected
 *
 * @param controller Pointer to N64Controller context
 * @return true if pak is present (could be Rumble or Memory pak)
 */
bool N64Controller_HasPak(N64Controller* controller);

/**
 * @brief Initialize the rumble pak
 *
 * Must be called before SetRumble will work. Writes initialization
 * sequence to address 0x8000.
 *
 * @param controller Pointer to N64Controller context
 * @return true if initialization was acknowledged
 */
bool N64Controller_InitRumblePak(N64Controller* controller);

/**
 * @brief Set rumble pak state
 *
 * Sends the rumble pak control command to enable or disable rumble.
 * Only works if a Rumble Pak is inserted and initialized.
 *
 * @param controller Pointer to N64Controller context
 * @param enabled true to enable rumble, false to disable
 * @return true if command was sent successfully
 */
bool N64Controller_SetRumble(N64Controller* controller, bool enabled);

#ifdef __cplusplus
}
#endif

#endif // _JOYBUS_N64CONTROLLER_H
