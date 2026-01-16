/**
 * GamecubeController.h - GameCube Controller Reader (C port)
 *
 * Reads Nintendo GameCube controllers using the joybus protocol over PIO.
 * Ported from GamecubeController.cpp for use in C projects.
 *
 * Based on joybus-pio library by:
 * https://github.com/JonnyHaystack/joybus-pio
 */

#ifndef _JOYBUS_GAMECUBECONTROLLER_H
#define _JOYBUS_GAMECUBECONTROLLER_H

#include "joybus.h"
#include "gamecube_definitions.h"

#include <hardware/pio.h>
#include <pico/stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// GameCube Controller Reader context
typedef struct {
    joybus_port_t _port;
    uint _polling_period_us;
    gc_status_t _status;
    bool _initialized;
    absolute_time_t _next_poll;
} GamecubeController;

/**
 * @brief Initialize a GameCube controller reader
 *
 * @param controller Pointer to GamecubeController context
 * @param pin The GPIO pin connected to the GameCube controller's data line
 * @param polling_rate The frequency (in Hz) at which to poll the controller (typically 125)
 * @param pio The PIO instance; either pio0 or pio1
 * @param sm The PIO state machine to use (-1 for auto-claim)
 * @param offset The instruction memory offset (-1 for auto-allocate)
 */
void GamecubeController_init(GamecubeController* controller, uint pin, uint polling_rate,
                              PIO pio, int sm, int offset);

/**
 * @brief Cleanly terminate the GameCube controller reader
 *
 * @param controller Pointer to GamecubeController context
 */
void GamecubeController_terminate(GamecubeController* controller);

/**
 * @brief Poll the GameCube controller for its current state
 *
 * This function handles the full polling sequence including:
 * - Initial probe/origin sequence if not yet initialized
 * - Rate limiting to respect polling_rate
 * - Reading the controller report
 *
 * @param controller Pointer to GamecubeController context
 * @param report Pointer to buffer where controller state will be written
 * @param rumble True to enable rumble, false to disable
 * @return true if poll was successful and report is valid
 * @return false if controller not responding (will retry init on next poll)
 */
bool GamecubeController_Poll(GamecubeController* controller, gc_report_t* report, bool rumble);

/**
 * @brief Get the PIO program offset
 *
 * Useful for sharing the joybus PIO program across multiple controllers
 *
 * @param controller Pointer to GamecubeController context
 * @return The instruction memory offset where the PIO program is loaded
 */
int GamecubeController_GetOffset(GamecubeController* controller);

/**
 * @brief Check if controller is initialized
 *
 * @param controller Pointer to GamecubeController context
 * @return true if controller has been successfully probed
 */
bool GamecubeController_IsInitialized(GamecubeController* controller);

/**
 * @brief Get the controller's device type from probe response
 *
 * @param controller Pointer to GamecubeController context
 * @return Pointer to the status struct (valid only if initialized)
 */
const gc_status_t* GamecubeController_GetStatus(GamecubeController* controller);

#ifdef __cplusplus
}
#endif

#endif // _JOYBUS_GAMECUBECONTROLLER_H
