#ifndef _JOYBUS_GAMECUBECONSOLE_H
#define _JOYBUS_GAMECUBECONSOLE_H

#include "gamecube_definitions.h"
#include "joybus.h"

#include <hardware/pio.h>
#include <pico/stdlib.h>

typedef enum {
    PollStatus_RUMBLE_OFF = 0,
    PollStatus_RUMBLE_ON,
    PollStatus_ERROR,
} PollStatus;

typedef struct {
    joybus_port_t _port;
    absolute_time_t _receive_end;
    int _reading_mode;
} GamecubeConsole;

// Function prototypes
void GamecubeConsole_init(GamecubeConsole* console, uint pin, PIO pio, int sm, int offset);
bool GamecubeConsole_Detect(GamecubeConsole* console);
bool GamecubeConsole_WaitForPoll(GamecubeConsole* console);
void GamecubeConsole_WaitForPollStart(GamecubeConsole* console);
PollStatus GamecubeConsole_WaitForPollEnd(GamecubeConsole* console);
void GamecubeConsole_SendReport(GamecubeConsole* console, gc_report_t *report);
int GamecubeConsole_GetOffset(GamecubeConsole* console);

#endif
