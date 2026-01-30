// N64Console.h - C API for N64 Console (Device Mode)
//
// Emulates an N64 controller connected to an N64 console.
// Responds to console poll commands and sends controller reports.
// C wrapper following the same pattern as GamecubeConsole.h

#ifndef _JOYBUS_N64CONSOLE_H
#define _JOYBUS_N64CONSOLE_H

#include "n64_definitions.h"
#include "joybus.h"

#include <hardware/pio.h>
#include <pico/stdlib.h>

typedef struct {
    joybus_port_t _port;
    absolute_time_t _receive_end;
} N64Console_t;

// Function prototypes
void N64Console_init(N64Console_t* console, uint pin, PIO pio, int sm, int offset);
void N64Console_terminate(N64Console_t* console);
bool N64Console_Detect(N64Console_t* console);
bool N64Console_WaitForPoll(N64Console_t* console);
void N64Console_SendReport(N64Console_t* console, n64_report_t *report);
int N64Console_GetOffset(N64Console_t* console);

#endif // _JOYBUS_N64CONSOLE_H
