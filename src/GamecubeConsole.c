#include "gamecube_definitions.h"
#include "GamecubeConsole.h"
#include "joybus.h"

#include <hardware/pio.h>
#include <hardware/timer.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <stdio.h>
#include <stdint.h> // for uint64_t

#define gc_incoming_bit_length_us 5
#define gc_max_command_bytes 3
#define gc_receive_timeout_us (gc_incoming_bit_length_us * 10)
#define gc_reset_wait_period_us ((gc_incoming_bit_length_us * 8) * (gc_max_command_bytes - 1) + gc_receive_timeout_us)
#define gc_reply_delay (gc_incoming_bit_length_us - 1)  // is uint64_t

gc_report_t default_gc_report = DEFAULT_GC_REPORT_INITIALIZER;
gc_origin_t default_gc_origin = DEFAULT_GC_ORIGIN_INITIALIZER;
gc_status_t default_gc_status = DEFAULT_GC_STATUS_INITIALIZER;

// Initialization function
void GamecubeConsole_init(GamecubeConsole* console, uint pin, PIO pio, int sm, int offset) {
    joybus_port_init(&console->_port, pin, pio, sm, offset);
}

// Termination function
void GamecubeConsole_terminate(GamecubeConsole* console) {
    joybus_port_terminate(&console->_port);
}

bool __no_inline_not_in_flash_func(GamecubeConsole_Detect)(GamecubeConsole* console) {
    uint8_t received[1];
    for (uint8_t attempts = 0; attempts < 10; attempts++) {
        if (joybus_receive_bytes(&console->_port, received, 1, 10000, true) != 1) {
            continue;
        }

        switch ((GamecubeCommand)received[0]) {
            case GamecubeCommand_PROBE:
            case GamecubeCommand_RESET:
                busy_wait_us(gc_reply_delay);
                joybus_send_bytes(&console->_port, (uint8_t *)&default_gc_status, sizeof(gc_status_t));
                break;
            case GamecubeCommand_ORIGIN:
            case GamecubeCommand_RECALIBRATE:
                return true;
            case GamecubeCommand_POLL:
                GamecubeConsole_WaitForPollEnd(console);
                return true;
            default:
                busy_wait_us(gc_reset_wait_period_us);
                joybus_port_reset(&console->_port);
        }
    }
    return false;
}

bool __no_inline_not_in_flash_func(GamecubeConsole_WaitForPoll)(GamecubeConsole* console) {
    while (true) {
        GamecubeConsole_WaitForPollStart(console);
        PollStatus status = GamecubeConsole_WaitForPollEnd(console);

        if (status == PollStatus_ERROR) {
            busy_wait_us(gc_reset_wait_period_us);
            joybus_port_reset(&console->_port);
            continue;
        }

        return status == PollStatus_RUMBLE_ON;
    }
}

void __no_inline_not_in_flash_func(GamecubeConsole_WaitForPollStart)(GamecubeConsole* console) {
    uint8_t received[1];
    while (true) {
        joybus_receive_bytes(&console->_port, received, 1, gc_receive_timeout_us, false);

        switch ((GamecubeCommand)received[0]) {
            case GamecubeCommand_GAME_ID:
                printf("GAMEID: ");
                uint8_t gameId[10];
                uint received_len = joybus_receive_bytes(&console->_port, gameId, 10, gc_receive_timeout_us, false);
                for (uint8_t i = 0; i < received_len; i++) {
                    printf(" %x", gameId[i]);
                }
                printf("\n");
                break;
            case GamecubeCommand_RESET:
            case GamecubeCommand_PROBE:
                busy_wait_us(gc_reply_delay);
                joybus_send_bytes(&console->_port, (uint8_t *)&default_gc_status, sizeof(gc_status_t));
                break;
            case GamecubeCommand_RECALIBRATE:
                printf("RECALIBRATE: ");
                uint8_t recalibrate[2];
                joybus_receive_bytes(&console->_port, recalibrate, 2, gc_receive_timeout_us, false);
                for (uint8_t i = 0; i < 2; i++) {
                    printf(" %x", recalibrate[i]);
                }
                printf("\n");
            case GamecubeCommand_ORIGIN:
                // printf("GamecubeCommand_ORIGIN: 0x%x\n", (GamecubeCommand)received[0]);
                busy_wait_us(gc_reply_delay);
                joybus_send_bytes(&console->_port, (uint8_t *)&default_gc_origin, sizeof(gc_origin_t));
                break;
            case GamecubeCommand_POLL:
                return;
            default:
                printf("COMMAND: 0x%x\n", (GamecubeCommand)received[0]);
                busy_wait_us(gc_reset_wait_period_us);
                joybus_port_reset(&console->_port);
        }
    }
}

PollStatus __no_inline_not_in_flash_func(GamecubeConsole_WaitForPollEnd)(GamecubeConsole* console) {
    uint8_t received[2];
    uint received_len = joybus_receive_bytes(&console->_port, received, 2, gc_receive_timeout_us, true);

    if (received_len != 2 || received[0] > 0x07) {
        return PollStatus_ERROR;
    }

    console->_reading_mode = received[0];
    console->_receive_end = make_timeout_time_us(gc_reply_delay);

    return received[1] & 0x01 ? PollStatus_RUMBLE_ON : PollStatus_RUMBLE_OFF;
}

void __no_inline_not_in_flash_func(GamecubeConsole_SendReport)(GamecubeConsole* console, gc_report_t *report) {
    while (!time_reached(console->_receive_end)) {
        tight_loop_contents();
    }
    // TODO: Translate report according to reading mode.
    joybus_send_bytes(&console->_port, (uint8_t *)report, sizeof(gc_report_t));
}

int GamecubeConsole_GetOffset(GamecubeConsole* console) {
    return console->_port.offset;
}
