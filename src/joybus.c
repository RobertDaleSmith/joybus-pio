#include "joybus.h"

#include "joybus.pio.h"

#include <stdio.h>
#include <hardware/pio.h>
#include <pico/stdlib.h>

uint joybus_port_init(joybus_port_t *port, uint pin, PIO pio, int sm, int offset) {
    printf("[joybus] port_init: pio=%d, sm=%d, requested_offset=%d, program_len=%d\n",
           pio == pio0 ? 0 : 1, sm, offset, joybus_program.length);

    if (sm < 0) {
        sm = pio_claim_unused_sm(pio, true);
    } else {
        pio_sm_claim(pio, sm);
    }

    if (offset < 0) {
        offset = pio_add_program(pio, &joybus_program);
        printf("[joybus] pio_add_program returned offset=%d\n", offset);
    } else {
        // Use specific offset (e.g., to reserve space for other programs)
        pio_add_program_at_offset(pio, &joybus_program, offset);
        printf("[joybus] pio_add_program_at_offset used offset=%d\n", offset);
    }

    port->pin = pin;
    port->pio = pio;
    port->sm = sm;
    port->offset = offset;
    port->config = joybus_program_get_config(pio, sm, offset, pin);

    joybus_port_reset(port);

    return offset;
}

void joybus_port_terminate(joybus_port_t *port) {
    pio_sm_set_enabled(port->pio, port->sm, false);
    pio_sm_unclaim(port->pio, port->sm);
    pio_remove_program(port->pio, &joybus_program, port->offset);
}

void joybus_port_reset(joybus_port_t *port) {
    joybus_program_receive_init(port->pio, port->sm, port->offset, port->pin, &port->config);
    // Disable SM after init - will be re-enabled when actually polling
    // This prevents interference with other SMs on same PIO (e.g., maple_rx)
    pio_sm_set_enabled(port->pio, port->sm, false);
}

uint __no_inline_not_in_flash_func(joybus_send_receive)(
    joybus_port_t *port,
    uint8_t *message,
    uint message_len,
    uint8_t *response_buf,
    uint response_len,
    uint read_timeout_us
) {
    // If the message has length zero, we send nothing and manually init
    // the state machine for receiving.
    if (message_len > 0) {
        joybus_send_bytes(port, message, message_len);
    } else {
        joybus_port_reset(port);
    }

    return joybus_receive_bytes(port, response_buf, response_len, read_timeout_us, false);
}

void __no_inline_not_in_flash_func(joybus_send_bytes)(
    joybus_port_t *port,
    uint8_t *bytes,
    uint len
) {
    // Wait for line to be high before sending anything.
    while (!gpio_get(port->pin)) {
        tight_loop_contents();
    }

    joybus_program_send_init(port->pio, port->sm, port->offset, port->pin, &port->config);

    for (int i = 0; i < len; i++) {
        joybus_send_byte(port, bytes[i], i == len - 1);
    }
}

void __no_inline_not_in_flash_func(joybus_send_byte)(joybus_port_t *port, uint8_t byte, bool stop) {
    uint32_t data_shifted = (byte << 24) | (stop << 23);
    pio_sm_put_blocking(port->pio, port->sm, data_shifted);
}

uint __no_inline_not_in_flash_func(joybus_receive_bytes)(
    joybus_port_t *port,
    uint8_t *buf,
    uint len,
    uint64_t timeout_us,
    bool first_byte_can_timeout
) {
    uint8_t bytes_received;

    for (bytes_received = 0; bytes_received < len; bytes_received++) {
        if (bytes_received > 0 || first_byte_can_timeout) {
            absolute_time_t timeout_timestamp = make_timeout_time_us(timeout_us);
            while (pio_sm_is_rx_fifo_empty(port->pio, port->sm)) {
                if (time_reached(timeout_timestamp)) {
                    pio_sm_set_enabled(port->pio, port->sm, false);
                    return bytes_received;
                }
            }
        }

        buf[bytes_received] = joybus_receive_byte(port);
    }

    pio_sm_set_enabled(port->pio, port->sm, false);
    return bytes_received;
}

uint8_t __no_inline_not_in_flash_func(joybus_receive_byte)(joybus_port_t *port) {
    return pio_sm_get_blocking(port->pio, port->sm);
}
