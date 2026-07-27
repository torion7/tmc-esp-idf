#pragma once

// Hardware abstraction for a 3-bit binary chip-select multiplexer
// (e.g. 74HC138). Selects one of up to eight downstream chip-select lines
// from three address pins (A0..A2) plus an enable pin.
//
// The library is self-contained and depends only on the ESP-IDF GPIO
// driver. It is intended to be reusable in any project that needs to
// fan out an SPI chip-select.

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cs_mux_dev_t *cs_mux_handle_t;

typedef struct {
    gpio_num_t pin_a0;
    gpio_num_t pin_a1;
    gpio_num_t pin_a2;
    gpio_num_t pin_enable;
    bool       enable_active_low;
    uint8_t    default_channel;   // address pre-selected at idle (0..7)
} cs_mux_config_t;

esp_err_t cs_mux_init(const cs_mux_config_t *cfg, cs_mux_handle_t *out_handle);
void      cs_mux_deinit(cs_mux_handle_t handle);

// Update the address pins; does not assert the enable line.
esp_err_t cs_mux_set_channel(cs_mux_handle_t handle, uint8_t channel);
uint8_t   cs_mux_get_channel(cs_mux_handle_t handle);

// Drive the enable pin to its "selected" / "idle" level.
esp_err_t cs_mux_assert(cs_mux_handle_t handle);
esp_err_t cs_mux_deassert(cs_mux_handle_t handle);

// Callback adapters with a `void*` signature, suitable as drop-in
// chip-select hooks for SPI driver libraries that accept user-provided
// select/deselect callbacks (e.g. the `tmc5160` library's `cs_ops`).
//
// Use these for single-driver setups where the channel is fixed at the
// MUX's stored value. For multi-driver setups where each driver lives
// on its own channel, use `cs_mux_port_*` below instead.
esp_err_t cs_mux_select_cb(void *handle_as_voidp);
esp_err_t cs_mux_deselect_cb(void *handle_as_voidp);

// ---------------------------------------------------------------------------
// Per-driver port. A `cs_mux_port_t` binds a MUX handle to one channel,
// so several SPI drivers can share the same physical MUX, each addressing
// its own downstream chip. The select callback always re-applies the
// port's channel before asserting the enable line.
//
// Threading note: the shared address pins are serialized internally. The
// port select callback takes a bus lock and holds it until the matching
// deselect callback returns, so channel switch + SPI transfer + deassert
// form one atomic unit across all ports of the same MUX. Callers no longer
// need their own mutex around a single driver call.
//
// The scope is one select..deselect span, i.e. a single SPI datagram.
// Callers that need several datagrams to stay together (e.g. the
// request/fetch pair of a TMC register read) must still add their own lock
// on top.
//
// The `cs_mux_select_cb`/`cs_mux_deselect_cb` adapters above and direct
// `cs_mux_set_channel`/`cs_mux_assert`/`cs_mux_deassert` calls drive the
// address pins WITHOUT taking the bus lock — do not mix them with port
// callbacks on a live MUX.
// ---------------------------------------------------------------------------
typedef struct cs_mux_port_t *cs_mux_port_handle_t;

esp_err_t cs_mux_port_create (cs_mux_handle_t mux, uint8_t channel, cs_mux_port_handle_t *out_port);
void      cs_mux_port_destroy(cs_mux_port_handle_t port);
uint8_t   cs_mux_port_get_channel(cs_mux_port_handle_t port);

// Re-target an existing port to a different MUX channel without
// destroying the port. Useful for scanning / probing multiple chips
// through one driver handle.
esp_err_t cs_mux_port_set_channel(cs_mux_port_handle_t port, uint8_t channel);

esp_err_t cs_mux_port_select_cb  (void *port_as_voidp);
esp_err_t cs_mux_port_deselect_cb(void *port_as_voidp);

#ifdef __cplusplus
}
#endif
