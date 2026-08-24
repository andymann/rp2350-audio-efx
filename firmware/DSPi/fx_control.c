/*
 * fx_control.c - Hardware UART control surface for the multi-effects chain
 *
 * See fx_control.h for the wire protocol. As with uart_control.c, nothing
 * here ever blocks: the UART IRQ only drains the RX FIFO into a ring, and
 * every parse/state-update/TX step happens from fx_control_poll() in
 * main-loop context.
 */

#include "fx_control.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Fixed pins / baud
// ---------------------------------------------------------------------------
//
// TODO(andy): confirmed pins per your board (physical pin 21 = GPIO16/TXD0,
// physical pin 22 = GPIO17/RXD0). Note these are also UART_CTRL_DEFAULT_TX_PIN/
// RX_PIN in config.h -- the *stock* default pins for the separate, configurable
// uart_control.c interface. That interface ships disabled, so there's no
// conflict at boot; fx_control claims these pins first (see the init-order
// comment in main.c), so if uart_control is ever enabled on its default pins
// it will correctly fail pin validation (PIN_CONFIG_PIN_IN_USE) rather than
// silently collide. Give it different pins via REQ_SET_UART_CONFIG if you
// want both interfaces live at once.
#ifndef FX_UART_TX_PIN
#define FX_UART_TX_PIN   16
#endif
#ifndef FX_UART_RX_PIN
#define FX_UART_RX_PIN   17
#endif
#define FX_UART_BAUD     9600u

#define RX_RING_SIZE     32u                  // power of two; frames are tiny
#define RX_RING_MASK     (RX_RING_SIZE - 1u)

#define FRAME_TIMEOUT_US 50000u               // inter-byte mid-frame timeout

// Commands
#define CMD_SET_FX        0x01u
#define CMD_QUERY_FX      0x02u
#define CMD_QUERY_FW      0x03u
#define CMD_SET_BPM       0x04u
#define CMD_QUERY_BPM     0x05u

#define SET_FX_LEN          7u   // cmd + effect_num + on_off + p1 + p2 + p3 + drywet
#define QUERY_FX_LEN        2u   // cmd + effect_num
#define QUERY_FW_LEN        1u   // cmd only
#define QUERY_FW_RESP_LEN   4u   // cmd + major + minor + patch
#define SET_BPM_LEN         3u   // cmd + bpm_hi + bpm_lo
#define QUERY_BPM_LEN       1u   // cmd only
#define QUERY_BPM_RESP_LEN  3u   // cmd + bpm_hi + bpm_lo
#define MAX_FRAME_LEN       8u   // largest of the above, matches the protocol's cap

// Boot banner: sent once, unsolicited, right after the port comes up --
// simple presence/liveness check for whatever's listening on the other end.
// Not part of the command/response protocol (no command byte, no echo).
#define BOOT_BANNER      "Andyland.info"
#define BOOT_BANNER_LEN  (sizeof(BOOT_BANNER) - 1u)   // exclude the NUL

// TX buffer must hold the largest of: a Set-FX echo/Query-FX response
// (SET_FX_LEN) or the boot banner (BOOT_BANNER_LEN).
#define TX_BUF_CAP  (BOOT_BANNER_LEN > SET_FX_LEN ? BOOT_BANNER_LEN : SET_FX_LEN)

// ---------------------------------------------------------------------------
// Peripheral state
// ---------------------------------------------------------------------------

static uart_inst_t *g_uart = NULL;
static uint          g_irq = 0;
static bool          g_is_live = false;

// ---------------------------------------------------------------------------
// Effect state store
// ---------------------------------------------------------------------------

static FxState fx_state[FX_CONTROL_NUM_EFFECTS];

// Tempo, in BPM x100 (e.g. 12345 == 123.45 BPM). Defaults to 120.00 BPM
// (12000) at boot until a Set BPM command changes it.
#define BPM_X100_DEFAULT  12000u   // 120.00 BPM
static uint16_t bpm_x100 = BPM_X100_DEFAULT;

// ---------------------------------------------------------------------------
// RX ring (single-producer ISR, single-consumer poll)
// ---------------------------------------------------------------------------

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;   // written by ISR only
static volatile uint16_t rx_tail = 0;   // written by consumer only

// ---------------------------------------------------------------------------
// Parser state
// ---------------------------------------------------------------------------

static uint8_t  frame_buf[MAX_FRAME_LEN];
static uint8_t  frame_len = 0;          // total bytes expected for this frame
static uint8_t  frame_pos = 0;          // bytes received so far
static uint32_t last_byte_time = 0;

// ---------------------------------------------------------------------------
// TX (echo/response) pump
// ---------------------------------------------------------------------------

static uint8_t  tx_buf[TX_BUF_CAP];
static uint8_t  tx_len = 0;
static uint8_t  tx_pos = 0;

static void fx_control_irq(void) {
    while (uart_is_readable(g_uart)) {
        uint8_t b = (uint8_t)uart_get_hw(g_uart)->dr;
        uint16_t next = (uint16_t)((rx_head + 1u) & RX_RING_MASK);
        if (next != rx_tail) {   // drop the byte silently if the ring is full
            rx_ring[rx_head] = b;
            rx_head = next;
        }
    }
}

static void reset_parser(void) {
    frame_len = 0;
    frame_pos = 0;
}

static void start_tx(const uint8_t *data, uint8_t len) {
    memcpy(tx_buf, data, len);
    tx_len = len;
    tx_pos = 0;
}

static void pump_tx(void) {
    while (tx_pos < tx_len && uart_is_writable(g_uart)) {
        uart_get_hw(g_uart)->dr = tx_buf[tx_pos++];
    }
}

// Returns the total frame length for a command byte, or 0 if the command is
// unrecognised (caller drops the byte and stays in sync on the next one).
static uint8_t expected_len_for_cmd(uint8_t cmd) {
    switch (cmd) {
        case CMD_SET_FX:     return SET_FX_LEN;
        case CMD_QUERY_FX:   return QUERY_FX_LEN;
        case CMD_QUERY_FW:   return QUERY_FW_LEN;
        case CMD_SET_BPM:    return SET_BPM_LEN;
        case CMD_QUERY_BPM:  return QUERY_BPM_LEN;
        default:             return 0;
    }
}

// Build a 7-byte Set-FX-shaped state frame for `effect_num` into `out`
// (used both for the Set FX echo and the Query FX response).
static void build_fx_frame(uint8_t effect_num, uint8_t *out) {
    const FxState *s = &fx_state[effect_num];
    out[0] = CMD_SET_FX;
    out[1] = effect_num;
    out[2] = s->enabled;
    out[3] = s->param1;
    out[4] = s->param2;
    out[5] = s->param3;
    out[6] = s->dry_wet;
}

static void handle_set_fx(const uint8_t *f) {
    uint8_t effect_num = f[1];
    uint8_t on_off     = f[2];
    if (effect_num >= FX_CONTROL_NUM_EFFECTS || on_off > 1) return;  // drop, no echo

    FxState *s = &fx_state[effect_num];
    s->enabled = on_off;
    s->param1  = f[3];
    s->param2  = f[4];
    s->param3  = f[5];
    s->dry_wet = f[6];

    // TODO: wire into the DSP pipeline once the individual effects (0-7)
    // are implemented. For now this just updates the control-plane state
    // fx_control_get() exposes.

    start_tx(f, SET_FX_LEN);   // echo the command verbatim
}

static void handle_query_fx(const uint8_t *f) {
    uint8_t effect_num = f[1];
    if (effect_num >= FX_CONTROL_NUM_EFFECTS) return;  // drop, no response

    uint8_t resp[SET_FX_LEN];
    build_fx_frame(effect_num, resp);
    start_tx(resp, SET_FX_LEN);
}

static void handle_query_fw(void) {
    uint8_t resp[QUERY_FW_RESP_LEN] = {
        CMD_QUERY_FW,
        (uint8_t)FW_VERSION_MAJOR,
        (uint8_t)FW_VERSION_MINOR,
        (uint8_t)FW_VERSION_PATCH,
    };
    start_tx(resp, QUERY_FW_RESP_LEN);
}

// Set BPM has no invalid value (the full 0-65535 range is meaningful), so
// this always succeeds and always echoes.
static void handle_set_bpm(const uint8_t *f) {
    bpm_x100 = (uint16_t)(((uint16_t)f[1] << 8) | f[2]);

    // TODO: wire into the DSP pipeline once tempo-synced effects exist. For
    // now this just updates the control-plane state fx_control_get_bpm()
    // exposes.

    start_tx(f, SET_BPM_LEN);   // echo the command verbatim
}

static void handle_query_bpm(void) {
    uint8_t resp[QUERY_BPM_RESP_LEN] = {
        CMD_QUERY_BPM,
        (uint8_t)(bpm_x100 >> 8),
        (uint8_t)(bpm_x100 & 0xFF),
    };
    start_tx(resp, QUERY_BPM_RESP_LEN);
}

static void dispatch_frame(void) {
    switch (frame_buf[0]) {
        case CMD_SET_FX:     handle_set_fx(frame_buf);   break;
        case CMD_QUERY_FX:   handle_query_fx(frame_buf); break;
        case CMD_QUERY_FW:   handle_query_fw();          break;
        case CMD_SET_BPM:    handle_set_bpm(frame_buf);  break;
        case CMD_QUERY_BPM:  handle_query_bpm();         break;
        default: break;   // unreachable: expected_len_for_cmd() already filtered
    }
}

// Pull ring bytes into the frame buffer and dispatch complete frames. Only
// called when TX is idle, so a response is never interleaved with the next
// request (matches uart_ctrl's "transport otherwise idle" rule).
static void parse_ring(void) {
    while (rx_tail != rx_head) {
        uint8_t b = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) & RX_RING_MASK);
        last_byte_time = time_us_32();

        if (frame_len == 0) {
            frame_len = expected_len_for_cmd(b);
            if (frame_len == 0) continue;   // unknown command byte: drop, resync
            frame_buf[0] = b;
            frame_pos = 1;
            if (frame_pos == frame_len) { dispatch_frame(); reset_parser(); }
            continue;
        }

        frame_buf[frame_pos++] = b;
        if (frame_pos == frame_len) {
            dispatch_frame();
            reset_parser();
            // A response may now be queued in tx_buf; stop draining the ring
            // this call so pump_tx() gets a chance before more bytes parse.
            if (tx_len) return;
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void fx_control_init(void) {
    memset(fx_state, 0, sizeof(fx_state));

    // Slot 3 (fx_phaser.c, FX_PHASER_EFFECT_NUM) ships with non-zero
    // defaults so it's immediately musical the first time it's turned on,
    // without needing a full Set FX command to pick sane values first:
    // param1=0x06 (quarter-note LFO rate), param2=0xC0 (~75% sweep depth,
    // 192/255). Still off (enabled=0, same as every other slot) until an
    // explicit Set FX turns it on -- this only pre-loads param1/param2,
    // it doesn't start the effect running at boot.
    fx_state[3].param1 = 0x06u;
    fx_state[3].param2 = 0xC0u;

    // Slot 4 (fx_djfilter.c, FX_DJFILTER_EFFECT_NUM) similarly ships with
    // non-zero defaults: param1=0x7F (127, exact bypass -- the filter
    // does nothing until explicitly swept away from center) and
    // param2=0x40 (mild resonance, well below the 0xC0-ish territory
    // where a DJ filter starts sounding aggressive/resonant-peaky).
    fx_state[4].param1 = 0x7Fu;
    fx_state[4].param2 = 0x40u;

    bpm_x100 = BPM_X100_DEFAULT;
    reset_parser();
    rx_head = 0;
    rx_tail = 0;
    tx_len = 0;
    tx_pos = 0;

    uint8_t idx = (uint8_t)((((FX_UART_TX_PIN >> 2) + 1) >> 1) & 1);
    g_uart = idx ? uart1 : uart0;
    g_irq  = idx ? UART1_IRQ : UART0_IRQ;

    uart_init(g_uart, FX_UART_BAUD);
    uart_set_format(g_uart, 8, 1, UART_PARITY_NONE);   // 9600 8N1
    uart_set_hw_flow(g_uart, false, false);
    uart_set_fifo_enabled(g_uart, true);
    gpio_set_function(FX_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(FX_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(FX_UART_RX_PIN);   // keep RX idle-high with no MCU attached

    while (uart_is_readable(g_uart)) (void)uart_get_hw(g_uart)->dr;  // drain stale RX

    irq_set_exclusive_handler(g_irq, fx_control_irq);
    irq_set_priority(g_irq, 0xC0);
    irq_set_enabled(g_irq, true);
    uart_set_irq_enables(g_uart, true, false);  // RX + RX-timeout, no TX IRQ

    g_is_live = true;

    // Queue the boot banner. Bytes go into the UART's TX FIFO here (a few
    // microseconds), not onto the wire -- the peripheral shifts them out
    // asynchronously over the following ~14ms at 9600 baud while the rest of
    // boot proceeds, so this does not block. If a Set/Query command arrives
    // before the banner finishes draining, it queues in the RX ring and is
    // parsed once the banner's TX completes (fx_control_poll() drains tx_buf
    // before parsing new frames).
    start_tx((const uint8_t *)BOOT_BANNER, BOOT_BANNER_LEN);
}

void fx_control_poll(void) {
    if (!g_is_live) return;

    // Inter-byte timeout: drop a stalled partial frame, send nothing.
    if (frame_len != 0 &&
        (uint32_t)(time_us_32() - last_byte_time) > FRAME_TIMEOUT_US)
        reset_parser();

    if (tx_len) pump_tx();
    if (tx_pos == tx_len) { tx_len = 0; tx_pos = 0; }

    if (!tx_len) parse_ring();
}

bool fx_control_owns_pin(uint8_t pin) {
    return g_is_live && (pin == FX_UART_TX_PIN || pin == FX_UART_RX_PIN);
}

// Called every audio block from fx_delay_process_block() (RP2350: the whole
// call chain from there down needs to stay RAM-resident -- PSRAM and flash
// share the same physical QMI bus on this chip, so any link in that chain
// still executing from flash via XIP competes with the PSRAM data traffic
// the effect itself is doing. See fx_delay.c's history comment.
DSP_TIME_CRITICAL
bool fx_control_get(uint8_t effect_num, FxState *out) {
    if (effect_num >= FX_CONTROL_NUM_EFFECTS || !out) return false;
    *out = fx_state[effect_num];
    return true;
}

DSP_TIME_CRITICAL
uint16_t fx_control_get_bpm(void) {
    return bpm_x100;
}
