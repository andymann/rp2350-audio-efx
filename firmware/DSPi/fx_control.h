/*
 * fx_control.h - Hardware UART control surface for the multi-effects chain
 *
 * A small, fixed-format command protocol for an external MCU (pedal
 * controller, panel, etc.) to drive the up-to-8-slot effects chain over a
 * dedicated hardware UART. This is intentionally NOT the same transport as
 * uart_control.c (which tunnels the full vendor-command surface behind a
 * synced, CRC-checked frame format on a user-configurable UART/pins). This
 * protocol is fixed at 9600 8N1 on fixed pins, has no sync byte and no CRC,
 * and only understands the five commands below -- it is meant to be dead
 * simple for a small external MCU to bit-bang or talk to from a basic UART
 * peripheral.
 *
 * Wire protocol: every command is 1-8 raw bytes with no framing. Byte 0 is
 * always the command; the receiver knows how many further bytes to expect
 * from the command byte alone.
 *
 *   Set FX        (0x01, 7 bytes total):
 *     0x01, effect_num[0-7], on_off[0-1], param1, param2, param3, dry_wet
 *     On success, the device echoes the exact 7-byte command back.
 *     A frame with effect_num > 7 or on_off > 1 is dropped: no echo, no
 *     state change.
 *
 *     As of the tempo-sync framework, every effect's param1 follows a
 *     shared time-division convention (see tempo_sync.h for the exact
 *     math): param1's byte value IS the step number directly (1-16,
 *     clamped at the edges); 1-8 are straight quarters of a 4/4 bar,
 *     9-16 the same lengths as triplets. What param1's interval actually
 *     controls, and what param2/param3/dry_wet mean (if anything), are
 *     effect-specific -- see the slot registry below and each effect's
 *     own header.
 *
 *     Slot registry (effect_num -> effect):
 *       0  fx_delay.c   - tempo-synced feedback delay. param2 = feedback,
 *                         dry_wet = wet mix.
 *       1  (unassigned)
 *       2  fx_stutter.c - tempo-synced stutter/gate. dry_wet = gate depth
 *                         (255 = full silence when muted, 0 = no gating
 *                         effect at all). param2/param3 unused.
 *       3-7 (unassigned)
 *     An unassigned slot's FxState still updates normally via Set FX (the
 *     control plane doesn't know which slots have a DSP effect wired to
 *     them), it just has nothing reading it yet -- same situation slot 0
 *     was in before fx_delay existed.
 *
 *   Query FX      (0x02, 2 bytes total):
 *     0x02, effect_num[0-7]
 *     Response is 7 bytes, laid out exactly like a Set FX command (leading
 *     byte 0x01, not 0x02) for the requested slot's current state:
 *     0x01, effect_num, on_off, param1, param2, param3, dry_wet
 *     A frame with effect_num > 7 is dropped: no response.
 *
 *   Query Firmware (0x03, 1 byte total):
 *     0x03
 *     Response is always 4 bytes: 0x03, FW_VERSION_MAJOR, FW_VERSION_MINOR,
 *     FW_VERSION_PATCH.
 *
 *   Set BPM        (0x04, 3 bytes total):
 *     0x04, bpm_hi (MSB), bpm_lo (LSB)
 *     The 16-bit value (bpm_hi << 8 | bpm_lo) is the tempo in BPM x100 --
 *     e.g. 12345 (0x30, 0x39) means 123.45 BPM. The full 0-65535 range is
 *     accepted; there is no invalid value, so this command always succeeds.
 *     On success, the device echoes the exact 3-byte command back.
 *
 *   Query BPM      (0x05, 1 byte total):
 *     0x05
 *     Response is 3 bytes: 0x05, bpm_hi (MSB), bpm_lo (LSB) -- the current
 *     stored tempo, same encoding as Set BPM. Defaults to 12000 (120.00 BPM)
 *     at boot until a Set BPM command changes it.
 *
 * On boot, before any command is processed, the device sends the literal
 * ASCII string "Andyland.info" (13 bytes, no framing) unsolicited as a
 * liveness/presence banner. It is not part of the command/response protocol
 * -- it carries no command byte and expects no reply.
 *
 * Design rule (matches the rest of the control-interface code in this
 * project): nothing here ever blocks or busy-waits. The UART IRQ only drains
 * the RX FIFO into a ring buffer; all parsing, state changes and TX happen
 * from fx_control_poll() in main-loop context, so the audio pipeline is
 * never delayed.
 *
 * Effect state is held here as a simple control-plane store (fx_control_get)
 * so the actual DSP effect implementations can be wired in incrementally --
 * this module does not yet touch the audio pipeline itself.
 */

#ifndef FX_CONTROL_H
#define FX_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#define FX_CONTROL_NUM_EFFECTS   8u

typedef struct {
    uint8_t enabled;   // 0 or 1
    uint8_t param1;
    uint8_t param2;
    uint8_t param3;
    uint8_t dry_wet;   // 0-255
} FxState;

// Bring up the dedicated FX-control UART (fixed pins/baud; see fx_control.c).
// Call once at boot, after the audio pins above it in main.c's init order
// have claimed their GPIOs (mirrors uart_ctrl_init's ordering rule).
void fx_control_init(void);

// Main-loop tick: drain the RX ring, parse commands, update state, and pump
// any pending echo/response bytes. Never blocks; safe to call every
// iteration. Mirrors uart_ctrl_poll()/i2c_ctrl_poll()'s calling convention.
void fx_control_poll(void);

// True iff the FX-control UART is up and `pin` is its TX or RX GPIO (for the
// pin-collision checks the rest of the control-interface code performs).
bool fx_control_owns_pin(uint8_t pin);

// Read the current state of one effect slot (0-7). Returns false and leaves
// *out untouched if effect_num is out of range. For use by the DSP pipeline
// once individual effects are implemented, and by fx_control.c itself when
// building a Query FX response.
bool fx_control_get(uint8_t effect_num, FxState *out);

// Current stored tempo in BPM x100 (e.g. 12345 == 123.45 BPM), as last set
// by a Set BPM command. Defaults to 12000 (120.00 BPM) at boot. For use by
// the
// DSP pipeline once tempo-synced effects exist, and by fx_control.c itself
// when building a Query BPM response.
uint16_t fx_control_get_bpm(void);

#endif // FX_CONTROL_H
