/*
 * config.h - DSPi minimal build: USB audio in, I2S audio in, FX chain,
 * I2S audio out.
 *
 * This is a from-scratch, minimal replacement for the original DSPi's
 * config.h (which also carried S/PDIF, ADAT, PDM, a persistent-preset
 * system, a vendor USB control protocol, control surfaces/IR remote, and
 * several DSP-island features -- crossfeed, crossover, loudness, psybass,
 * leveller, upmix -- none of which this build includes). Only what the
 * USB/I2S-in -> FX chain -> I2S-out path actually needs.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/audio.h"

// ----------------------------------------------------------------------
// Sample rate / block size
// ----------------------------------------------------------------------

// Fixed operating rate. The original device could switch between
// 44100/48000/96000 at the host's request; this build runs a single
// fixed rate to keep the I2S input/output PIO+DMA setup simple (no
// runtime PIO reload / divider recompute).
//
// Set to 44100 (was originally 48000) while chasing a severe,
// "bitcrusher"-like distortion report -- that change turned out NOT to
// be the actual fix (the real cause was a main-loop architecture issue;
// see main.c's top comment), but 44100 is the configuration that was
// verified working afterward, so it's been left as-is rather than
// switched back and re-verified separately. 307.2MHz (this board's
// fixed system clock -- see main.c) gives an exact-integer PIO clock
// divider for 48kHz-family rates but only an approximate (fractional-
// divider) one at 44100; this introduces a small amount of clock jitter
// that wouldn't exist at 48000, evidently inaudible in practice. 48000
// would need its own round of verification before switching back.
#define SAMPLE_RATE_HZ 44100u

// Samples processed per pipeline call. Matches the original's
// AUDIO_BUFFER_SAMPLES exactly (4ms blocks @ 48kHz) -- every FX effect's
// real-time-safety comments (chunked PSRAM copies, etc.) were reasoned
// about against this same block size.
#define AUDIO_BUFFER_SAMPLES 192u

// ----------------------------------------------------------------------
// Board pins
// ----------------------------------------------------------------------

// USB audio: handled entirely by TinyUSB's device-mode USB PHY (no GPIO
// pins to configure -- the RP2350's USB peripheral is fixed-function).

// I2S: BCK/LRCLK are shared between input and output (this board
// generates one I2S clock domain; the input free-runs against the
// output's clock rather than needing its own clock-recovery/PLL, unlike
// the original's support for an I2S input clocked by a separate,
// external master). Pin numbers match the original DSPi's own defaults
// (config.h's PICO_I2S_BCK_PIN, audio_input.h's PICO_I2S_RX_PIN_DEFAULT,
// usb_audio.c's output_pins[0] default) so a board wired for the
// original firmware doesn't need rewiring for this build.
#define I2S_BCK_PIN       14u   // LRCLK = I2S_BCK_PIN + 1 = GPIO 15
#define I2S_DATA_OUT_PIN   6u   // TX (I2S out) serial data
#define I2S_DATA_IN_PIN    1u   // RX (I2S in) serial data

// No MCLK (master clock) output in this build -- simplification versus
// the original, which could generate one (PICO_I2S_MCK_PIN). Many I2S
// DACs/ADCs derive their internal clocks from BCK alone and don't need
// it; if the connected hardware specifically requires MCLK, that's the
// next thing to add (see pico_audio_i2s_multi's audio_i2s_mck_* API,
// already vendored in pico-extras and unused by this build).

// ----------------------------------------------------------------------
// Firmware version (Query Firmware, fx_control.c)
// ----------------------------------------------------------------------

// Distinct from the original DSPi's version numbering (it was at 1.1.5) --
// this is a different firmware variant (minimal USB/I2S-in -> FX ->
// I2S-out build), not a continuation of that version history.
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
#define FW_VERSION_PATCH 0

// ----------------------------------------------------------------------
// Misc
// ----------------------------------------------------------------------

// Marks a function's code as RAM-resident rather than flash/XIP.
// Required for anything in the per-sample audio hot path so it isn't
// stalled by concurrent flash or (for the FX effects that use it) PSRAM
// QMI-bus traffic -- see fx_delay.c's comment for the detailed
// rationale. Every FX effect file already assumes this macro exists
// under this exact name.
#define DSP_TIME_CRITICAL __attribute__((section(".time_critical")))

#endif // CONFIG_H
