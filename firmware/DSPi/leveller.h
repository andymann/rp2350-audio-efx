/*
 * leveller.h - Volume Leveller (Dynamic Range Compressor)
 *
 * Ported from the original DSPi project's own leveller.c/h
 * (github.com/WeebLabs/DSPi#volume-leveller): a feedforward, stereo-
 * linked, single-band RMS compressor with soft-knee upward compression.
 * Simplified for this minimal build's needs -- always stereo L/R (no
 * detector_mask/apply_mask multichannel selection, since this build has
 * exactly one active input source, never more than a stereo pair to
 * detect or apply gain to) and RP2350-float-only (this build has no
 * RP2040/Q28 fixed-point path at all).
 *
 * Runs on mix_l/mix_r right after input source selection (audio_pipeline.c),
 * before the FX chain -- the same structural position ("after input
 * processing, before downstream effects") as the original's own PASS 2.5
 * placement ("after Master EQ, before crossfeed"), adapted to this
 * build's much shorter pipeline (which has neither of those stages).
 *
 * Coefficient convention (Form A; "retention" form), unchanged from the
 * original:
 *   alpha = exp(-log(10) / (Fs * T))
 *   Gives exact 90% step response at time T (0% -> 90%).
 *   alpha near 1.0 = slow (retains previous value)
 *   alpha near 0.0 = fast (tracks input immediately)
 *   Update rule: env = alpha * env + (1 - alpha) * x
 *
 * Control: Set Leveller / Query Leveller commands (0x09/0x0A,
 * fx_control.h) -- this build's dedicated FX UART, not the original's
 * USB vendor control protocol (0xB4-0xBF), which this build doesn't have.
 */

#ifndef LEVELLER_H
#define LEVELLER_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// 5ms lookahead, scaled to this build's actual SAMPLE_RATE_HZ rather than
// the original's fixed 240 (5ms at its own 48kHz) -- this build runs at
// 96kHz (config.h), so 5ms is 480 samples here.
#define LEVELLER_LOOKAHEAD_SAMPLES  ((SAMPLE_RATE_HZ * 5u) / 1000u)

// Speed presets
#define LEVELLER_SPEED_SLOW    0   // Music, orchestral
#define LEVELLER_SPEED_MEDIUM  1   // General purpose
#define LEVELLER_SPEED_FAST    2   // Speech, dialogue
#define LEVELLER_SPEED_COUNT   3

// Parameter limits
#define LEVELLER_AMOUNT_MIN      0.0f
#define LEVELLER_AMOUNT_MAX    100.0f
#define LEVELLER_MAX_GAIN_MIN    0.0f
#define LEVELLER_MAX_GAIN_MAX   35.0f
#define LEVELLER_GATE_MIN      (-96.0f)
#define LEVELLER_GATE_MAX        0.0f

// Fixed internal parameters -- same values as the original
#define LEVELLER_THRESHOLD_DB     (-20.0f)   // Compression threshold (dBFS)
#define LEVELLER_KNEE_WIDTH_DB      6.0f     // Soft knee width (dB)
#define LEVELLER_LIMITER_CEIL     0.70795f   // -3 dBFS gain ceiling

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

typedef struct {
    bool    enabled;
    float   amount;            // 0.0 - 100.0 (compression strength %)
    uint8_t speed;              // LEVELLER_SPEED_SLOW/MEDIUM/FAST
    float   max_gain_db;        // 0.0 - 35.0 dB (max boost for quiet content)
    bool    lookahead;          // Enable 5ms lookahead delay
    float   gate_threshold_db;  // -96.0 - 0.0 dBFS (silence gate level) --
                                 // not exposed over this build's simple FX
                                 // UART protocol; stays at its factory default.
} LevellerConfig;

// Factory defaults -- unchanged from the original
#define LEVELLER_DEFAULT_ENABLED      false
#define LEVELLER_DEFAULT_AMOUNT       50.0f
#define LEVELLER_DEFAULT_SPEED        LEVELLER_SPEED_SLOW
#define LEVELLER_DEFAULT_MAX_GAIN_DB  15.0f
#define LEVELLER_DEFAULT_LOOKAHEAD    true
#define LEVELLER_DEFAULT_GATE_DB      (-96.0f)

// ---------------------------------------------------------------------------
// Derived Coefficients (recomputed on config change)
// ---------------------------------------------------------------------------

typedef struct {
    float alpha_rms;
    float alpha_attack;
    float alpha_release;

    float threshold_db;
    float ratio;
    float knee_width_db;
    float makeup_db;
    float gate_threshold_db;

    float max_gain_db;
} LevellerCoeffs;

// ---------------------------------------------------------------------------
// Runtime State
// ---------------------------------------------------------------------------

typedef struct {
    float env_sq[2];              // L, R RMS squared envelopes

    float gain_smooth_db;
    float gain_linear;
    float gain_prev_linear;

    float lookahead_buf[2][LEVELLER_LOOKAHEAD_SAMPLES];
    uint32_t la_write_idx;
} LevellerState;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Sets factory-default config, computes initial coefficients, and resets
// runtime state. Call once at boot.
void leveller_init(void);

// Process count stereo samples in l/r in-place at sample_rate_hz.
// Marked DSP_TIME_CRITICAL internally; runs in the audio hot path.
void leveller_process_block(float *l, float *r, uint32_t count, uint32_t sample_rate_hz);

// Set Leveller command (0x09, fx_control.c): updates config, recomputes
// coefficients, and resets runtime state if lookahead's on/off state
// changed (matching the original's own "reset when lookahead toggled"
// rule, since the ring's content becomes meaningless the moment its
// bypass state flips).
void leveller_set_config(bool enabled, float amount, uint8_t speed,
                         float max_gain_db, bool lookahead);

// Query Leveller command (0x0A, fx_control.c): current config values.
void leveller_get_config(bool *enabled, float *amount, uint8_t *speed,
                         float *max_gain_db, bool *lookahead);

#endif // LEVELLER_H
