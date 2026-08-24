/*
 * tempo_sync.h - Shared BPM-based timing for the effects chain
 *
 * From here on, every effect's time-based parameter is derived from the
 * internal tempo (fx_control_get_bpm()) rather than a fixed millisecond
 * value, assuming a 4/4 bar.
 *
 * Convention shared by every tempo-synced effect's FxState (fx_control.h):
 *
 *   param1 - time division, one of 16 steps:
 *                1-8  : straight quarter-note multiples of a bar
 *                       (1 = 1/4, 2 = 2/4, ... 8 = 8/4 == 2 bars)
 *                9-16 : the same 8 lengths again, as triplets
 *                       (9 = 1/4 triplet, ... 16 = 8/4 triplet)
 *              Arrives over the wire as a raw byte (0-255); bucket it into
 *              a step with tempo_sync_step_from_raw() first.
 *
 *   param2 - feedback, raw byte (0-255) scaled to 0.0-FX_FEEDBACK_MAX via
 *              fx_feedback_from_raw(). Capped below 1.0 so no effect can be
 *              parameterized into a runaway self-oscillating loop.
 *
 * A triplet duration is 2/3 of the straight duration at the same step
 * number (n): fitting 3 notes in the space of 2 straight ones is the
 * standard "delay pedal triplet" convention.
 */

#ifndef TEMPO_SYNC_H
#define TEMPO_SYNC_H

#include <stdint.h>

#define TEMPO_SYNC_STEPS 16

// Bucket a raw 0-255 param byte into a 1-16 tempo-sync step.
uint8_t tempo_sync_step_from_raw(uint8_t raw);

// Duration of the given step (1-16) at the given tempo. bpm_x100 uses the
// same encoding as fx_control_get_bpm() (e.g. 12000 == 120.00 BPM).
float tempo_sync_ms(uint8_t step, uint16_t bpm_x100);

// Same, in samples at the given sample rate (rounded to nearest).
uint32_t tempo_sync_samples(uint8_t step, uint16_t bpm_x100, uint32_t sample_rate_hz);

// Convenience: raw param1 byte straight to samples.
uint32_t tempo_sync_samples_from_raw(uint8_t raw, uint16_t bpm_x100, uint32_t sample_rate_hz);

// Feedback param convention (param2): raw byte -> 0.0-FX_FEEDBACK_MAX float.
#define FX_FEEDBACK_MAX 0.95f
float fx_feedback_from_raw(uint8_t raw);

#endif // TEMPO_SYNC_H
