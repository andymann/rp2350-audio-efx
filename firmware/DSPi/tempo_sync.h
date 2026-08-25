/*
 * tempo_sync.h - Shared BPM-based timing for the effects chain
 *
 * From here on, every effect's time-based parameter is derived from the
 * internal tempo (fx_control_get_bpm()) rather than a fixed millisecond
 * value, assuming a 4/4 bar.
 *
 * param1's actual unit is effect-specific (see fx_control.h's slot
 * registry and each effect's own header) -- this module offers two
 * conventions effects can opt into, not one mandatory shared scheme:
 *
 *   tempo_sync_step_from_raw() / tempo_sync_ms() / tempo_sync_samples():
 *     param1 is one of 16 steps:
 *       1-8  : straight quarter-note multiples of a bar
 *              (1 = 1/4, 2 = 2/4, ... 8 = 8/4 == 2 bars)
 *       9-16 : the same 8 lengths again, as triplets
 *              (9 = 1/4 triplet, ... 16 = 8/4 triplet)
 *     param1's raw byte value ACCEPTS 0-15 over the wire, mapping
 *     directly to steps 1-16 (0 -> step 1, 15 -> step 16);
 *     tempo_sync_step_from_raw() clamps anything above 15 down to 15
 *     (step 16) rather than mapping it -- effectively ignoring the rest
 *     of the raw byte's range. Used by fx_delay.
 *
 *     A triplet duration is 2/3 of the straight duration at the same step
 *     number (n): fitting 3 notes in the space of 2 straight ones is the
 *     standard "delay pedal triplet" convention.
 *
 *   tempo_sync_bar_fraction_ms() / tempo_sync_bar_fraction_samples():
 *     param1 is a direct count of 1/subdivisions_per_bar-note units (the
 *     caller picks subdivisions_per_bar -- 16 for sixteenth notes, 32 for
 *     32nd notes, etc.), NOT bucketed or stepped -- param1=1 means one
 *     such unit, param1=subdivisions_per_bar means a full bar, and so on
 *     up to 255. No triplet variant. Used by fx_stutter, which needed
 *     finer resolution than the 16-step scheme offers (its shortest
 *     interval there is a quarter note); the exact subdivision fx_stutter
 *     uses is its own choice, see FX_STUTTER_SUBDIVISIONS_PER_BAR in
 *     fx_stutter.h -- changing it there doesn't require touching this
 *     file, precisely so another "make it finer" request doesn't need a
 *     new tempo_sync function each time.
 *
 * param2 - feedback, raw byte (0-255) scaled to 0.0-FX_FEEDBACK_MAX via
 *   fx_feedback_from_raw(). Capped below 1.0 so no effect can be
 *   parameterized into a runaway self-oscillating loop. Used by fx_delay;
 *   not a mandatory convention either -- see each effect's own header for
 *   what param2/param3/dry_wet mean there.
 */

#ifndef TEMPO_SYNC_H
#define TEMPO_SYNC_H

#include <stdint.h>

#define TEMPO_SYNC_STEPS 16

// Shared "raw wire byte -> 1-indexed internal value" clamp: accepts
// [0, max_accepted] and returns raw+1 (0 -> 1, max_accepted ->
// max_accepted+1); anything above max_accepted clamps down to it first
// (ignored, same as sending max_accepted itself). max_accepted must be
// < 255 (the +1 would otherwise overflow a uint8_t). Used by
// tempo_sync_step_from_raw() and directly by effects that need this
// same "accept 0-N, ignore above" pattern for their own param1 range
// (see fx_stutter.c, fx_phaser.c, fx_beatrepeat.c).
uint8_t tempo_sync_clamp1_from_raw(uint8_t raw, uint8_t max_accepted);

// Clamp param1's raw byte value (0-15 accepted) into the 1-16 tempo-sync
// step range -- see tempo_sync_clamp1_from_raw() for the general pattern
// this is built on.
uint8_t tempo_sync_step_from_raw(uint8_t raw);

// Duration of the given step (1-16) at the given tempo. bpm_x100 uses the
// same encoding as fx_control_get_bpm() (e.g. 12000 == 120.00 BPM).
float tempo_sync_ms(uint8_t step, uint16_t bpm_x100);

// Same, in samples at the given sample rate (rounded to nearest).
uint32_t tempo_sync_samples(uint8_t step, uint16_t bpm_x100, uint32_t sample_rate_hz);

// Convenience: raw param1 byte straight to samples.
uint32_t tempo_sync_samples_from_raw(uint8_t raw, uint16_t bpm_x100, uint32_t sample_rate_hz);

// Duration of n subdivisions-per-bar units (direct count, not
// stepped/bucketed) at the given tempo. n < 1 is clamped up to 1 (no
// zero-length interval). subdivisions_per_bar is how many equal parts a
// 4/4 bar is cut into (16 = sixteenth notes, 32 = 32nd notes, ...); < 1
// is clamped up to 1 (one giant "unit" spanning the whole bar).
float tempo_sync_bar_fraction_ms(uint8_t n, uint16_t subdivisions_per_bar, uint16_t bpm_x100);

// Same, in samples at the given sample rate (rounded to nearest).
uint32_t tempo_sync_bar_fraction_samples(uint8_t n, uint16_t subdivisions_per_bar,
                                          uint16_t bpm_x100, uint32_t sample_rate_hz);

// Feedback param convention (param2): raw byte -> 0.0-FX_FEEDBACK_MAX float.
#define FX_FEEDBACK_MAX 0.95f
float fx_feedback_from_raw(uint8_t raw);

#endif // TEMPO_SYNC_H
