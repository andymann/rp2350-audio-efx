/*
 * fx_phaser.h - Effect slot 3 (id 0x03): tempo-synced 2-notch phaser
 *
 * Classic 4-stage first-order allpass phaser (4 allpass stages = 2 notches
 * when swept -- the "Phase 90"-style topology; half that, 2 stages, would
 * be a 1-notch "Phase 45"-style design). All 4 stages share the same
 * LFO-modulated corner frequency, processed independently per channel
 * (separate allpass filter state for L/R, same shared LFO phase) for a
 * proper stereo image rather than mono-summing like fx_delay does.
 * Applied to the main S/PDIF 1 L/R pair (buf_out[0]/[1]), chained after
 * fx_stutter (slot 2) in audio_pipeline.c.
 *
 * Fixed effect characteristics (not runtime-adjustable, per spec):
 *   FX_PHASER_CENTER_HZ   - LFO sweep center frequency, 400Hz.
 *   FX_PHASER_BASE_SPREAD - LFO sweep half-range as a fraction of center,
 *                           0.35 (35%): the corner frequency swings between
 *                           400*(1-0.35)=260Hz and 400*(1+0.35)=540Hz at
 *                           full param2 depth (see param2 below).
 *
 * Parameter mapping:
 *   param1  - LFO rate, one of 14 values. ACCEPTS 0-63 over the wire
 *             (tempo_sync_clamp1_from_raw(), 0 -> value 1, 63 -> value
 *             64); only values 1-14 correspond to a real rate table
 *             entry, values 14 up through 63 (and, before the outer
 *             clamp, everything above 63) all land on the last table
 *             entry (value 14, fastest rate). Each value is the LFO's
 *             full sweep period as an exact bar fraction, computed via
 *             tempo_sync_bar_fraction_ms() against fx_control_get_bpm()
 *             -- reuses the same generic function fx_stutter uses, just
 *             with a lookup table of (n, subdivisions_per_bar) pairs
 *             instead of one fixed subdivisions_per_bar (see
 *             PHASER_RATE_TABLE in fx_phaser.c for the derivation of
 *             each pair):
 *               1: 2 bars      6: 1/4 bar (quarter)    11: 1/16 bar
 *               2: 1 bar       7: 3/16 bar (dotted 8th) 12: 3/64 bar
 *               3: 3/4 bar     8: 1/8 bar (eighth)      13: 1/24 bar (trip. 16th)
 *               4: 1/2 bar     9: 1/12 bar (trip. 8th)  14: 1/32 bar
 *               5: 3/8 bar (dotted quarter) 10: 5/64 bar
 *             Value 7 is 3/16 here, NOT the 3/4 as literally written in
 *             the original spec (which had 3/4 appear twice, at both
 *             value 3 and value 7) -- every other value strictly
 *             decreases in duration as the value number increases, and
 *             3/16 is the only reading that keeps that pattern unbroken
 *             (it also slots exactly between the quarter-note and
 *             eighth-note rates, where a "dotted eighth" belongs
 *             musically). Flagged in the commit message; trivial to
 *             correct in PHASER_RATE_TABLE if wrong.
 *   param2  - amount/depth: how strongly the LFO modulates the corner
 *             frequency, 0-255 scaled to 0.0-FX_PHASER_BASE_SPREAD. At 0,
 *             the corner frequency is pinned at FX_PHASER_CENTER_HZ (no
 *             sweep -- the allpass stages become a fixed, inaudible-ish
 *             phase-only filter, not silence). At 255, the full 35% swing
 *             specified above. Distinct from dry_wet: this controls how
 *             deep the sweep goes, dry_wet controls how much of the
 *             swept (wet) signal is heard at all.
 *   param3  - unused (reserved for future use, per spec).
 *   dry_wet - wet mix: 0 = fully dry (no audible effect), 255 = fully wet
 *             (only the "wet bus" -- see below -- is heard). NOT the same
 *             convention as fx_delay's dry_wet, which blends dry against
 *             the raw delayed signal: here the internal wet bus is
 *             already input+allpass summed (0.5*(in + allpass_cascade(in))),
 *             because that summation, not the allpass output alone, is
 *             what produces a phaser's notches (allpass stages only shift
 *             phase, not magnitude, so their output alone is spectrally
 *             flat and barely audible as an effect). This means dry_wet=255
 *             still sounds like a phaser, not a flat/uneffected signal.
 *
 * No PSRAM/buffer needed: filter state is 8 small (x1,y1) float pairs (4
 * stages x 2 channels) plus one shared LFO phase float, all in .bss.
 */

#ifndef FX_PHASER_H
#define FX_PHASER_H

#include <stdint.h>

#define FX_PHASER_EFFECT_NUM   3u
#define FX_PHASER_NUM_STAGES   4u      // 4 allpass stages = 2 notches
#define FX_PHASER_CENTER_HZ    400.0f
#define FX_PHASER_BASE_SPREAD  0.35f   // +/-35% of center at full param2 depth

// Reset filter state and LFO phase. Call once at boot before the pipeline
// starts.
void fx_phaser_init(void);

// Process sample_count samples of the main stereo pair in place. Reads
// this effect's state via fx_control_get(FX_PHASER_EFFECT_NUM, ...);
// no-op (passthrough) if that slot is disabled. Safe to call every packet
// regardless of state.
void fx_phaser_process_block(float *out_l, float *out_r, uint32_t sample_count,
                              uint32_t sample_rate_hz);

#endif // FX_PHASER_H
