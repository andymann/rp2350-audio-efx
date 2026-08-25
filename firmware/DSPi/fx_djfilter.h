/*
 * fx_djfilter.h - Effect slot 4 (id 0x04): DJ-style sweep filter
 *
 * A single cutoff/mode knob (param1) that's a low-pass below center and a
 * high-pass above it -- the classic DJ mixer filter-knob behavior. Self-
 * contained 2nd-order (12dB/oct) resonant biquad, RBJ cookbook formulas,
 * same tan/cos-based coefficient style as fx_phaser.c -- NOT the shared
 * Filter/FilterType engine in dsp_biquad.h/config.h, which is tied into
 * the persistent PEQ/crossover subsystem's flash storage and wire-protocol
 * versioning, a different concern from this lightweight FX chain. Applied
 * to the main S/PDIF 1 L/R pair (buf_out[0]/[1]), chained after fx_phaser
 * (slot 3) in audio_pipeline.c, independent per-channel filter state (own
 * x1/x2/y1/y2 history for L/R, coefficients shared since both channels
 * use the same cutoff/Q).
 *
 * Parameter mapping:
 *   param1  - cutoff/mode, 0-255. 127 is an exact bypass (no filtering at
 *             all, not just a very wide-open one) -- guaranteed by an
 *             explicit early-out, not by the filter math happening to
 *             land somewhere inaudible.
 *               0..126 : low-pass. Cutoff sweeps LOGARITHMICALLY (matches
 *                        how a real sweep pot/ear perceives frequency)
 *                        from FX_DJFILTER_LP_OPEN_HZ (127, wide open, ~ no
 *                        audible filtering) down to FX_DJFILTER_LP_CLOSED_HZ
 *                        (0, bass-only) as param1 falls from 126 to 0.
 *               128..255 : high-pass. Cutoff sweeps logarithmically from
 *                        FX_DJFILTER_HP_OPEN_HZ (128, wide open) up to
 *                        FX_DJFILTER_HP_CLOSED_HZ (255, treble-only) as
 *                        param1 rises from 128 to 255.
 *   param2  - resonance/Q, 0-255 scaled to FX_DJFILTER_Q_MIN..
 *             FX_DJFILTER_Q_MAX (0.707 = flat/Butterworth, no peak; 8.0 =
 *             pronounced resonant peak at the cutoff -- the "acid sweep"
 *             character DJ filters are known for). Not specified by the
 *             original request ("no idea what to do with parameter 2");
 *             this is a judgment call, easy to change if it's not what's
 *             wanted.
 *   param3  - unused, per spec.
 *   dry_wet - wet mix, standard convention: 0 = fully dry (no audible
 *             effect regardless of param1), 255 = fully wet (only the
 *             filtered signal). Unlike fx_phaser, no internal wet-bus
 *             summation is needed here -- a biquad's OWN output already
 *             has the LP/HP magnitude response baked in (it doesn't need
 *             summing with the dry signal to have any effect the way an
 *             allpass-only signal does), so this is the same simple
 *             convention fx_delay uses.
 *
 * Coefficients are recomputed once per block (not per sample like
 * fx_phaser's continuously-LFO-swept ones) -- param1/param2 only change
 * via discrete Set FX commands, not continuous modulation within a block,
 * so there's no need to pay the cos/sin/tan cost every sample here.
 */

#ifndef FX_DJFILTER_H
#define FX_DJFILTER_H

#include <stdint.h>

#define FX_DJFILTER_EFFECT_NUM 4u

#define FX_DJFILTER_LP_OPEN_HZ    18000.0f   // param1=126: ~no audible LP filtering
#define FX_DJFILTER_LP_CLOSED_HZ     30.0f   // param1=0: bass-only
#define FX_DJFILTER_HP_OPEN_HZ       30.0f   // param1=128: ~no audible HP filtering
#define FX_DJFILTER_HP_CLOSED_HZ  18000.0f   // param1=255: treble-only

#define FX_DJFILTER_Q_MIN  0.707f   // Butterworth, flat, no resonant peak
#define FX_DJFILTER_Q_MAX  8.0f     // pronounced resonant peak at the cutoff

// Reset filter state. Call once at boot before the pipeline starts.
void fx_djfilter_init(void);

// Process sample_count samples of the main stereo pair in place. Reads
// this effect's state via fx_control_get(FX_DJFILTER_EFFECT_NUM, ...);
// no-op (passthrough) if that slot is disabled. Safe to call every packet
// regardless of state.
void fx_djfilter_process_block(float *out_l, float *out_r, uint32_t sample_count,
                                uint32_t sample_rate_hz);

#endif // FX_DJFILTER_H
