/*
 * fx_reverb.h - Effect slot 1 (id 0x01): tempo-synced pre-delay reverb
 *
 * Classic Schroeder/Freeverb-style topology: N parallel damped comb
 * filters summed together, feeding M series allpass filters for
 * diffusion, preceded by a short tempo-synced pre-delay. Mono-summed
 * processing (same convention as fx_delay/fx_stutter/fx_beatrepeat),
 * output applied identically to both channels.
 *
 * Unlike fx_delay/fx_stutter/fx_beatrepeat, every buffer here lives in
 * on-chip SRAM, not PSRAM -- comb/allpass/pre-delay lines for a reverb
 * are all short enough (tens to low hundreds of ms) to fit comfortably,
 * unlike those three effects' much longer buffers. Trimmed to 4 combs +
 * 2 allpasses (half the classic Freeverb 8+4) specifically to keep this
 * effect's SRAM footprint modest -- SRAM is a tight, shared budget
 * across the whole FX chain (~80KB was free before this effect; see
 * fx_delay.h's history for why fx_delay/fx_stutter/fx_beatrepeat all
 * had to move to PSRAM instead of using it). A full 8+4 tank would
 * roughly double this effect's ~41KB SRAM usage for a modestly denser
 * tail; trading some of that density for headroom felt like the right
 * call for a first cut -- doubling FX_REVERB_NUM_COMBS/NUM_ALLPASS and
 * extending the tuning tables is the natural way to revisit that later.
 *
 * Parameter mapping:
 *   param1  - pre-delay, tempo-synced like every other effect's param1
 *             in this chain, but HARD-CAPPED in absolute time regardless
 *             of BPM (FX_REVERB_PREDELAY_MAX_MS = 150ms) -- unlike
 *             fx_delay/fx_stutter/fx_beatrepeat's buffers, this one
 *             can't be allowed to scale unboundedly with a slow-BPM edge
 *             case, since it has to stay small enough for SRAM. Accepts
 *             0-63 (tempo_sync_clamp1_from_raw(), 0 -> 1 unit, 63 -> 64
 *             units) of FX_REVERB_PREDELAY_SUBDIVISIONS_PER_BAR
 *             (128ths); anything above 63 is ignored (clamped down), and
 *             the resulting time is separately clamped to the 150ms
 *             ceiling regardless of BPM.
 *   param2  - decay/size: comb feedback, 0-255 scaled to
 *             FX_REVERB_FEEDBACK_MIN..MAX (0.70 = short/small room decay,
 *             0.98 = long/huge hall decay). NOT tempo-synced -- reverb
 *             decay is conventionally a continuous "small room -> huge
 *             hall" sweep on real hardware/plugins, not a bars/beats
 *             value, so forcing tempo-sync here would fight how people
 *             actually expect to dial in reverb size.
 *   param3  - damping: one-pole lowpass coefficient in each comb's
 *             feedback path, 0-255 scaled to 0.0 (bright/undamped,
 *             metallic-leaning tail) .. FX_REVERB_DAMP_MAX (dark/heavily
 *             damped, warm/natural-leaning tail).
 *   dry_wet - wet mix, standard convention (same as fx_delay's): 0 =
 *             fully dry, 255 = fully wet (only the reverb tank's
 *             output). Unlike fx_phaser, no internal wet-bus summation
 *             is needed -- a properly-tuned reverb tank's own output
 *             already has full reverb character at 100% wet, it doesn't
 *             need summing with the dry signal the way an allpass-only
 *             signal does.
 */

#ifndef FX_REVERB_H
#define FX_REVERB_H

#include <stdint.h>

#define FX_REVERB_EFFECT_NUM 1u

// Pre-delay sizing (see fx_reverb.h's top comment for the SRAM-vs-PSRAM
// reasoning). Fixed in samples (not adjusted for sample rate), matching
// the precedent set by fx_delay/fx_stutter/fx_beatrepeat's PSRAM
// buffers: the same sample count represents less absolute time at
// higher sample rates (e.g. 96kHz), a known/accepted tradeoff already
// established elsewhere in this codebase, not a new limitation.
#define FX_REVERB_PREDELAY_SUBDIVISIONS_PER_BAR 128u
#define FX_REVERB_PREDELAY_MAX_MS      150u
#define FX_REVERB_PREDELAY_MAX_SAMPLES 7200u   // 150ms @ 48kHz

// Tank topology. Comb/allpass delay-line lengths are declared in
// fx_reverb.c (FX_REVERB_COMB_LENGTHS / FX_REVERB_ALLPASS_LENGTHS),
// adapted from the classic Freeverb tuning (scaled from 44100Hz to
// 48000Hz, then trimmed from 8/4 down to 4/2 -- see this file's top
// comment).
#define FX_REVERB_NUM_COMBS    4u
#define FX_REVERB_NUM_ALLPASS  2u

// Comb feedback range (param2): 0.70 (short decay) .. 0.98 (long decay).
// Matches classic Freeverb's roomsize -> feedback mapping range.
#define FX_REVERB_FEEDBACK_MIN 0.70f
#define FX_REVERB_FEEDBACK_MAX 0.98f

// Damping range (param3): 0.0 (bright/undamped) .. this max (dark).
#define FX_REVERB_DAMP_MAX 0.90f

// Allpass feedback is a fixed classic-Schroeder constant, not a runtime
// parameter -- real reverb designs don't expose this as a user knob.
#define FX_REVERB_ALLPASS_FEEDBACK 0.5f

void fx_reverb_init(void);

void fx_reverb_process_block(float *out_l, float *out_r, uint32_t sample_count,
                              uint32_t sample_rate_hz);

#endif // FX_REVERB_H
