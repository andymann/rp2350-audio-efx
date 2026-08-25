/*
 * fx_beatrepeat.h - Effect slot 5 (id 0x05): tempo-synced beat-repeat
 *
 * Single-buffer, direct-record design: when the effect is turned on, live
 * audio passes through unchanged while it's ALSO written directly into
 * loop_buf. The instant the buffer is full (loop length has elapsed --
 * e.g. 4 bars when param1==16), playback switches to looping that buffer
 * -- always split into a FIXED 8 equal slices (see
 * FX_BEATREPEAT_NUM_SLICES), replayed in one of 16 orders per param2 --
 * until it's turned back off. There's no separate "record" buffer and no
 * copy step: the buffer being filled IS the buffer that gets played back,
 * so the switch from filling to looping is just a phase change, not a
 * data-movement operation -- nothing to introduce latency at that moment.
 *
 * When the effect is turned off, loop_buf is emptied (zeroed) so no
 * session's captured audio lingers into a later one. This is spread
 * across blocks the same way the old snapshot copy was (see
 * FX_BEATREPEAT_CLEAR_CHUNK_SAMPLES) -- there's no urgency (the effect
 * is bypassed/passthrough while disabled regardless of clearing
 * progress), so this can safely take a little while in the background.
 *
 * STATE MACHINE (see fx_beatrepeat.c):
 *   Disabled -> enabled (rising edge): loop length (param1 x current BPM)
 *   locks in for this session, RECORDING phase begins: live audio passes
 *   through unmodified while loop_buf fills sample by sample from index
 *   0. The instant it's full, playback switches to LOOPING (same
 *   sample's audio decision, no gap): loop_buf plays on repeat, mixed
 *   per dry_wet. param2 changes take effect at the start of the next
 *   full loop cycle (not immediately), to avoid a mid-slice jump.
 *
 *   Enabled -> disabled (falling edge): if the effect was actively
 *   LOOPING, a brief FADING_OUT phase runs first (FX_BEATREPEAT_FADE_SAMPLES,
 *   ~21ms) -- ramping the wet contribution to 0 rather than cutting it
 *   instantly, since an instant switch from the wet loop signal to raw
 *   passthrough is an audible click (two unrelated waveforms with no
 *   continuity at the seam). If it was RECORDING (already plain
 *   passthrough) or already idle, there's nothing to fade, so it goes
 *   straight to CLEARING. Either way, CLEARING then zeroes loop_buf in
 *   the background (bounded chunk per block, same reasoning as the old
 *   copy's chunking -- a large synchronous memset could still cost real
 *   time on a PSRAM buffer). Output is plain passthrough throughout
 *   CLEARING, since the effect is off; clearing progress has no audible
 *   effect while disabled. If re-enabled before clearing finishes,
 *   RECORDING simply starts overwriting loop_buf from index 0 again --
 *   correct either way, since playback only ever reads the range
 *   RECORDING just finished (re)writing.
 *
 * Parameter mapping (param3 reworked -- see below; param1/param2/dry_wet
 * unchanged from the previous revision):
 *   param1  - loop length, number of 16ths of a bar (NOT fx_stutter's
 *             32nds convention). tempo_sync_bar_fraction_ms(param1, 16,
 *             bpm_x100). Clamped to FX_BEATREPEAT_MAX_SIXTEENTHS (64 = 4
 *             bars). Default 12 (0.75 bar).
 *   param2  - playback order, 0-15 (clamped at the edges). 0 is normal
 *             (forward) playback. See PLAYBACK ORDERS below. Default 0.
 *   param3  - live gain: how much of the INCOMING (pre-effect) signal is
 *             heard, independent of dry_wet. 0 = incoming audio
 *             completely blocked (only the loop is heard, per dry_wet);
 *             255 = incoming audio passed through at full level AND
 *             summed with the loop. NOT a crossfade complementary to
 *             dry_wet -- param3 and dry_wet are two independent gains on
 *             two different signals (live input vs. the loop), so both
 *             maxed means both are heard together at full level, not
 *             blended. Only applies during LOOPING; during the RECORDING
 *             fill, output is always unmodified live audio regardless of
 *             param3 (there's no loop yet for "mixed with the loop" to
 *             mean anything).
 *   dry_wet - loop gain: how much of the loop is heard, standard
 *             convention (0 = none, 255 = full). During the RECORDING
 *             fill (not yet looping), output is unmodified live audio
 *             regardless of dry_wet, same reasoning as param3 -- no
 *             valid wet signal exists yet.
 *
 * Disabling the effect while LOOPING fades param3's live gain UP to 1.0
 * (full passthrough) and dry_wet's loop gain DOWN to 0 over the same
 * FX_BEATREPEAT_FADE_SAMPLES window (see fx_beatrepeat.c's
 * PHASE_FADING_OUT) -- disabling always ends at plain, unprocessed
 * passthrough, regardless of what param3 was set to while looping.
 *
 * Every loop is split into exactly FX_BEATREPEAT_NUM_SLICES (8) equal
 * pieces. If the loop length doesn't divide evenly by 8, the LAST slice
 * (by original position, not playback order) absorbs the remainder
 * samples so the full loop length is always covered exactly.
 *
 * PLAYBACK ORDERS (param2, 0-indexed slice positions internally, N is
 * always FX_BEATREPEAT_NUM_SLICES=8):
 *   0  Forward (identity): 0,1,2,...,N-1
 *   1  First + reverse rest (spec example, N=4: 1,4,3,2 in 1-indexed =
 *      0,3,2,1 in 0-indexed; at the now-fixed N=8 this generalizes to
 *      1,8,7,6,5,4,3,2 in 1-indexed): 0, N-1, N-2, ..., 1
 *   2  Full reverse: N-1, N-2, ..., 0
 *   3  Last + forward rest: N-1, 0, 1, ..., N-2
 *   4  Adjacent pairs swapped: 1,0,3,2,5,4,... (odd N: last slice stays)
 *   5  Rotate left by 1: 1,2,...,N-1,0
 *   6  Rotate right by 1: N-1,0,1,...,N-2
 *   7  Rotate left by half (ceil(N/2)): e.g. N=4 -> 2,3,0,1
 *   8  Rotate right by half (floor(N/2)): mirrors 7 with the opposite
 *      rounding, giving mild variety at odd N
 *   9  Odd-then-even (1-indexed slice numbers: 1,3,5,... then 2,4,6,...)
 *   10 Even-then-odd (1-indexed: 2,4,6,... then 1,3,5,...)
 *   11 Riffle interleave, first half leads (classic "out-shuffle")
 *   12 Riffle interleave, second half leads (classic "in-shuffle")
 *   13 Each half reversed in place (halves stay where they are)
 *   14 Halves swapped AND each reversed
 *   15 Outside-in zigzag: 0,N-1,1,N-2,2,N-3,... (alternates ends)
 */

#ifndef FX_BEATREPEAT_H
#define FX_BEATREPEAT_H

#include <stdint.h>
#include <stdbool.h>

#define FX_BEATREPEAT_EFFECT_NUM 5u

// Buffer sizing assumption: slowest tempo treated as "practical" (60 BPM)
// at the longest loop length allowed (64 sixteenths = 4 bars) -> 16s @
// 48kHz -> 768000 samples. A param1/BPM combination requesting longer
// than this clamps to it rather than being refused, same policy as
// fx_delay. Single PSRAM allocation now (see fx_beatrepeat.c) -- the
// earlier always-recording revision needed two (a background recorder
// plus a frozen snapshot); this design only needs the one buffer that's
// both recorded into and played back from.
#define FX_BEATREPEAT_MIN_PRACTICAL_BPM 60u
#define FX_BEATREPEAT_MAX_SIXTEENTHS    64u
#define FX_BEATREPEAT_MAX_SAMPLES       768000u

// Fixed slice count -- no longer a runtime parameter.
#define FX_BEATREPEAT_NUM_SLICES 8u

// Number of distinct param2 playback orders.
#define FX_BEATREPEAT_NUM_PATTERNS 16u

// Per-block cap on how much of loop_buf gets zeroed in a single
// fx_beatrepeat_process_block() call during the background CLEARING
// phase (see fx_beatrepeat.h's top comment). Kept SMALL and conservative
// -- unlike the old snapshot-copy's chunk size (which had a genuine
// latency requirement pushing it up to 96000), clearing has NO latency
// requirement at all: it's invisible background housekeeping the whole
// time the effect is disabled/passthrough. A too-large chunk risks
// costing real time on a PSRAM buffer (unverified throughput at the
// 40MHz QMI clock) for zero user-facing benefit, so this errs toward
// safety -- full clearing at the 768000-sample maximum takes ~188 blocks
// (~750ms wall-clock) at this size, which is fine since nothing is
// waiting on it.
#define FX_BEATREPEAT_CLEAR_CHUNK_SAMPLES 4096u

// Length of the fade-to-dry ramp when the effect is disabled while
// actively LOOPING (see fx_beatrepeat.c's PHASE_FADING_OUT). An instant
// switch from the wet loop signal to raw passthrough is a discontinuity
// between two unrelated waveforms -- audible as a click/crackle. 1024
// samples (~21ms @ 48kHz) covers at least a couple of full cycles even
// for low bass content (e.g. 100Hz's period is 480 samples -- a much
// shorter fade can cut off mid-cycle and still leave an audible
// transient) while still being short enough not to read as a deliberate
// "fade" to the ear. Not needed on any other transition: disabling
// during RECORDING, or the rising edge itself, are both already
// passthrough-to-passthrough with nothing discontinuous to smooth.
#define FX_BEATREPEAT_FADE_SAMPLES 1024u

void fx_beatrepeat_init(void);

// True iff fx_beatrepeat_init() confirmed loop_buf's PSRAM address range
// is actually mapped. Same purpose as fx_delay_psram_ok().
bool fx_beatrepeat_psram_ok(void);

void fx_beatrepeat_process_block(float *out_l, float *out_r, uint32_t sample_count,
                                  uint32_t sample_rate_hz);

#endif // FX_BEATREPEAT_H
