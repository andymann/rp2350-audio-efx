/*
 * fx_beatrepeat.h - Effect slot 5 (id 0x05): tempo-synced beat-repeat
 *
 * Always-recording design: a circular PSRAM buffer continuously captures
 * live audio, unconditionally, regardless of whether the effect is on or
 * off. The moment the effect is turned on, the most recent loop-length
 * worth of that recording is snapshotted into a second, frozen PSRAM
 * buffer and looped (sliced into param2 equal pieces, replayed in one of
 * 16 orders per param3) until it's turned back off. Turning it off then
 * on again snapshots a FRESH window of whatever's playing right then --
 * the standard beat-repeat/glitch-box behavior, and because recording
 * never stops, triggering is fast rather than needing a fill period.
 *
 * STATE MACHINE (see fx_beatrepeat.c):
 *   Every sample, unconditionally: write the current input into the
 *   circular record_buf (wrapping), advancing the write pointer. This
 *   happens whether the effect is enabled or not, and even while it's
 *   actively looping (so a quick off/on re-trigger gets fresh audio, not
 *   a repeat of the same frozen content).
 *
 *   Disabled -> enabled (rising edge): loop length (param1 x current BPM)
 *   locks in for this session, and the SNAPSHOTTING phase begins: the
 *   most recent loop_len_samples of record_buf (as of the trigger moment)
 *   get copied into loop_buf. This copy is NOT done synchronously in one
 *   block -- copying up to 768000 samples inline would itself blow the
 *   real-time audio budget and cause an audible glitch, the exact
 *   problem this redesign is trying to avoid. Instead it's spread across
 *   multiple blocks in bounded chunks (FX_BEATREPEAT_COPY_CHUNK_SAMPLES
 *   per block); live audio passes through unmodified while a snapshot is
 *   in progress. Once the copy completes, playback switches to LOOPING.
 *   param2/param3 changes still take effect at the start of the next
 *   full loop cycle once looping, same as before.
 *
 * Parameter mapping: unchanged from the previous revision -- see below,
 * copied for reference.
 *   param1  - loop length, number of 16ths of a bar (NOT fx_stutter's
 *             32nds convention). tempo_sync_bar_fraction_ms(param1, 16,
 *             bpm_x100). Clamped to FX_BEATREPEAT_MAX_SIXTEENTHS (64 = 4
 *             bars). Default 12 (0.75 bar).
 *   param2  - number of slices, clamped to FX_BEATREPEAT_MAX_SLICES (32).
 *             Default 4. Uneven division: the LAST original slice
 *             absorbs the remainder samples.
 *   param3  - playback order, 0-15. See PLAYBACK ORDERS below.
 *   dry_wet - wet mix, standard convention: 0 = fully dry, 255 = fully
 *             wet. During a SNAPSHOTTING phase (mid-copy, not yet
 *             looping), output is unmodified live audio regardless of
 *             dry_wet, same reasoning as before -- no valid wet signal
 *             exists yet.
 *
 * PLAYBACK ORDERS (param3, 0-indexed slice positions internally, N =
 * current slice count) -- unchanged from the previous revision:
 *   0  Forward (identity): 0,1,2,...,N-1
 *   1  First + reverse rest (spec example, N=4: 1,4,3,2 in 1-indexed =
 *      0,3,2,1 in 0-indexed): 0, N-1, N-2, ..., 1
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
// fx_delay. Both record_buf and loop_buf are this size (two separate
// PSRAM allocations -- see fx_beatrepeat.c).
#define FX_BEATREPEAT_MIN_PRACTICAL_BPM 60u
#define FX_BEATREPEAT_MAX_SIXTEENTHS    64u
#define FX_BEATREPEAT_MAX_SAMPLES       768000u

// Ceiling on param2 (slice count) -- judgment call, not specified.
#define FX_BEATREPEAT_MAX_SLICES 32u

// Number of distinct param3 playback orders.
#define FX_BEATREPEAT_NUM_PATTERNS 16u

// Per-block cap on how much of the record_buf -> loop_buf snapshot copy
// happens in a single fx_beatrepeat_process_block() call, so triggering
// never does the whole copy synchronously in one block (see
// fx_beatrepeat.h's top comment for why that would be a problem). At
// AUDIO_BUFFER_SAMPLES=192 (4ms blocks @ 48kHz), 8192 completes the
// default 12-sixteenth loop length in ~9 blocks (~35ms) and the maximum
// 768000-sample case in ~94 blocks (~375ms) -- both far faster than the
// old single-buffer design's up-to-full-loop-length fill delay (which,
// for that same max case, was up to 16 SECONDS).
#define FX_BEATREPEAT_COPY_CHUNK_SAMPLES 8192u

void fx_beatrepeat_init(void);

// True iff fx_beatrepeat_init() confirmed BOTH PSRAM buffers (record_buf
// and loop_buf) are actually mapped. Same purpose as fx_delay_psram_ok().
bool fx_beatrepeat_psram_ok(void);

void fx_beatrepeat_process_block(float *out_l, float *out_r, uint32_t sample_count,
                                  uint32_t sample_rate_hz);

#endif // FX_BEATREPEAT_H

