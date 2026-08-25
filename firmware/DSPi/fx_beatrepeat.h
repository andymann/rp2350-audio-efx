/*
 * fx_beatrepeat.h - Effect slot 5 (id 0x05): tempo-synced beat-repeat
 *
 * Captures a short window of recent audio into a PSRAM buffer the moment
 * the effect is turned on, then freezes and loops that exact window
 * (sliced into param2 equal pieces, replayed in one of 16 orders per
 * param3) until it's turned back off. Turning it off then on again
 * re-triggers a fresh capture -- "repeat whatever's playing right now",
 * the standard beat-repeat/glitch-box behavior.
 *
 * STATE MACHINE (see fx_beatrepeat.c):
 *   Disabled -> enabled (rising edge): loop length (param1 x current BPM)
 *   is locked in for this "session" and a RECORDING phase starts: live
 *   audio passes through unchanged while the buffer fills. Once full, it
 *   switches to LOOPING: the frozen buffer plays on repeat, mixed per
 *   dry_wet, until the effect is disabled. param1 changes and BPM changes
 *   only take effect on the NEXT enable-edge (loop length doesn't change
 *   mid-session); param2/param3 changes take effect at the start of the
 *   NEXT full loop cycle (not the sample they arrive on), to avoid a
 *   mid-slice jump.
 *
 *   This is a single-buffer design: there's no always-on background
 *   recorder keeping a rolling window ready before the effect is
 *   triggered, so every trigger has a brief (up to param1's full length)
 *   RECORDING fill before looping starts, during which you hear live
 *   audio, not silence. A double-buffered "instant trigger" design is
 *   possible but needs twice the PSRAM and a snapshot-copy step; this is
 *   the simpler of the two, worth revisiting if the fill delay is
 *   noticeable/undesirable in practice.
 *
 * Parameter mapping:
 *   param1  - loop length, number of 16ths of a bar (NOT the same
 *             convention as fx_stutter's param1, which is 32nds -- this
 *             effect uses sixteenths specifically, per spec). Converted
 *             via tempo_sync_bar_fraction_ms(param1, 16, bpm_x100).
 *             Clamped to FX_BEATREPEAT_MAX_SIXTEENTHS (64 = 4 bars) for
 *             buffer-sizing purposes; see FX_BEATREPEAT_MIN_PRACTICAL_BPM
 *             for the other half of that sizing assumption. Default 12
 *             (0.75 bar).
 *   param2  - number of slices the loop is split into, 1-255 clamped to
 *             FX_BEATREPEAT_MAX_SLICES (32) -- a judgment-call ceiling,
 *             not specified in the request; higher slice counts get
 *             musically noisy/glitchy fast, but there was no explicit
 *             max given. Default 4. If the loop length doesn't divide
 *             evenly by the slice count, the LAST slice (by original
 *             position, not playback order) absorbs the remainder
 *             samples so the full loop length is always covered exactly.
 *   param3  - playback order, 0-15 (clamped at the edges). 0 is normal
 *             (forward) playback. See PLAYBACK ORDERS below for all 16;
 *             pattern 1 is the one given in the original request (first
 *             slice normal, the rest played in reverse), the other 14
 *             are this implementation's own choices, since only one
 *             example was given and it needed a general rule to work for
 *             any param2 slice count, not just 4.
 *   dry_wet - wet mix, standard convention: 0 = fully dry (no audible
 *             effect regardless of param1/2/3 -- but note: during a
 *             RECORDING fill, output is ALWAYS unmodified live audio
 *             regardless of dry_wet, since there's no valid wet signal
 *             yet), 255 = fully wet (only the looped/reordered signal,
 *             once LOOPING has started).
 *
 * PLAYBACK ORDERS (param3, 0-indexed slice positions internally, N =
 * current slice count):
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
 *
 * No PSRAM/buffer needed check like fx_delay's psram_ok flag -- actually
 * IS needed here too, same reasoning: the loop buffer lives in PSRAM.
 */

#ifndef FX_BEATREPEAT_H
#define FX_BEATREPEAT_H

#include <stdint.h>
#include <stdbool.h>

#define FX_BEATREPEAT_EFFECT_NUM 5u

// Buffer sizing assumption: slowest tempo treated as "practical" (60 BPM)
// at the longest loop length allowed (64 sixteenths = 4 bars) -> 16s @
// 48kHz -> 768000 samples (same figure fx_delay's buffer landed on,
// coincidentally, via a different practical-tempo assumption). A
// param1/BPM combination requesting longer than this clamps to it rather
// than being refused, same policy as fx_delay.
#define FX_BEATREPEAT_MIN_PRACTICAL_BPM 60u
#define FX_BEATREPEAT_MAX_SIXTEENTHS    64u
#define FX_BEATREPEAT_MAX_SAMPLES       768000u

// Ceiling on param2 (slice count) -- judgment call, not specified.
#define FX_BEATREPEAT_MAX_SLICES 32u

// Number of distinct param3 playback orders.
#define FX_BEATREPEAT_NUM_PATTERNS 16u

void fx_beatrepeat_init(void);

// True iff fx_beatrepeat_init() confirmed the loop buffer's PSRAM address
// range is actually mapped. Same purpose as fx_delay_psram_ok().
bool fx_beatrepeat_psram_ok(void);

void fx_beatrepeat_process_block(float *out_l, float *out_r, uint32_t sample_count,
                                  uint32_t sample_rate_hz);

#endif // FX_BEATREPEAT_H
