/*
 * limiter.h - Output brickwall limiter
 *
 * The very last DSP stage before the float -> 24-in-32 integer conversion
 * in audio_pipeline.c: after the whole FX chain, so it also catches overs
 * produced by the effects themselves (fx_delay/fx_reverb feedback,
 * fx_beatrepeat's loop + live sum, fx_djfilter resonance), which the
 * Volume Leveller -- sitting BEFORE the FX chain, with only a -3dBFS cap
 * on its own boost -- cannot see. Without this stage, anything above
 * 1.0 reaching the output is hard-clipped by the final conversion clamp.
 *
 * Not a port: the original DSPi firmware has no standalone output limiter
 * (only the leveller's internal boost cap), so this is written fresh.
 *
 * Algorithm: feedforward, stereo-linked, peak-sensing lookahead limiter
 * with a guaranteed no-overshoot ceiling (sample peaks, not true-peak):
 *   1. Per sample, the required total gain for the INCOMING sample:
 *        target = min(input_gain, ceiling / max(|L|, |R|))
 *   2. Sliding-window minimum of target over the last B+1 samples
 *      (monotonic deque, O(1) amortised).
 *   3. Release: the envelope drops instantly to that minimum, recovers
 *      toward it exponentially with the configured release time.
 *   4. Box (moving-average) filter of length B over the envelope -- this
 *      IS the attack: a linear-in-time ramp spread across the lookahead.
 *   5. Output = input delayed by B samples, times the box-filter output.
 *   With delay D = B and window W = B + 1, every envelope value inside
 *   the box's window is <= the target of the sample exiting the delay
 *   line at that moment, so their average is too: the ceiling can never
 *   be exceeded (up to float rounding), with no hard clip anywhere.
 *
 * Input gain is folded into the gain computer (step 1) rather than
 * applied to the samples up front, so gain changes are smoothed by the
 * same attack/release path (decreases over the 2ms lookahead, increases
 * at the release rate) -- no zipper noise, and no overshoot while the
 * gain is moving.
 *
 * Latency: B = 2ms (192 samples at 96kHz, exactly one AUDIO_BUFFER_SAMPLES
 * block) when enabled, 0 when bypassed. Enable/disable crossfades over one
 * block (see limiter.c) rather than jumping between the delayed and
 * undelayed signal.
 *
 * Control: Set Limiter / Query Limiter commands (0x0B/0x0C, fx_control.h).
 * Not touched by Disable All (0x06) -- that command only affects the 8 FX
 * slots, same as it leaves the leveller alone.
 */

#ifndef LIMITER_H
#define LIMITER_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

// 2ms lookahead/attack at this build's fixed SAMPLE_RATE_HZ (config.h).
#define LIMITER_LOOKAHEAD_SAMPLES  ((SAMPLE_RATE_HZ * 2u) / 1000u)

// Wire-format parameter scaling (fx_control.h, Set Limiter 0x0B). Config
// is stored as the raw wire bytes, so Query Limiter round-trips exactly.
//   ceiling:    byte * 0.1 dB below 0dBFS    -> 0.0 .. -25.5 dBFS
//   release:    10 + byte * 4 ms             -> 10 .. 1030 ms
//   input_gain: byte * 0.1 dB of boost       -> 0.0 .. +25.5 dB
#define LIMITER_RELEASE_MS_BASE    10u
#define LIMITER_RELEASE_MS_STEP     4u

// Factory defaults (raw wire bytes). Disabled by default, matching the
// leveller's convention: a genuine no-op bypass until explicitly enabled,
// so adding this stage changes nothing about existing behaviour.
#define LIMITER_DEFAULT_ENABLED     false
#define LIMITER_DEFAULT_CEILING     10u    // -1.0 dBFS
#define LIMITER_DEFAULT_RELEASE     23u    // 10 + 23*4 = 102 ms
#define LIMITER_DEFAULT_INPUT_GAIN   0u    // 0.0 dB

// Call once at boot, before core 1 launches.
void limiter_init(void);

// Process count stereo samples in l/r in-place. Core 1 only (audio hot
// path, RAM-resident). Must be the last stage before output conversion.
void limiter_process_block(float *l, float *r, uint32_t count);

// Set Limiter command (0x0B, fx_control.c, core 0). Raw wire bytes.
void limiter_set_config(bool enabled, uint8_t ceiling, uint8_t release,
                        uint8_t input_gain);

// Query Limiter command (0x0C, fx_control.c, core 0). Raw wire bytes.
void limiter_get_config(bool *enabled, uint8_t *ceiling, uint8_t *release,
                        uint8_t *input_gain);

#endif // LIMITER_H
