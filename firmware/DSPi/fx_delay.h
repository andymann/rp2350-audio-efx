/*
 * fx_delay.h - Effect slot 0: tempo-synced feedback delay
 *
 * The first effect wired from fx_control's control-plane state into the
 * actual audio path. Applied to the main S/PDIF 1 L/R pair (buf_out[0]/[1])
 * as a mono-summed feedback delay: both channels feed the same delay line
 * and receive the same wet signal back (a simple, symmetric echo rather
 * than a stereo ping-pong).
 *
 * Parameter mapping (see tempo_sync.h for the shared convention):
 *   param1  - time division, 1-16 steps (raw byte bucketed via
 *             tempo_sync_step_from_raw), converted to samples via
 *             tempo_sync_samples() against fx_control_get_bpm().
 *   param2  - feedback, raw byte scaled via fx_feedback_from_raw()
 *             (0.0-0.95).
 *   dry_wet - standard FxState field (0-255), used as the wet mix amount.
 *   param3  - unused by this effect.
 *
 * BUFFER SIZE CAP: the delay line is a fixed-size int16 SRAM buffer, not
 * PSRAM (this repo has no PSRAM driver/linker region set up yet -- see the
 * RP2350 reverb work in an earlier session for what that involves). At
 * build time this firmware has ~80KB of SRAM free after everything else
 * (memmap_dspi_rp2350_xip.ld: 512KB RAM total); FX_DELAY_MAX_SAMPLES is
 * sized to fit comfortably inside that with headroom for stack/heap.
 *
 * A requested delay (time division x BPM) longer than the cap is silently
 * CLAMPED to the cap rather than refused, so the effect is always audible,
 * just shorter than the musical value at slow tempi / long divisions. At
 * 48kHz the cap is ~600ms; it halves at 96kHz since the cap is fixed in
 * samples, not milliseconds. Moving this to a PSRAM-backed buffer would
 * remove the cap.
 */

#ifndef FX_DELAY_H
#define FX_DELAY_H

#include <stdint.h>

// Fixed in samples (not ms) so the cap is sample-rate-independent code-wise;
// the time it represents shrinks at higher sample rates. 28800 samples =
// 600ms @ 48kHz / 300ms @ 96kHz. int16 mono: 28800 * 2 bytes = 56.25KB.
#define FX_DELAY_MAX_SAMPLES 28800u

// Zero the delay buffer. Call once at boot before the pipeline starts.
void fx_delay_init(void);

// Process sample_count samples of the main stereo pair in place. Reads
// effect slot 0's state via fx_control_get(0, ...); no-op (passthrough) if
// that slot is disabled. Safe to call every packet regardless of state.
void fx_delay_process_block(float *out_l, float *out_r, uint32_t sample_count,
                             uint32_t sample_rate_hz);

#endif // FX_DELAY_H
