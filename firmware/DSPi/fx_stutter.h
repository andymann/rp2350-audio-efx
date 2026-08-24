/*
 * fx_stutter.h - Effect slot 2 (id 0x02): tempo-synced stutter/gate
 *
 * Cuts the audio off in controlled intervals: passes through for one
 * time-division interval, mutes for the next, repeating -- a hard 50%-duty
 * gate locked to tempo, not a crossfaded one (matches "cutting off audio",
 * not "fading audio"). Applied to the main S/PDIF 1 L/R pair
 * (buf_out[0]/[1]), same as fx_delay (slot 0), chained after it in
 * audio_pipeline.c so slot ordering by effect_num matches processing
 * order.
 *
 * Slot id 1 is intentionally left unassigned for now (see fx_control.h's
 * slot registry comment).
 *
 * Parameter mapping (see tempo_sync.h for the two conventions it offers --
 * this effect uses the sixteenth-note one, NOT the same one fx_delay uses):
 *   param1  - time division, a direct count of sixteenth notes (1-255,
 *             via tempo_sync_sixteenth_samples() against
 *             fx_control_get_bpm()) -- param1=1 means 1/16 of a bar,
 *             param1=16 means a full bar, param1=32 means 2 bars. This is
 *             the length of BOTH the open and the muted half of the
 *             cycle -- param1=1 means 1/16 bar passes through, then 1/16
 *             bar is silent, repeating (a full cycle is 2/16 bar).
 *             Finer resolution than fx_delay's 16-step scheme on purpose:
 *             that scheme's shortest interval is a quarter note, too
 *             coarse for a stutter gate.
 *   param2  - unused by this effect (reserved for future use).
 *   param3  - unused by this effect (reserved for future use).
 *   dry_wet - gate depth: 255 (fully wet) mutes the closed half completely
 *             (silence), 0 (fully dry) leaves it untouched (no gating
 *             effect at all), values between are a partial attenuation.
 *             The open half is always unaffected regardless of this
 *             control.
 *
 * No PSRAM/buffer needed (unlike fx_delay): this is pure sample counting,
 * so there's no delay_buf-style hardware-availability guard to check.
 *
 * Phase state (cycle_pos) is free-running from fx_stutter_init() at boot,
 * not reset when the effect is toggled on/off or when param1 changes --
 * changing the interval mid-cycle can shift the gate boundary by up to
 * one interval on that transition (an audible but one-off discontinuity),
 * same tradeoff fx_delay makes when its delay time changes live.
 */

#ifndef FX_STUTTER_H
#define FX_STUTTER_H

#include <stdint.h>

#define FX_STUTTER_EFFECT_NUM 2u

// Reset phase state. Call once at boot before the pipeline starts.
void fx_stutter_init(void);

// Process sample_count samples of the main stereo pair in place. Reads
// this effect's state via fx_control_get(FX_STUTTER_EFFECT_NUM, ...);
// no-op (passthrough) if that slot is disabled. Safe to call every packet
// regardless of state.
void fx_stutter_process_block(float *out_l, float *out_r, uint32_t sample_count,
                               uint32_t sample_rate_hz);

#endif // FX_STUTTER_H
