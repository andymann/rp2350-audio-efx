/*
 * fx_stutter.c - see fx_stutter.h
 */

#include "fx_stutter.h"
#include "fx_control.h"
#include "tempo_sync.h"
#include "config.h"   // DSP_TIME_CRITICAL

// Position within the current open+mute cycle, in samples: 0..(cycle_len-1)
// where cycle_len = 2 * interval_samples. Gate is open while
// cycle_pos < interval_samples, muted otherwise.
static uint32_t cycle_pos = 0;

void fx_stutter_init(void)
{
    cycle_pos = 0;
}

// RAM-resident for the same reason as fx_delay_process_block (see its
// comment): this runs in the same per-sample hot path, chained right
// after it on the same buf_out[0]/[1] channels.
DSP_TIME_CRITICAL
void fx_stutter_process_block(float *out_l, float *out_r, uint32_t sample_count,
                               uint32_t sample_rate_hz)
{
    FxState st;
    if (!fx_control_get(FX_STUTTER_EFFECT_NUM, &st) || !st.enabled) {
        return;   // slot off or unavailable: passthrough, phase keeps aging silently
    }

    uint16_t bpm_x100 = fx_control_get_bpm();
    // param1 accepts 0-63, mapping to 1-64 units (see
    // tempo_sync_clamp1_from_raw()'s doc); anything above 63 is ignored
    // (clamped down to 63, i.e. 64 units -- 2 bars at the current
    // FX_STUTTER_SUBDIVISIONS_PER_BAR=32).
    uint8_t n = tempo_sync_clamp1_from_raw(st.param1, 63u);
    uint32_t interval_samples = tempo_sync_bar_fraction_samples(
        n, FX_STUTTER_SUBDIVISIONS_PER_BAR, bpm_x100, sample_rate_hz);
    if (interval_samples < 1u) interval_samples = 1u;

    uint32_t cycle_len = interval_samples * 2u;
    if (cycle_pos >= cycle_len) {
        // Interval shrank since the last block (param1/BPM changed) and the
        // old position no longer fits the new cycle -- restart at the open
        // half rather than leaving it stuck past the end.
        cycle_pos = 0;
    }

    // Gate depth: dry_wet=255 (fully wet) mutes completely, as before;
    // dry_wet=0 (fully dry) leaves the muted half untouched (no gating
    // effect at all); values between are a partial attenuation. Only the
    // muted half is affected -- the open half always passes at full
    // level regardless of this control, same as fx_delay's dry/wet only
    // shaping its wet (delayed) contribution, not the source signal.
    float mute_dry = 1.0f - ((float)st.dry_wet / 255.0f);

    for (uint32_t i = 0; i < sample_count; i++) {
        if (cycle_pos >= interval_samples) {
            out_l[i] *= mute_dry;
            out_r[i] *= mute_dry;
        }
        cycle_pos++;
        if (cycle_pos >= cycle_len) cycle_pos = 0;
    }
}
