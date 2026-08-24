/*
 * fx_delay.c - see fx_delay.h
 */

#include "fx_delay.h"
#include "fx_control.h"
#include "tempo_sync.h"
#include "config.h"                   // DSP_TIME_CRITICAL
#include "pico/platform/sections.h"   // __uninitialized_psram()
#include <string.h>

static int16_t __uninitialized_psram("fx_delay") delay_buf[FX_DELAY_MAX_SAMPLES];
static uint32_t write_idx = 0;

void fx_delay_init(void)
{
    memset(delay_buf, 0, sizeof(delay_buf));
    write_idx = 0;
}

// RAM-resident (not just RP2350's -O3 DSP-hot-file convention): on RP2350,
// PSRAM and flash share the same physical QMI bus (different chip
// selects). If this function's own *code* stayed in flash and ran via the
// XIP cache, its instruction fetches would compete with the PSRAM *data*
// traffic this function is doing every sample -- an earlier session hit
// exactly this as one half of a two-part cause of audible noise on real
// RP2350B+PSRAM hardware (the other half was too-fast a PSRAM clock; see
// firmware/CMakeLists.txt's PICO_DEFAULT_PSRAM_MAX_FREQ comment). The
// whole call chain from here down (fx_control_get/fx_control_get_bpm,
// tempo_sync_*, fx_feedback_from_raw) is marked DSP_TIME_CRITICAL for the
// same reason -- any link still fetching from flash reintroduces the race.
DSP_TIME_CRITICAL
void fx_delay_process_block(float *out_l, float *out_r, uint32_t sample_count,
                             uint32_t sample_rate_hz)
{
    FxState st;
    if (!fx_control_get(0, &st) || !st.enabled) {
        return;   // slot 0 off or unavailable: passthrough, buffer keeps aging silently
    }

    uint16_t bpm_x100 = fx_control_get_bpm();
    uint8_t  step = tempo_sync_step_from_raw(st.param1);
    uint32_t delay_samples = tempo_sync_samples(step, bpm_x100, sample_rate_hz);

    if (delay_samples < 1u) delay_samples = 1u;
    if (delay_samples > FX_DELAY_MAX_SAMPLES) delay_samples = FX_DELAY_MAX_SAMPLES;

    float feedback = fx_feedback_from_raw(st.param2);
    float wet = (float)st.dry_wet / 255.0f;
    float dry = 1.0f - wet;

    for (uint32_t i = 0; i < sample_count; i++) {
        uint32_t read_idx = (write_idx >= delay_samples)
                                 ? (write_idx - delay_samples)
                                 : (write_idx + FX_DELAY_MAX_SAMPLES - delay_samples);

        float delayed = (float)delay_buf[read_idx] * (1.0f / 32767.0f);

        float mono_in = 0.5f * (out_l[i] + out_r[i]);
        float to_store = mono_in + delayed * feedback;
        if (to_store > 1.0f) to_store = 1.0f;
        else if (to_store < -1.0f) to_store = -1.0f;
        delay_buf[write_idx] = (int16_t)(to_store * 32767.0f);

        out_l[i] = out_l[i] * dry + delayed * wet;
        out_r[i] = out_r[i] * dry + delayed * wet;

        write_idx++;
        if (write_idx >= FX_DELAY_MAX_SAMPLES) write_idx = 0;
    }
}
