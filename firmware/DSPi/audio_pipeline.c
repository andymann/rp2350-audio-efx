/*
 * audio_pipeline.c - see audio_pipeline.h
 *
 * Exactly one input source is ever active at a time -- matches the
 * original DSPi's own single-active-input-source model, not a
 * simultaneous mix of USB and I2S. Selected via the Set Input Source
 * command (0x08, fx_control.h); defaults to USB at boot. An earlier
 * revision of this file unconditionally summed both sources together
 * (with dynamic per-source gain to avoid ducking a source when the
 * other was silent) -- reverted in favor of this simpler, original-
 * firmware-matching model once the request became explicit: a single,
 * user-selected source, not an automatic mix.
 */

#include "audio_pipeline.h"
#include "config.h"
#include "usb_audio.h"
#include "i2s_input.h"
#include "fx_control.h"
#include "fx_delay.h"
#include "fx_reverb.h"
#include "fx_stutter.h"
#include "fx_phaser.h"
#include "fx_djfilter.h"
#include "fx_beatrepeat.h"

#include <string.h>

static float usb_l[AUDIO_BUFFER_SAMPLES];
static float usb_r[AUDIO_BUFFER_SAMPLES];
static float i2s_l[AUDIO_BUFFER_SAMPLES];
static float i2s_r[AUDIO_BUFFER_SAMPLES];
static float mix_l[AUDIO_BUFFER_SAMPLES];
static float mix_r[AUDIO_BUFFER_SAMPLES];

void audio_pipeline_fill_block(int32_t *out_stereo, uint32_t sample_count,
                                uint32_t sample_rate_hz)
{
    if (sample_count > AUDIO_BUFFER_SAMPLES) sample_count = AUDIO_BUFFER_SAMPLES;

    // Restart Clock command (0x07, fx_control.h): resets rhythmic
    // effects' phase back to their start-of-cycle position. Checked
    // once per block, here rather than in fx_control.c, because this is
    // core 1 (where fx_stutter/fx_phaser's state is actually touched);
    // fx_control_poll() -- which sets this flag -- runs on core 0.
    if (fx_control_clock_restart_requested()) {
        fx_stutter_reset_phase();
        fx_phaser_reset_phase();
        fx_control_clock_restart_ack();
    }

    memset(usb_l, 0, sample_count * sizeof(float));
    memset(usb_r, 0, sample_count * sizeof(float));
    memset(i2s_l, 0, sample_count * sizeof(float));
    memset(i2s_r, 0, sample_count * sizeof(float));

    // Both are drained every block regardless of which is actually
    // selected -- USB packets and I2S capture both keep arriving
    // whether or not their source is the active one, and draining
    // whichever isn't selected here still prevents its own ring from
    // backing up/overflowing in the meantime, so switching sources
    // later doesn't start from a backlog of stale data.
    usb_audio_drain_ring(usb_l, usb_r, sample_count);
    i2s_input_poll(i2s_l, i2s_r, sample_count);

    // Exactly one source reaches the mix -- see this file's top comment.
    const float *src_l = (fx_control_get_input_source() == FX_INPUT_SOURCE_I2S)
                         ? i2s_l : usb_l;
    const float *src_r = (fx_control_get_input_source() == FX_INPUT_SOURCE_I2S)
                         ? i2s_r : usb_r;
    memcpy(mix_l, src_l, sample_count * sizeof(float));
    memcpy(mix_r, src_r, sample_count * sizeof(float));

    // FX chain, in ascending effect_num order (fx_control.h's slot
    // registry) -- slot 1 (reverb) chained after slot 0 (delay), etc.
    // Slot indices with no effect assigned (currently none, all of 0-5
    // are in use) are simply absent from this list.
    fx_delay_process_block(mix_l, mix_r, sample_count, sample_rate_hz);
    fx_reverb_process_block(mix_l, mix_r, sample_count, sample_rate_hz);
    fx_stutter_process_block(mix_l, mix_r, sample_count, sample_rate_hz);
    fx_phaser_process_block(mix_l, mix_r, sample_count, sample_rate_hz);
    fx_djfilter_process_block(mix_l, mix_r, sample_count, sample_rate_hz);
    fx_beatrepeat_process_block(mix_l, mix_r, sample_count, sample_rate_hz);

    // Float [-1, 1] -> 24-bit-in-low-bits signed integer, the producer
    // format pico_audio_i2s_multi's audio_i2s_connect_extra() expects
    // (see i2s_output.h's top comment) -- it left-shifts by 8 to build
    // the final MSB-aligned 32-bit I2S frame itself.
    for (uint32_t i = 0; i < sample_count; i++) {
        float l = mix_l[i] * 8388607.0f;
        float r = mix_r[i] * 8388607.0f;
        if (l > 8388607.0f) l = 8388607.0f; else if (l < -8388608.0f) l = -8388608.0f;
        if (r > 8388607.0f) r = 8388607.0f; else if (r < -8388608.0f) r = -8388608.0f;
        out_stereo[2 * i]     = (int32_t)l;
        out_stereo[2 * i + 1] = (int32_t)r;
    }
}
