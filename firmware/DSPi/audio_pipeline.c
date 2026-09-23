/*
 * audio_pipeline.c - see audio_pipeline.h
 *
 * USB and I2S input are unconditionally summed together (clamped to
 * avoid immediate clipping from the sum) rather than switched between --
 * there's no control surface, vendor protocol, or other mechanism left
 * in this build to select one source over the other, and the request
 * this was built for ("audio input via usb and i2s") reads most
 * naturally as "both should be usable", not "exactly one, switchable".
 * If a source isn't sending anything, it contributes silence to the sum,
 * so this is harmless when only one is actually in use.
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

    // Each drains only as many frames as it actually has; the memsets
    // above leave the rest as silence, so a source that's idle or
    // slower than sample_count this call just contributes nothing for
    // those frames rather than stale/garbage data.
    usb_audio_drain_ring(usb_l, usb_r, sample_count);
    i2s_input_poll(i2s_l, i2s_r, sample_count);

    for (uint32_t i = 0; i < sample_count; i++) {
        // Attenuate by half (-6dB) before summing, not after: if both
        // sources are simultaneously near full-scale (0dBFS, common for
        // real digital sources -- USB audio and a hot I2S line both
        // routinely hit this), a straight sum-then-hard-clamp clips on
        // nearly every sample, which is audibly harsh, bitcrusher-like
        // digital distortion, not the occasional overs a limiter would
        // produce. Halving first means even the worst case (+1.0 and
        // +1.0) lands exactly at +1.0, needing no clipping at all in
        // ordinary use -- the safety clamp below exists only for a
        // source that's already out of [-1, 1] before it reaches here.
        float l = usb_l[i] * 0.5f + i2s_l[i] * 0.5f;
        float r = usb_r[i] * 0.5f + i2s_r[i] * 0.5f;
        if (l > 1.0f) l = 1.0f; else if (l < -1.0f) l = -1.0f;
        if (r > 1.0f) r = 1.0f; else if (r < -1.0f) r = -1.0f;
        mix_l[i] = l;
        mix_r[i] = r;
    }

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
