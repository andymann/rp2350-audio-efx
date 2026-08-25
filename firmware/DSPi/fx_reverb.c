/*
 * fx_reverb.c - see fx_reverb.h
 */

#include "fx_reverb.h"
#include "fx_control.h"
#include "tempo_sync.h"
#include "config.h"                   // DSP_TIME_CRITICAL
#include "pico/platform/sections.h"   // __uninitialized_psram()
#include "hardware/psram.h"           // psram_is_available(), psram_check_address()
#include <string.h>

// PSRAM, not SRAM (see fx_reverb.h's revision note): the tank's ~41KB
// combined footprint ate too far into the shared on-chip SRAM budget
// (roughly halved it, from ~80KB free to ~39KB), and the device failed
// to boot on real hardware after that -- the LED never lit and USB
// enumeration never happened, both consistent with a crash early in
// boot from insufficient stack headroom after that SRAM regression.
// PSRAM was already comfortably under budget (~3.4MB free out of 8MB)
// and is the proven-safe place for buffers this codebase doesn't
// strictly need SRAM-speed access for -- same reasoning fx_delay and
// fx_beatrepeat already followed for their (much larger) buffers.

// Pre-delay: circular buffer, int16 (same as fx_delay's convention) --
// this one doesn't recirculate through feedback, so a single quantize/
// dequantize round-trip per sample is fine.
static int16_t __uninitialized_psram("fx_reverb_predelay") predelay_buf[FX_REVERB_PREDELAY_MAX_SAMPLES];
static uint32_t predelay_write_pos = 0;

// Comb/allpass tank: fixed lengths, adapted from the classic Freeverb
// tuning (44100Hz -> 48000Hz scale factor 48000/44100 = 1.0884, then
// trimmed from Freeverb's 8 combs/4 allpasses down to 4/2 -- see
// fx_reverb.h's top comment for why). Every-other value kept from the
// original 8, to preserve reasonable spacing/decorrelation between the
// four that remain. Float, not int16, unlike the pre-delay buffer above:
// comb/allpass samples recirculate through feedback many times per
// second of decay, so a per-cycle int16 quantize/dequantize round-trip
// would compound quantization noise over the tail in a way a single
// non-recirculating delay tap doesn't risk.
#define FX_REVERB_COMB_LEN_0 1215u
#define FX_REVERB_COMB_LEN_1 1390u
#define FX_REVERB_COMB_LEN_2 1548u
#define FX_REVERB_COMB_LEN_3 1695u
#define FX_REVERB_ALLPASS_LEN_0 605u
#define FX_REVERB_ALLPASS_LEN_1 480u

static const uint32_t FX_REVERB_COMB_LENGTHS[FX_REVERB_NUM_COMBS] = {
    FX_REVERB_COMB_LEN_0, FX_REVERB_COMB_LEN_1, FX_REVERB_COMB_LEN_2, FX_REVERB_COMB_LEN_3
};
static const uint32_t FX_REVERB_ALLPASS_LENGTHS[FX_REVERB_NUM_ALLPASS] = {
    FX_REVERB_ALLPASS_LEN_0, FX_REVERB_ALLPASS_LEN_1
};

static float __uninitialized_psram("fx_reverb_comb0") comb_buf_0[FX_REVERB_COMB_LEN_0];
static float __uninitialized_psram("fx_reverb_comb1") comb_buf_1[FX_REVERB_COMB_LEN_1];
static float __uninitialized_psram("fx_reverb_comb2") comb_buf_2[FX_REVERB_COMB_LEN_2];
static float __uninitialized_psram("fx_reverb_comb3") comb_buf_3[FX_REVERB_COMB_LEN_3];
static float * const comb_bufs[FX_REVERB_NUM_COMBS] = {
    comb_buf_0, comb_buf_1, comb_buf_2, comb_buf_3
};
static uint32_t comb_pos[FX_REVERB_NUM_COMBS];
static float    comb_filterstore[FX_REVERB_NUM_COMBS];

static float __uninitialized_psram("fx_reverb_ap0") allpass_buf_0[FX_REVERB_ALLPASS_LEN_0];
static float __uninitialized_psram("fx_reverb_ap1") allpass_buf_1[FX_REVERB_ALLPASS_LEN_1];
static float * const allpass_bufs[FX_REVERB_NUM_ALLPASS] = {
    allpass_buf_0, allpass_buf_1
};
static uint32_t allpass_pos[FX_REVERB_NUM_ALLPASS];

static bool psram_ok = false;

void fx_reverb_init(void)
{
    psram_ok = psram_is_available() &&
               psram_check_address(&predelay_buf[FX_REVERB_PREDELAY_MAX_SAMPLES - 1]) &&
               psram_check_address(&comb_buf_0[FX_REVERB_COMB_LEN_0 - 1]) &&
               psram_check_address(&comb_buf_1[FX_REVERB_COMB_LEN_1 - 1]) &&
               psram_check_address(&comb_buf_2[FX_REVERB_COMB_LEN_2 - 1]) &&
               psram_check_address(&comb_buf_3[FX_REVERB_COMB_LEN_3 - 1]) &&
               psram_check_address(&allpass_buf_0[FX_REVERB_ALLPASS_LEN_0 - 1]) &&
               psram_check_address(&allpass_buf_1[FX_REVERB_ALLPASS_LEN_1 - 1]);
    if (!psram_ok) {
        return;   // do not touch any buffer -- see fx_delay.c's identical guard rationale
    }

    memset(predelay_buf, 0, sizeof(predelay_buf));
    predelay_write_pos = 0;

    for (uint32_t c = 0; c < FX_REVERB_NUM_COMBS; c++) {
        memset(comb_bufs[c], 0, FX_REVERB_COMB_LENGTHS[c] * sizeof(float));
        comb_pos[c] = 0;
        comb_filterstore[c] = 0.0f;
    }
    for (uint32_t a = 0; a < FX_REVERB_NUM_ALLPASS; a++) {
        memset(allpass_bufs[a], 0, FX_REVERB_ALLPASS_LENGTHS[a] * sizeof(float));
        allpass_pos[a] = 0;
    }
}

bool fx_reverb_psram_ok(void)
{
    return psram_ok;
}

// Classic damped-comb step (Freeverb topology): read the delayed sample,
// lowpass-filter it (damping) before feeding it back, write input +
// damped feedback, advance. Returns the (undamped) delayed output.
static inline float comb_process(float *buf, uint32_t len, uint32_t *pos,
                                  float *filterstore, float in,
                                  float feedback, float damp)
{
    float out = buf[*pos];
    *filterstore = out * (1.0f - damp) + (*filterstore) * damp;
    buf[*pos] = in + (*filterstore) * feedback;
    (*pos)++;
    if (*pos >= len) *pos = 0;
    return out;
}

// Classic Schroeder allpass step (same difference equation as
// fx_phaser's first-order allpass, just with a multi-sample delay line
// instead of a single-sample one, and a fixed feedback coefficient).
static inline float allpass_process(float *buf, uint32_t len, uint32_t *pos, float in)
{
    float bufout = buf[*pos];
    float out = -in + bufout;
    buf[*pos] = in + bufout * FX_REVERB_ALLPASS_FEEDBACK;
    (*pos)++;
    if (*pos >= len) *pos = 0;
    return out;
}

// RAM-resident for the same reason as the other FX process-block
// functions (see fx_delay.c's comment): shares the per-sample hot path,
// AND (now that its buffers moved to PSRAM) for the same PSRAM/flash
// QMI-bus-contention reason fx_delay's code needed it.
DSP_TIME_CRITICAL
void fx_reverb_process_block(float *out_l, float *out_r, uint32_t sample_count,
                              uint32_t sample_rate_hz)
{
    if (!psram_ok) {
        return;   // PSRAM not confirmed present/mapped -- see fx_delay.c's identical guard
    }

    FxState st;
    if (!fx_control_get(FX_REVERB_EFFECT_NUM, &st) || !st.enabled) {
        return;   // slot off or unavailable: passthrough, tank state ages silently
    }

    uint16_t bpm_x100 = fx_control_get_bpm();
    uint8_t  n = tempo_sync_clamp1_from_raw(st.param1, 63u);
    uint32_t predelay_samples = tempo_sync_bar_fraction_samples(
        n, FX_REVERB_PREDELAY_SUBDIVISIONS_PER_BAR, bpm_x100, sample_rate_hz);
    if (predelay_samples < 1u) predelay_samples = 1u;
    if (predelay_samples > FX_REVERB_PREDELAY_MAX_SAMPLES) {
        predelay_samples = FX_REVERB_PREDELAY_MAX_SAMPLES;
    }

    float feedback = FX_REVERB_FEEDBACK_MIN +
        ((float)st.param2 / 255.0f) * (FX_REVERB_FEEDBACK_MAX - FX_REVERB_FEEDBACK_MIN);
    float damp = ((float)st.param3 / 255.0f) * FX_REVERB_DAMP_MAX;

    float wet = (float)st.dry_wet / 255.0f;
    float dry = 1.0f - wet;

    for (uint32_t i = 0; i < sample_count; i++) {
        float in_l = out_l[i];
        float in_r = out_r[i];
        float mono_in = 0.5f * (in_l + in_r);

        // Pre-delay: same live-changing-length pattern as fx_delay's
        // main delay line -- read from write_pos - predelay_samples,
        // wrapping, recomputed every block so param1/BPM changes take
        // effect smoothly rather than needing a reset.
        uint32_t read_pos = (predelay_write_pos >= predelay_samples)
                                 ? (predelay_write_pos - predelay_samples)
                                 : (predelay_write_pos + FX_REVERB_PREDELAY_MAX_SAMPLES - predelay_samples);
        float predelayed = (float)predelay_buf[read_pos] * (1.0f / 32767.0f);

        float clamped_in = mono_in;
        if (clamped_in > 1.0f) clamped_in = 1.0f;
        else if (clamped_in < -1.0f) clamped_in = -1.0f;
        predelay_buf[predelay_write_pos] = (int16_t)(clamped_in * 32767.0f);
        predelay_write_pos++;
        if (predelay_write_pos >= FX_REVERB_PREDELAY_MAX_SAMPLES) predelay_write_pos = 0;

        // Parallel damped combs, summed.
        float comb_sum = 0.0f;
        for (uint32_t c = 0; c < FX_REVERB_NUM_COMBS; c++) {
            comb_sum += comb_process(comb_bufs[c], FX_REVERB_COMB_LENGTHS[c], &comb_pos[c],
                                      &comb_filterstore[c], predelayed, feedback, damp);
        }
        // Normalize by comb count so the summed tank doesn't clip before
        // the dry/wet stage -- simple, safe starting point (1/N), not a
        // precisely-tuned Freeverb-style output gain constant.
        comb_sum *= 1.0f / (float)FX_REVERB_NUM_COMBS;

        // Series allpasses for diffusion.
        float tank_out = comb_sum;
        for (uint32_t a = 0; a < FX_REVERB_NUM_ALLPASS; a++) {
            tank_out = allpass_process(allpass_bufs[a], FX_REVERB_ALLPASS_LENGTHS[a],
                                        &allpass_pos[a], tank_out);
        }

        out_l[i] = in_l * dry + tank_out * wet;
        out_r[i] = in_r * dry + tank_out * wet;
    }
}
