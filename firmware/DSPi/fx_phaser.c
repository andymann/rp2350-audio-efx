/*
 * fx_phaser.c - see fx_phaser.h
 */

#include "fx_phaser.h"
#include "fx_control.h"
#include "tempo_sync.h"
#include "config.h"   // DSP_TIME_CRITICAL
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f   // same fallback convention as crossover.c
#endif

typedef struct {
    float x1, y1;   // first-order allpass state: previous input/output
} ApState;

static ApState stages_l[FX_PHASER_NUM_STAGES];
static ApState stages_r[FX_PHASER_NUM_STAGES];
static float   lfo_phase = 0.0f;   // radians, wraps at 2*pi

// One (n, subdivisions_per_bar) pair per param1 value (index = value - 1),
// each giving the LFO's full sweep period as an exact bar fraction via
// tempo_sync_bar_fraction_ms(n, sub, bpm_x100). See fx_phaser.h's param1
// doc for the musical name and bar-fraction of every entry, and the note
// on value 7's correction from the original spec's literal "3/4"
// (duplicate of value 3) to "3/16".
typedef struct {
    uint8_t  n;
    uint16_t sub;
} RateEntry;

#define PHASER_RATE_STEPS 14u

static const RateEntry PHASER_RATE_TABLE[PHASER_RATE_STEPS] = {
    { 8,  4},   // 1: 2 bars
    { 4,  4},   // 2: 1 bar
    { 3,  4},   // 3: 3/4 bar
    { 2,  4},   // 4: 1/2 bar
    { 3,  8},   // 5: 3/8 bar (dotted quarter)
    { 1,  4},   // 6: 1/4 bar (quarter)
    { 3, 16},   // 7: 3/16 bar (dotted eighth) -- corrected, see fx_phaser.h
    { 1,  8},   // 8: 1/8 bar (eighth)
    { 1, 12},   // 9: 1/12 bar (triplet eighth)
    { 5, 64},   // 10: 5/64 bar
    { 1, 16},   // 11: 1/16 bar
    { 3, 64},   // 12: 3/64 bar
    { 1, 24},   // 13: 1/24 bar (triplet sixteenth)
    { 1, 32},   // 14: 1/32 bar
};

void fx_phaser_init(void)
{
    for (uint32_t s = 0; s < FX_PHASER_NUM_STAGES; s++) {
        stages_l[s].x1 = stages_l[s].y1 = 0.0f;
        stages_r[s].x1 = stages_r[s].y1 = 0.0f;
    }
    lfo_phase = 0.0f;
}

static inline float allpass_process(ApState *st, float in, float a)
{
    float out = a * in + st->x1 - a * st->y1;
    st->x1 = in;
    st->y1 = out;
    return out;
}

// RAM-resident for the same reason as fx_delay_process_block/
// fx_stutter_process_block (see fx_delay.c's comment): shares the
// per-sample hot path, chained after slot 2. sinf()/tanf() below are
// pico_float functions, already forced RAM-resident project-wide via
// PICO_FLOAT_IN_RAM=1 (CMakeLists.txt), so no separate guard needed for
// those calls specifically.
DSP_TIME_CRITICAL
void fx_phaser_process_block(float *out_l, float *out_r, uint32_t sample_count,
                              uint32_t sample_rate_hz)
{
    FxState st;
    if (!fx_control_get(FX_PHASER_EFFECT_NUM, &st) || !st.enabled) {
        return;   // slot off or unavailable: passthrough, filter state/phase keep aging silently
    }

    uint16_t bpm_x100 = fx_control_get_bpm();

    uint8_t rate_step = st.param1;
    if (rate_step < 1u) rate_step = 1u;
    if (rate_step > PHASER_RATE_STEPS) rate_step = (uint8_t)PHASER_RATE_STEPS;
    const RateEntry *rate = &PHASER_RATE_TABLE[rate_step - 1u];

    float period_ms = tempo_sync_bar_fraction_ms(rate->n, rate->sub, bpm_x100);
    if (period_ms < 1.0f) period_ms = 1.0f;   // guard an absurdly fast/zero period
    float lfo_freq_hz = 1000.0f / period_ms;
    float phase_inc = 2.0f * (float)M_PI * lfo_freq_hz / (float)sample_rate_hz;

    float depth = ((float)st.param2 / 255.0f) * FX_PHASER_BASE_SPREAD;   // 0..0.35
    float wet   = (float)st.dry_wet / 255.0f;
    float dry   = 1.0f - wet;

    for (uint32_t i = 0; i < sample_count; i++) {
        float lfo = sinf(lfo_phase);
        float fc  = FX_PHASER_CENTER_HZ * (1.0f + depth * lfo);

        // Guard fc into a sane range before the tan() prewarp below --
        // shouldn't be reachable at depth<=0.35/center=400Hz, but a bad
        // param2/BPM combination should degrade gracefully, not produce
        // a NaN/unstable allpass coefficient.
        if (fc < 20.0f) fc = 20.0f;
        if (fc > (float)sample_rate_hz * 0.45f) fc = (float)sample_rate_hz * 0.45f;

        // Standard bilinear-transform first-order allpass coefficient
        // (same tan-prewarp pattern as crossover.c/dsp_pipeline.c's
        // filters, just recomputed every sample here since fc is
        // continuously LFO-swept rather than fixed).
        float w = tanf((float)M_PI * fc / (float)sample_rate_hz);
        float a = (w - 1.0f) / (w + 1.0f);

        float wet_l = out_l[i];
        float wet_r = out_r[i];
        for (uint32_t s = 0; s < FX_PHASER_NUM_STAGES; s++) {
            wet_l = allpass_process(&stages_l[s], wet_l, a);
            wet_r = allpass_process(&stages_r[s], wet_r, a);
        }

        out_l[i] = out_l[i] * dry + wet_l * wet;
        out_r[i] = out_r[i] * dry + wet_r * wet;

        lfo_phase += phase_inc;
        if (lfo_phase >= 2.0f * (float)M_PI) lfo_phase -= 2.0f * (float)M_PI;
    }
}
