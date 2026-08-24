/*
 * fx_djfilter.c - see fx_djfilter.h
 */

#include "fx_djfilter.h"
#include "fx_control.h"
#include "config.h"   // DSP_TIME_CRITICAL
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f   // same fallback convention as crossover.c/fx_phaser.c
#endif

typedef struct {
    float x1, x2, y1, y2;   // Direct Form I history
} BiquadState;

static BiquadState state_l;
static BiquadState state_r;

void fx_djfilter_init(void)
{
    state_l.x1 = state_l.x2 = state_l.y1 = state_l.y2 = 0.0f;
    state_r.x1 = state_r.x2 = state_r.y1 = state_r.y2 = 0.0f;
}

static inline float biquad_process(BiquadState *st, float in,
                                    float b0, float b1, float b2, float a1, float a2)
{
    float out = b0 * in + b1 * st->x1 + b2 * st->x2 - a1 * st->y1 - a2 * st->y2;
    st->x2 = st->x1;
    st->x1 = in;
    st->y2 = st->y1;
    st->y1 = out;
    return out;
}

// RAM-resident for the same reason as the other FX process-block
// functions (see fx_delay.c's comment): shares the per-sample hot path,
// chained after slot 3.
DSP_TIME_CRITICAL
void fx_djfilter_process_block(float *out_l, float *out_r, uint32_t sample_count,
                                uint32_t sample_rate_hz)
{
    FxState st;
    if (!fx_control_get(FX_DJFILTER_EFFECT_NUM, &st) || !st.enabled) {
        return;   // slot off or unavailable: passthrough, filter state ages silently
    }

    if (st.param1 == 127u) {
        return;   // exact bypass -- guaranteed, not just "very wide open"
    }

    bool  is_lowpass;
    float t;   // 0 = wide open, 1 = fully closed
    if (st.param1 < 127u) {
        is_lowpass = true;
        t = (127.0f - (float)st.param1) / 127.0f;
    } else {
        is_lowpass = false;
        t = ((float)st.param1 - 128.0f) / 127.0f;
    }

    float open_hz   = is_lowpass ? FX_DJFILTER_LP_OPEN_HZ   : FX_DJFILTER_HP_OPEN_HZ;
    float closed_hz = is_lowpass ? FX_DJFILTER_LP_CLOSED_HZ : FX_DJFILTER_HP_CLOSED_HZ;
    // Logarithmic sweep: matches both how a real sweep pot is wired and
    // how pitch/frequency is perceived -- a linear Hz sweep would spend
    // almost the whole knob travel in the (perceptually tiny) top octave.
    float fc = open_hz * powf(closed_hz / open_hz, t);

    if (fc < 10.0f) fc = 10.0f;
    if (fc > (float)sample_rate_hz * 0.45f) fc = (float)sample_rate_hz * 0.45f;

    float q = FX_DJFILTER_Q_MIN + ((float)st.param2 / 255.0f) * (FX_DJFILTER_Q_MAX - FX_DJFILTER_Q_MIN);

    // Standard RBJ cookbook LPF/HPF, normalized by a0.
    float w0     = 2.0f * (float)M_PI * fc / (float)sample_rate_hz;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha  = sin_w0 / (2.0f * q);

    float b0, b1, b2, a0, a1, a2;
    if (is_lowpass) {
        b0 = (1.0f - cos_w0) * 0.5f;
        b1 = 1.0f - cos_w0;
        b2 = (1.0f - cos_w0) * 0.5f;
    } else {
        b0 = (1.0f + cos_w0) * 0.5f;
        b1 = -(1.0f + cos_w0);
        b2 = (1.0f + cos_w0) * 0.5f;
    }
    a0 = 1.0f + alpha;
    a1 = -2.0f * cos_w0;
    a2 = 1.0f - alpha;

    float inv_a0 = 1.0f / a0;
    b0 *= inv_a0; b1 *= inv_a0; b2 *= inv_a0;
    a1 *= inv_a0; a2 *= inv_a0;

    float wet = (float)st.dry_wet / 255.0f;
    float dry = 1.0f - wet;

    for (uint32_t i = 0; i < sample_count; i++) {
        float in_l = out_l[i];
        float in_r = out_r[i];

        float wet_l = biquad_process(&state_l, in_l, b0, b1, b2, a1, a2);
        float wet_r = biquad_process(&state_r, in_r, b0, b1, b2, a1, a2);

        out_l[i] = in_l * dry + wet_l * wet;
        out_r[i] = in_r * dry + wet_r * wet;
    }
}
