/*
 * limiter.c - see limiter.h for the algorithm and its no-overshoot proof.
 *
 * Cross-core split: limiter_set_config() runs on core 0 (fx_control.c)
 * and only writes the cfg_* variables below -- each a single aligned
 * 32-bit store, so never torn. limiter_process_block() (core 1) snapshots
 * them once at the top of each block and is the ONLY code that touches
 * the runtime state (delay line, deque, envelope, box filter), so a
 * config change can never land mid-block inside that state. Same
 * "core 0 writes plain values, core 1 owns all DSP state" pattern as
 * leveller.c's pending-reset flag -- here no reset flag is needed at all,
 * since enable/disable transitions are detected by core 1 itself by
 * comparing cfg_enabled against its own `active` state.
 */

#include "limiter.h"
#include "config.h"

#include "hardware/sync.h"
#include <math.h>
#include <string.h>

#define B  LIMITER_LOOKAHEAD_SAMPLES   // box length == delay (see limiter.h)
#define W  (B + 1u)                    // sliding-min window length

// Absorbs the final int64->float conversion's and the multiply's own
// rounding (~1e-7 relative each): 2^-20 = -0.000008 dB, far below one
// 24-bit LSB's worth of level change -- inaudible, but makes "never
// above the ceiling" hold exactly rather than to within float epsilon.
#define GAIN_MARGIN  (1.0f - 1.0f / 1048576.0f)

// ---------------------------------------------------------------------------
// Config (written by core 0, read by core 1)
// ---------------------------------------------------------------------------

static volatile bool  cfg_enabled;
static volatile float cfg_ceiling_lin;
static volatile float cfg_input_gain_lin;
static volatile float cfg_alpha_release;   // per-sample, Form A (leveller.h)

// Raw wire bytes, core 0 only -- for exact Query Limiter round-trip.
static uint8_t raw_ceiling, raw_release, raw_input_gain;

// ---------------------------------------------------------------------------
// Runtime state (core 1 only)
// ---------------------------------------------------------------------------

static float    delay_l[B];
static float    delay_r[B];
// Box filter history in Q26 fixed point (max input gain 25.5dB = x18.8 <
// 32, fits int32), summed exactly in 64 bits: a float running sum drifts
// by ~1e-5 relative within a block -- enough to measurably break the
// ceiling guarantee in testing. Each value is truncated (rounded DOWN)
// when quantised, so the sum can only err on the safe side.
#define Q_ONE  67108864.0f         // 2^26
static int32_t  env_hist[B];      // same ring index as delay_*
static uint32_t idx;              // shared ring index for delay_*/env_hist
static int64_t  box_sum;
static float    env;              // release envelope (linear gain)

// Monotonic deque (ascending values front->back) for the sliding minimum.
static float    dq_val[W];
static uint32_t dq_pos[W];
static uint32_t dq_head, dq_count;
static uint32_t n;                // running sample counter (wraps; only
                                  // ever compared by unsigned difference)

static bool     active;           // core 1's currently-applied enable state

// ---------------------------------------------------------------------------

static float alpha_for_ms(float ms)
{
    // Same "retention" convention as leveller.h: 90% step response at T.
    float t = ms * 0.001f;
    return expf(-logf(10.0f) / ((float)SAMPLE_RATE_HZ * t));
}

static void apply_config(bool enabled, uint8_t ceiling, uint8_t release,
                         uint8_t input_gain)
{
    raw_ceiling    = ceiling;
    raw_release    = release;
    raw_input_gain = input_gain;

    cfg_ceiling_lin    = powf(10.0f, -(float)ceiling    * 0.1f / 20.0f);
    cfg_input_gain_lin = powf(10.0f,  (float)input_gain * 0.1f / 20.0f);
    cfg_alpha_release  = alpha_for_ms((float)(LIMITER_RELEASE_MS_BASE +
                                              (uint32_t)release * LIMITER_RELEASE_MS_STEP));
    __dmb();   // parameters visible before the enable flag that uses them
    cfg_enabled = enabled;
}

void limiter_init(void)
{
    memset(delay_l, 0, sizeof(delay_l));
    memset(delay_r, 0, sizeof(delay_r));
    idx = 0;
    n = 0;
    active = false;
    apply_config(LIMITER_DEFAULT_ENABLED, LIMITER_DEFAULT_CEILING,
                 LIMITER_DEFAULT_RELEASE, LIMITER_DEFAULT_INPUT_GAIN);
}

// One detector step for one incoming sample: steps 1-4 of limiter.h.
// Writes env_hist[slot] and returns the box-filtered gain to apply to the
// sample leaving the delay line at this same instant.
static inline __attribute__((always_inline))
float detect(float peak, uint32_t slot, float ceil, float in_gain, float a_rel)
{
    // 1. Required total gain for this sample (no division unless needed).
    float target = in_gain;
    if (peak * in_gain > ceil) target = ceil / peak;

    // 2. Sliding-window minimum over the last W samples (positions
    //    n-B .. n). Expire the front BEFORE pushing: at that point every
    //    remaining entry lies in n-B .. n-1 (at most B entries), so the
    //    push brings the deque to at most B+1 == W -- exactly its
    //    capacity. The first version pushed first and expired after,
    //    which briefly needed W+1 slots: on any run of more than W
    //    strictly rising targets (falling peaks above the ceiling --
    //    every falling edge of loud bass) the push overwrote the front,
    //    dq_count grew past W, and the indices ran off the end of
    //    dq_val[]/dq_pos[], corrupting .bss and hanging the device.
    if (dq_count && (uint32_t)(n - dq_pos[dq_head]) >= W) {
        if (++dq_head >= W) dq_head = 0;
        dq_count--;
    }
    while (dq_count) {
        uint32_t back = dq_head + dq_count - 1u;
        if (back >= W) back -= W;
        if (dq_val[back] < target) break;
        dq_count--;
    }
    uint32_t ins = dq_head + dq_count;
    if (ins >= W) ins -= W;
    dq_val[ins] = target;
    dq_pos[ins] = n;
    dq_count++;
    float wmin = dq_val[dq_head];
    n++;

    // 3. Instant attack to the window minimum, exponential release.
    if (wmin < env) env = wmin;
    else            env = a_rel * env + (1.0f - a_rel) * wmin;

    // 4. Box filter (length B) over the envelope, exact in Q26.
    int32_t q = (int32_t)(env * Q_ONE);   // truncates toward zero (down)
    box_sum += (int64_t)q - env_hist[slot];
    env_hist[slot] = q;
    return (float)box_sum * (GAIN_MARGIN / ((float)B * Q_ONE));
}

// Rebuild detector state from what is already sitting in the delay line,
// so the no-overshoot guarantee holds from the very first enabled sample
// (the line is kept filled with live audio even while bypassed).
static void prime(float ceil, float in_gain, float a_rel)
{
    dq_head = 0;
    dq_count = 0;
    env = in_gain;
    int32_t q = (int32_t)(in_gain * Q_ONE);
    for (uint32_t k = 0; k < B; k++) env_hist[k] = q;
    box_sum = (int64_t)q * B;

    for (uint32_t k = 0; k < B; k++) {
        uint32_t j = (idx + k) % B;   // oldest -> newest
        float pl = fabsf(delay_l[j]);
        float pr = fabsf(delay_r[j]);
        (void)detect(pl > pr ? pl : pr, j, ceil, in_gain, a_rel);
    }
}

DSP_TIME_CRITICAL
void limiter_process_block(float *l, float *r, uint32_t count)
{
    if (count == 0) return;

    const bool  want    = cfg_enabled;
    const float ceil    = cfg_ceiling_lin;
    const float in_gain = cfg_input_gain_lin;
    const float a_rel   = cfg_alpha_release;

    uint32_t i_ring = idx;

    if (!active && !want) {
        // Bypass: audio untouched, zero latency. Only keep the delay line
        // filled with live audio so enabling later can prime from it.
        for (uint32_t i = 0; i < count; i++) {
            delay_l[i_ring] = l[i];
            delay_r[i_ring] = r[i];
            if (++i_ring >= B) i_ring = 0;
        }
        idx = i_ring;
        return;
    }

    // Fade direction for this block: +1 fading in (bypass -> limited),
    // -1 fading out (limited -> bypass), 0 steady limited. The crossfade
    // is between the undelayed dry signal and the delayed limited one --
    // a one-block (2ms) blend, avoiding the hard 2ms time jump a direct
    // switch would produce.
    int fade = 0;
    if (!active && want) {
        prime(ceil, in_gain, a_rel);
        fade = 1;
    } else if (active && !want) {
        fade = -1;
    }

    const float t_step = 1.0f / (float)count;

    for (uint32_t i = 0; i < count; i++) {
        float xl = l[i];
        float xr = r[i];
        float pl = fabsf(xl);
        float pr = fabsf(xr);

        float g = detect(pl > pr ? pl : pr, i_ring, ceil, in_gain, a_rel);

        float wl = delay_l[i_ring] * g;
        float wr = delay_r[i_ring] * g;
        delay_l[i_ring] = xl;
        delay_r[i_ring] = xr;
        if (++i_ring >= B) i_ring = 0;

        if (fade == 0) {
            l[i] = wl;
            r[i] = wr;
        } else {
            float t = (float)(i + 1u) * t_step;
            if (fade < 0) t = 1.0f - t;
            l[i] = xl + (wl - xl) * t;
            r[i] = xr + (wr - xr) * t;
        }
    }

    idx = i_ring;
    active = want;
}

void limiter_set_config(bool enabled, uint8_t ceiling, uint8_t release,
                        uint8_t input_gain)
{
    apply_config(enabled, ceiling, release, input_gain);
}

void limiter_get_config(bool *enabled, uint8_t *ceiling, uint8_t *release,
                        uint8_t *input_gain)
{
    *enabled    = cfg_enabled;
    *ceiling    = raw_ceiling;
    *release    = raw_release;
    *input_gain = raw_input_gain;
}
