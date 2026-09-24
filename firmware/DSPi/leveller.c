/*
 * leveller.c - see leveller.h
 *
 * Adapted from the original DSPi project's own leveller.c (RP2350 float
 * path only -- see leveller.h's top comment for what changed and why).
 * Signal flow per block, unchanged from the original:
 *   1. Per-sample: update RMS envelopes for both channels
 *   2. Per-block:  link = louder of the two envelopes; compute gain via
 *      soft-knee curve
 *   3. Per-block:  smooth gain with asymmetric attack/release
 *   4. Per-sample: lookahead delay (if enabled), interpolated gain +
 *      safety limiter
 */

#include "leveller.h"
#include "config.h"

#include <math.h>
#include <string.h>

static LevellerConfig cfg;
static LevellerCoeffs coeffs;
static LevellerState state;

// Set by leveller_set_config() (core 0, via fx_control.c) when lookahead
// toggles, cleared by leveller_process_block() (core 1) once it's acted
// on. This -- rather than calling reset_state() directly from core 0 --
// avoids a genuine cross-core race: leveller_process_block() is only
// ever called from core 1's own main loop, one block at a time, so
// having IT perform the reset (instead of core 0 reaching into state
// that core 1 might be mid-block on) means the reset can never land in
// the middle of a read/write of state.lookahead_buf[]/env_sq[]. Same
// simple-flag, no-explicit-lock cross-core pattern as fx_control.c's
// clock_restart_pending.
static volatile bool leveller_reset_pending = false;

// ---------------------------------------------------------------------------
// Speed preset table: {attack_sec, release_sec, rms_window_sec} -- same
// values as the original.
// ---------------------------------------------------------------------------

static const float speed_presets[LEVELLER_SPEED_COUNT][3] = {
    /* Slow   */ { 0.100f, 2.000f, 0.400f },
    /* Medium */ { 0.050f, 1.000f, 0.200f },
    /* Fast   */ { 0.020f, 0.500f, 0.100f },
};

static float compute_alpha(float sample_rate, float time_sec) {
    if (time_sec <= 0.0f || sample_rate <= 0.0f) return 0.0f;
    return expf(-logf(10.0f) / (sample_rate * time_sec));
}

static void compute_coefficients(float sample_rate_hz)
{
    if (sample_rate_hz < 1.0f) sample_rate_hz = (float)SAMPLE_RATE_HZ;

    uint8_t spd = cfg.speed;
    if (spd >= LEVELLER_SPEED_COUNT) spd = LEVELLER_SPEED_MEDIUM;

    float attack_sec  = speed_presets[spd][0];
    float release_sec = speed_presets[spd][1];
    float rms_sec     = speed_presets[spd][2];

    coeffs.alpha_rms     = compute_alpha(sample_rate_hz, rms_sec);
    coeffs.alpha_attack  = compute_alpha(sample_rate_hz, attack_sec);
    coeffs.alpha_release = compute_alpha(sample_rate_hz, release_sec);

    coeffs.threshold_db  = LEVELLER_THRESHOLD_DB;
    coeffs.knee_width_db = LEVELLER_KNEE_WIDTH_DB;

    float gate = cfg.gate_threshold_db;
    if (gate < LEVELLER_GATE_MIN) gate = LEVELLER_GATE_MIN;
    if (gate > LEVELLER_GATE_MAX) gate = LEVELLER_GATE_MAX;
    coeffs.gate_threshold_db = gate;

    float amount = cfg.amount;
    if (amount < LEVELLER_AMOUNT_MIN) amount = LEVELLER_AMOUNT_MIN;
    if (amount > LEVELLER_AMOUNT_MAX) amount = LEVELLER_AMOUNT_MAX;
    float norm = amount / 100.0f;
    coeffs.ratio = 1.0f + norm * 19.0f;   // 1:1 at 0%, 20:1 at 100%

    float max_g = cfg.max_gain_db;
    if (max_g < LEVELLER_MAX_GAIN_MIN) max_g = LEVELLER_MAX_GAIN_MIN;
    if (max_g > LEVELLER_MAX_GAIN_MAX) max_g = LEVELLER_MAX_GAIN_MAX;
    coeffs.max_gain_db = max_g;

    // No makeup gain; upward compression provides the boost directly.
    coeffs.makeup_db = 0.0f;
}

static void reset_state(void)
{
    memset(&state, 0, sizeof(state));
    state.gain_linear = 1.0f;
    state.gain_prev_linear = 1.0f;
    state.gain_smooth_db = 0.0f;
}

void leveller_init(void)
{
    cfg.enabled = LEVELLER_DEFAULT_ENABLED;
    cfg.amount = LEVELLER_DEFAULT_AMOUNT;
    cfg.speed = LEVELLER_DEFAULT_SPEED;
    cfg.max_gain_db = LEVELLER_DEFAULT_MAX_GAIN_DB;
    cfg.lookahead = LEVELLER_DEFAULT_LOOKAHEAD;
    cfg.gate_threshold_db = LEVELLER_DEFAULT_GATE_DB;

    compute_coefficients((float)SAMPLE_RATE_HZ);
    reset_state();
}

// ---------------------------------------------------------------------------
// Upward Compression Gain Computer -- identical to the original's.
// Boosts content BELOW the threshold, leaves content ABOVE untouched, with
// a quadratic soft knee around the threshold for a smooth transition.
// ---------------------------------------------------------------------------

static inline float gain_computer(float x_db, float threshold, float ratio,
                                  float knee_width) {
    float half_knee = knee_width * 0.5f;

    if (x_db > (threshold + half_knee)) {
        return 0.0f;   // Above knee: no boost
    } else if (x_db >= (threshold - half_knee)) {
        float d = threshold + half_knee - x_db;
        return (1.0f - 1.0f / ratio) * d * d / (2.0f * knee_width);
    } else {
        return (threshold - x_db) * (1.0f - 1.0f / ratio);
    }
}

DSP_TIME_CRITICAL
void leveller_process_block(float *l, float *r, uint32_t count, uint32_t sample_rate_hz)
{
    if (leveller_reset_pending) {
        reset_state();
        leveller_reset_pending = false;
    }

    if (count == 0 || !cfg.enabled) return;
    (void)sample_rate_hz;   // Fixed-rate build (config.h); coefficients are
                            // computed once at init/config-change time, not
                            // re-derived per block.

    // ---- Per-sample: update RMS envelopes (both channels) ----
    const float a_rms = coeffs.alpha_rms;
    const float one_minus_a_rms = 1.0f - a_rms;
    float env_l = state.env_sq[0];
    float env_r = state.env_sq[1];
    for (uint32_t i = 0; i < count; i++) {
        env_l = a_rms * env_l + one_minus_a_rms * (l[i] * l[i]);
        env_r = a_rms * env_r + one_minus_a_rms * (r[i] * r[i]);
    }
    if (env_l < 1e-30f) env_l = 0.0f;   // Prevent denormals in silent passages
    if (env_r < 1e-30f) env_r = 0.0f;
    state.env_sq[0] = env_l;
    state.env_sq[1] = env_r;
    float link_sq = (env_l > env_r) ? env_l : env_r;   // Stereo-linked: louder wins

    // ---- Per-block: compute target gain from the linked level ----
    float rms_db = 10.0f * log10f(link_sq + 1e-30f);

    float gc_db;
    if (rms_db < coeffs.gate_threshold_db) {
        gc_db = 0.0f;   // Below silence gate: unity gain
    } else {
        gc_db = gain_computer(rms_db, coeffs.threshold_db, coeffs.ratio, coeffs.knee_width_db);
        gc_db += coeffs.makeup_db;
        if (gc_db > coeffs.max_gain_db) gc_db = coeffs.max_gain_db;
    }

    // ---- Per-block: asymmetric gain smoothing ----
    // alpha_attack/release are per-sample coefficients; raised to block
    // size here for the correct per-block time constant.
    float alpha_sample = (gc_db < state.gain_smooth_db) ? coeffs.alpha_attack
                                                        : coeffs.alpha_release;
    float alpha = powf(alpha_sample, (float)count);
    state.gain_smooth_db = alpha * state.gain_smooth_db + (1.0f - alpha) * gc_db;

    state.gain_prev_linear = state.gain_linear;
    state.gain_linear = powf(10.0f, state.gain_smooth_db / 20.0f);

    // ---- Per-sample: lookahead delay + interpolated gain + limiter ----
    float gain_prev = state.gain_prev_linear;
    float gain_cur  = state.gain_linear;
    float gain, gain_step;
    if (count == 1) {
        gain = gain_cur;
        gain_step = 0.0f;
    } else {
        gain_step = (gain_cur - gain_prev) / (float)(count - 1);
        gain = gain_prev;
    }

    const float ceil = LEVELLER_LIMITER_CEIL;
    bool use_la = cfg.lookahead;
    uint32_t la_idx = state.la_write_idx;

    for (uint32_t i = 0; i < count; i++) {
        float out_l = l[i];
        float out_r = r[i];

        if (use_la) {
            float dl = state.lookahead_buf[0][la_idx];
            float dr = state.lookahead_buf[1][la_idx];
            state.lookahead_buf[0][la_idx] = l[i];
            state.lookahead_buf[1][la_idx] = r[i];
            out_l = dl;
            out_r = dr;
            la_idx++;
            if (la_idx >= LEVELLER_LOOKAHEAD_SAMPLES) la_idx = 0;
        }

        // Cap gain so the leveller never boosts a sample above the
        // ceiling; existing loud content is never attenuated (gain
        // capped at 1.0, i.e. pass-through, if already above ceiling).
        float g = gain;
        if (g > 1.0f) {
            float peak = fabsf(out_l);
            float pr = fabsf(out_r);
            if (pr > peak) peak = pr;
            if (peak > 0.0f) {
                float max_g = ceil / peak;
                if (max_g < g) g = (max_g > 1.0f) ? max_g : 1.0f;
            }
        }

        l[i] = out_l * g;
        r[i] = out_r * g;
        gain += gain_step;
    }

    if (use_la) state.la_write_idx = la_idx;
}

void leveller_set_config(bool enabled, float amount, uint8_t speed,
                         float max_gain_db, bool lookahead)
{
    bool lookahead_changed = (lookahead != cfg.lookahead);

    cfg.enabled = enabled;
    cfg.amount = amount;
    cfg.speed = speed;
    cfg.max_gain_db = max_gain_db;
    cfg.lookahead = lookahead;

    compute_coefficients((float)SAMPLE_RATE_HZ);

    // Reset when lookahead toggles -- matches the original's own rule:
    // the ring's content becomes meaningless the moment its bypass state
    // flips (a sudden jump from "delayed" to "live" audio, or vice
    // versa, would otherwise glitch). Raises a flag for core 1 to act on
    // rather than calling reset_state() directly here on core 0 -- see
    // leveller_reset_pending's own comment for why.
    if (lookahead_changed) leveller_reset_pending = true;
}

void leveller_get_config(bool *enabled, float *amount, uint8_t *speed,
                         float *max_gain_db, bool *lookahead)
{
    *enabled = cfg.enabled;
    *amount = cfg.amount;
    *speed = cfg.speed;
    *max_gain_db = cfg.max_gain_db;
    *lookahead = cfg.lookahead;
}
