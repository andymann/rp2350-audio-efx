/*
 * tempo_sync.c - see tempo_sync.h for the parameter convention.
 */

#include "tempo_sync.h"
#include "config.h"   // DSP_TIME_CRITICAL

DSP_TIME_CRITICAL
uint8_t tempo_sync_clamp1_from_raw(uint8_t raw, uint8_t max_accepted)
{
    // Shared "0-indexed wire value -> 1-indexed internal value" pattern:
    // the raw byte's ACCEPTED range is [0, max_accepted]; anything above
    // max_accepted clamps down to it rather than being refused (same
    // clamp-not-refuse policy used throughout this codebase). The
    // returned value is raw+1, so 0 maps to 1 (never 0 -- callers that
    // use this as a step/count/index generally can't do anything
    // meaningful with a literal 0). max_accepted must be < 255 so the
    // +1 below can't overflow a uint8_t.
    if (raw > max_accepted) raw = max_accepted;
    return (uint8_t)(raw + 1u);
}

DSP_TIME_CRITICAL
uint8_t tempo_sync_step_from_raw(uint8_t raw)
{
    // param1's ACCEPTED range is 0-15 (16 values), mapping directly to
    // steps 1-16 -- 0 is step 1, 15 is step 16, and anything above 15 is
    // ignored (clamped down to 15, i.e. step 16). An earlier revision of
    // this function accepted 1-16 directly (no shift); changed so 0 is a
    // valid, meaningful first option rather than being clamped up to 1,
    // and raw values above the new 15 ceiling are explicitly ignored
    // rather than silently still doing something (they all just land on
    // step 16, same as raw=15).
    return tempo_sync_clamp1_from_raw(raw, 15u);
}

DSP_TIME_CRITICAL
float tempo_sync_ms(uint8_t step, uint16_t bpm_x100)
{
    if (step < 1) step = 1;
    if (step > TEMPO_SYNC_STEPS) step = TEMPO_SYNC_STEPS;
    if (bpm_x100 == 0) bpm_x100 = 1;   // guard div-by-zero from a bad Set BPM value

    float bpm        = bpm_x100 / 100.0f;
    float quarter_ms = 60000.0f / bpm;

    uint8_t is_triplet = (step > 8);
    uint8_t n = is_triplet ? (uint8_t)(step - 8) : step;   // 1..8 quarters

    float ms = n * quarter_ms;
    if (is_triplet) {
        ms *= (2.0f / 3.0f);   // 3 notes in the space of 2 straight ones
    }
    return ms;
}

DSP_TIME_CRITICAL
uint32_t tempo_sync_samples(uint8_t step, uint16_t bpm_x100, uint32_t sample_rate_hz)
{
    float ms = tempo_sync_ms(step, bpm_x100);
    return (uint32_t)(ms * (float)sample_rate_hz / 1000.0f + 0.5f);
}

uint32_t tempo_sync_samples_from_raw(uint8_t raw, uint16_t bpm_x100, uint32_t sample_rate_hz)
{
    return tempo_sync_samples(tempo_sync_step_from_raw(raw), bpm_x100, sample_rate_hz);
}

DSP_TIME_CRITICAL
float tempo_sync_bar_fraction_ms(uint8_t n, uint16_t subdivisions_per_bar, uint16_t bpm_x100)
{
    if (n < 1u) n = 1u;   // guard a zero-length interval, same policy as tempo_sync_ms's step clamp
    if (subdivisions_per_bar < 1u) subdivisions_per_bar = 1u;
    if (bpm_x100 == 0) bpm_x100 = 1;   // guard div-by-zero from a bad Set BPM value

    float bpm    = bpm_x100 / 100.0f;
    float bar_ms = (60000.0f / bpm) * 4.0f;   // 4/4 bar = 4 quarters

    return n * (bar_ms / (float)subdivisions_per_bar);
}

DSP_TIME_CRITICAL
uint32_t tempo_sync_bar_fraction_samples(uint8_t n, uint16_t subdivisions_per_bar,
                                          uint16_t bpm_x100, uint32_t sample_rate_hz)
{
    float ms = tempo_sync_bar_fraction_ms(n, subdivisions_per_bar, bpm_x100);
    return (uint32_t)(ms * (float)sample_rate_hz / 1000.0f + 0.5f);
}

DSP_TIME_CRITICAL
float fx_feedback_from_raw(uint8_t raw)
{
    return ((float)raw / 255.0f) * FX_FEEDBACK_MAX;
}
