/*
 * tempo_sync.c - see tempo_sync.h for the parameter convention.
 */

#include "tempo_sync.h"
#include "config.h"   // DSP_TIME_CRITICAL

DSP_TIME_CRITICAL
uint8_t tempo_sync_step_from_raw(uint8_t raw)
{
    // param1 IS the step number directly (1-16), clamped at the edges --
    // not a raw 0-255 byte bucketed into 16 ranges. An earlier revision of
    // this function did that bucketing ((raw >> 4) + 1), which meant
    // param1 values 0x00-0x0F all mapped to step 1: confusing when setting
    // param1 by hand over the FX UART, where sending literal 1, 2, 3...
    // is the natural way to pick a step. 0 clamps up to step 1 (not an
    // error -- there's no reason to refuse it) rather than down to 0.
    if (raw < 1u) return 1u;
    if (raw > TEMPO_SYNC_STEPS) return (uint8_t)TEMPO_SYNC_STEPS;
    return raw;
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
float fx_feedback_from_raw(uint8_t raw)
{
    return ((float)raw / 255.0f) * FX_FEEDBACK_MAX;
}
