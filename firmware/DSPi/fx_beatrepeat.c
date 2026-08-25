/*
 * fx_beatrepeat.c - see fx_beatrepeat.h
 */

#include "fx_beatrepeat.h"
#include "fx_control.h"
#include "tempo_sync.h"
#include "config.h"                   // DSP_TIME_CRITICAL
#include "pico/platform/sections.h"   // __uninitialized_psram()
#include "hardware/psram.h"           // psram_is_available(), psram_check_address()
#include <string.h>

// The one buffer: recorded into during RECORDING, played back from
// during LOOPING, zeroed in the background during CLEARING.
static int16_t __uninitialized_psram("fx_beatrepeat") loop_buf[FX_BEATREPEAT_MAX_SAMPLES];

static bool psram_ok = false;
static bool was_enabled = false;

typedef enum { PHASE_IDLE, PHASE_CLEARING, PHASE_RECORDING, PHASE_LOOPING, PHASE_FADING_OUT } Phase;
static Phase phase = PHASE_IDLE;

static uint32_t loop_len_samples = 0;    // locked in at the enable-edge
static uint32_t record_pos = 0;          // 0..loop_len_samples-1, during RECORDING
static uint32_t clear_pos = 0;           // 0..FX_BEATREPEAT_MAX_SAMPLES, during CLEARING
static uint32_t fade_pos = 0;            // 0..FX_BEATREPEAT_FADE_SAMPLES, during FADING_OUT
static float    fade_start_wet = 0.0f;   // loop wet gain at the moment the fade began
static float    fade_start_live = 0.0f;  // live gain (param3-based) at the moment the fade began

typedef struct {
    uint32_t start;   // sample offset within loop_buf
    uint32_t len;      // length in samples
} SliceInfo;

static SliceInfo slice_info[FX_BEATREPEAT_NUM_SLICES];
static uint8_t   slice_order[FX_BEATREPEAT_NUM_SLICES];

static uint32_t seq_idx = 0;             // position within slice_order[], during LOOPING
static uint32_t samples_into_slice = 0;

void fx_beatrepeat_init(void)
{
    psram_ok = psram_is_available() &&
               psram_check_address(&loop_buf[FX_BEATREPEAT_MAX_SAMPLES - 1]);
    if (psram_ok) {
        memset(loop_buf, 0, sizeof(loop_buf));
    }
    was_enabled = false;
    phase = PHASE_IDLE;
    loop_len_samples = 0;
    record_pos = 0;
    clear_pos = 0;
    fade_pos = 0;
    fade_start_wet = 0.0f;
    fade_start_live = 0.0f;
    seq_idx = 0;
    samples_into_slice = 0;
}

bool fx_beatrepeat_psram_ok(void)
{
    return psram_ok;
}

// Fills slice_info[] (boundaries of the FX_BEATREPEAT_NUM_SLICES equal
// pieces, last one absorbing any remainder so the total covers
// loop_len_samples exactly) and slice_order[] (the param2 playback
// permutation) for the CURRENT loop_len_samples/pattern. Called once
// when RECORDING completes and once per full loop-cycle wraparound (so
// param2 changes take effect at a clean boundary, not mid-slice).
static void compute_slice_order(uint8_t pattern)
{
    const uint8_t num_slices = FX_BEATREPEAT_NUM_SLICES;

    uint32_t base_len  = loop_len_samples / num_slices;
    uint32_t remainder = loop_len_samples % num_slices;
    uint32_t offset = 0;
    for (uint8_t i = 0; i < num_slices; i++) {
        uint32_t len = base_len;
        if (i == num_slices - 1u) len += remainder;   // last original slice absorbs rounding
        slice_info[i].start = offset;
        slice_info[i].len   = len;
        offset += len;
    }

    uint8_t n = num_slices;
    switch (pattern) {
        default:
        case 0:   // forward
            for (uint8_t i = 0; i < n; i++) slice_order[i] = i;
            break;
        case 1:   // first + reverse rest (spec example)
            slice_order[0] = 0;
            for (uint8_t i = 1; i < n; i++) slice_order[i] = (uint8_t)(n - i);
            break;
        case 2:   // full reverse
            for (uint8_t i = 0; i < n; i++) slice_order[i] = (uint8_t)(n - 1u - i);
            break;
        case 3:   // last + forward rest
            slice_order[0] = (uint8_t)(n - 1u);
            for (uint8_t i = 1; i < n; i++) slice_order[i] = (uint8_t)(i - 1u);
            break;
        case 4: { // adjacent pairs swapped
            uint8_t k = 0;
            for (; k + 1u < n; k += 2u) {
                slice_order[k]      = (uint8_t)(k + 1u);
                slice_order[k + 1u] = k;
            }
            if (k < n) slice_order[k] = k;   // odd n: last slice stays
            break;
        }
        case 5:   // rotate left 1
            for (uint8_t i = 0; i < n; i++) slice_order[i] = (uint8_t)((i + 1u) % n);
            break;
        case 6:   // rotate right 1
            for (uint8_t i = 0; i < n; i++) slice_order[i] = (uint8_t)((i + n - 1u) % n);
            break;
        case 7: { // rotate left by ceil(n/2)
            uint8_t shift = (uint8_t)((n + 1u) / 2u);
            for (uint8_t i = 0; i < n; i++) slice_order[i] = (uint8_t)((i + shift) % n);
            break;
        }
        case 8: { // rotate right by floor(n/2)
            uint8_t shift = (uint8_t)(n / 2u);
            for (uint8_t i = 0; i < n; i++) slice_order[i] = (uint8_t)((i + n - shift) % n);
            break;
        }
        case 9: { // odd-then-even (1-indexed slice numbers)
            uint8_t idx = 0;
            for (uint8_t s = 1; s <= n; s += 2u) slice_order[idx++] = (uint8_t)(s - 1u);
            for (uint8_t s = 2; s <= n; s += 2u) slice_order[idx++] = (uint8_t)(s - 1u);
            break;
        }
        case 10: { // even-then-odd (1-indexed slice numbers)
            uint8_t idx = 0;
            for (uint8_t s = 2; s <= n; s += 2u) slice_order[idx++] = (uint8_t)(s - 1u);
            for (uint8_t s = 1; s <= n; s += 2u) slice_order[idx++] = (uint8_t)(s - 1u);
            break;
        }
        case 11: { // riffle interleave, first half leads (out-shuffle)
            uint8_t h = (uint8_t)((n + 1u) / 2u);   // first-half size, ceil
            uint8_t idx = 0;
            for (uint8_t k = 0; idx < n; k++) {
                if (k < h)      slice_order[idx++] = k;
                if (k < n - h)  slice_order[idx++] = (uint8_t)(h + k);
            }
            break;
        }
        case 12: { // riffle interleave, second half leads (in-shuffle)
            uint8_t h = (uint8_t)(n / 2u);           // first-half size, floor
            uint8_t idx = 0;
            for (uint8_t k = 0; idx < n; k++) {
                if (k < n - h)  slice_order[idx++] = (uint8_t)(h + k);
                if (k < h)      slice_order[idx++] = k;
            }
            break;
        }
        case 13: { // each half reversed in place
            uint8_t h = (uint8_t)(n / 2u);           // first-half size, floor
            uint8_t idx = 0;
            for (uint8_t i = 0; i < h; i++) slice_order[idx++] = (uint8_t)(h - 1u - i);
            for (uint8_t i = h; i < n; i++) slice_order[idx++] = (uint8_t)(n - 1u - (i - h));
            break;
        }
        case 14: { // halves swapped and each reversed
            uint8_t h = (uint8_t)(n / 2u);
            uint8_t idx = 0;
            for (uint8_t i = n; i > h; i--) slice_order[idx++] = (uint8_t)(i - 1u);
            for (uint8_t i = h; i > 0u; i--) slice_order[idx++] = (uint8_t)(i - 1u);
            break;
        }
        case 15: { // outside-in zigzag
            uint8_t idx = 0;
            int lo = 0, hi = (int)n - 1;
            bool from_lo = true;
            while (lo <= hi) {
                if (from_lo) slice_order[idx++] = (uint8_t)lo++;
                else         slice_order[idx++] = (uint8_t)hi--;
                from_lo = !from_lo;
            }
            break;
        }
    }
}

// RAM-resident for the same reason as the other FX process-block
// functions (see fx_delay.c's comment): shares the per-sample hot path.
DSP_TIME_CRITICAL
void fx_beatrepeat_process_block(float *out_l, float *out_r, uint32_t sample_count,
                                  uint32_t sample_rate_hz)
{
    if (!psram_ok) {
        return;   // PSRAM not confirmed present/mapped -- see fx_delay.c's identical guard
    }

    FxState st;
    if (!fx_control_get(FX_BEATREPEAT_EFFECT_NUM, &st)) {
        return;
    }

    bool now_enabled = st.enabled;
    bool rising_edge  = (now_enabled && !was_enabled);
    bool falling_edge = (!now_enabled && was_enabled);
    was_enabled = now_enabled;   // update BEFORE any early-return so edges are never missed

    if (rising_edge) {
        // Lock in the loop length for this session from param1 + current
        // BPM; changes to either afterward only take effect on the NEXT
        // enable-edge.
        uint16_t bpm_x100 = fx_control_get_bpm();
        uint32_t len = tempo_sync_bar_fraction_samples(st.param1, 16u, bpm_x100, sample_rate_hz);
        if (len < 1u) len = 1u;
        if (len > FX_BEATREPEAT_MAX_SAMPLES) len = FX_BEATREPEAT_MAX_SAMPLES;
        loop_len_samples = len;
        record_pos = 0;
        phase = PHASE_RECORDING;
    } else if (falling_edge) {
        if (phase == PHASE_LOOPING) {
            // Was actively mixing in the wet (looped) signal and
            // possibly a partial (param3-scaled) live signal -- an
            // instant switch to raw passthrough would jump between
            // whatever that mix was and the live signal at full gain,
            // audible as a click/crackle. Fade the loop's contribution
            // to 0 AND the live gain up to full (1.0 -- disabling always
            // means "back to normal passthrough", regardless of what
            // param3 was set to while looping) over
            // FX_BEATREPEAT_FADE_SAMPLES instead of jumping instantly.
            fade_pos = 0;
            fade_start_wet = (float)st.dry_wet / 255.0f;
            fade_start_live = (float)st.param3 / 255.0f;
            phase = PHASE_FADING_OUT;
        } else {
            // Was RECORDING (already plain passthrough, nothing to fade)
            // or already IDLE/CLEARING -- no discontinuity to smooth,
            // go straight to clearing.
            clear_pos = 0;
            phase = PHASE_CLEARING;
        }
    }

    // Background clearing: bounded chunk per block, no audible urgency
    // (output is plain passthrough throughout, since !now_enabled).
    if (phase == PHASE_CLEARING) {
        uint32_t remaining = FX_BEATREPEAT_MAX_SAMPLES - clear_pos;
        uint32_t chunk = remaining < FX_BEATREPEAT_CLEAR_CHUNK_SAMPLES
                              ? remaining : FX_BEATREPEAT_CLEAR_CHUNK_SAMPLES;
        memset(&loop_buf[clear_pos], 0, chunk * sizeof(int16_t));
        clear_pos += chunk;
        if (clear_pos >= FX_BEATREPEAT_MAX_SAMPLES) {
            phase = PHASE_IDLE;
        }
    }

    float wet = (float)st.dry_wet / 255.0f;

    for (uint32_t i = 0; i < sample_count; i++) {
        float in_l = out_l[i];
        float in_r = out_r[i];

        if (phase == PHASE_RECORDING) {
            float mono_in = 0.5f * (in_l + in_r);
            if (mono_in > 1.0f) mono_in = 1.0f;
            else if (mono_in < -1.0f) mono_in = -1.0f;
            loop_buf[record_pos] = (int16_t)(mono_in * 32767.0f);
            record_pos++;
            // Live passthrough during the fill -- no valid wet signal
            // yet, dry_wet is ignored for this phase (see
            // fx_beatrepeat.h). out_l[i]/out_r[i] already hold the live
            // signal; leave as-is.

            if (record_pos >= loop_len_samples) {
                // Seamless: the very next sample reads from loop_buf[0]
                // with no gap and no data-movement step in between --
                // filling and playing back are the same buffer.
                phase = PHASE_LOOPING;
                seq_idx = 0;
                samples_into_slice = 0;
                compute_slice_order(st.param2);
            }
            continue;
        }

        if (phase == PHASE_FADING_OUT) {
            // Same playback as LOOPING (the loop content keeps advancing
            // naturally, avoiding a second discontinuity in the wet
            // signal itself), but the loop's gain ramps down to 0 while
            // the live gain ramps UP to 1.0 (full passthrough) over the
            // fade window -- ends exactly at the disabled/bypass state.
            float t = (float)fade_pos / (float)FX_BEATREPEAT_FADE_SAMPLES;
            float fade_wet  = fade_start_wet * (1.0f - t);
            float fade_live = fade_start_live + (1.0f - fade_start_live) * t;

            uint8_t orig_slice = slice_order[seq_idx];
            uint32_t sample_offset = slice_info[orig_slice].start + samples_into_slice;
            float wet_sample = (float)loop_buf[sample_offset] * (1.0f / 32767.0f);

            out_l[i] = in_l * fade_live + wet_sample * fade_wet;
            out_r[i] = in_r * fade_live + wet_sample * fade_wet;

            samples_into_slice++;
            if (samples_into_slice >= slice_info[orig_slice].len) {
                samples_into_slice = 0;
                seq_idx++;
                if (seq_idx >= FX_BEATREPEAT_NUM_SLICES) seq_idx = 0;
                // No compute_slice_order() call here -- the fade is brief
                // (FX_BEATREPEAT_FADE_SAMPLES) and about to end anyway,
                // not worth reacting to a param2 change mid-fade.
            }

            fade_pos++;
            if (fade_pos >= FX_BEATREPEAT_FADE_SAMPLES) {
                clear_pos = 0;
                phase = PHASE_CLEARING;
            }
            continue;
        }

        if (phase != PHASE_LOOPING) {
            continue;   // IDLE or CLEARING: passthrough (out_l/out_r left as-is)
        }

        // PHASE_LOOPING. param3 and dry_wet are independent gains, not a
        // complementary crossfade: param3 controls how much of the LIVE
        // incoming signal is heard (0 = blocked, 255 = full), dry_wet
        // controls how much of the LOOP is heard -- at both maxed, the
        // two are summed together ("passed through and mixed with the
        // loop", per spec), not blended.
        uint8_t orig_slice = slice_order[seq_idx];
        uint32_t sample_offset = slice_info[orig_slice].start + samples_into_slice;
        float wet_sample = (float)loop_buf[sample_offset] * (1.0f / 32767.0f);
        float live_gain = (float)st.param3 / 255.0f;

        out_l[i] = in_l * live_gain + wet_sample * wet;
        out_r[i] = in_r * live_gain + wet_sample * wet;

        samples_into_slice++;
        if (samples_into_slice >= slice_info[orig_slice].len) {
            samples_into_slice = 0;
            seq_idx++;
            if (seq_idx >= FX_BEATREPEAT_NUM_SLICES) {
                seq_idx = 0;
                // Full cycle complete: re-read param2 for the next
                // cycle (a clean boundary to change it at, not mid-slice).
                compute_slice_order(st.param2);
            }
        }
    }
}
