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

// Always-recording circular buffer: written unconditionally, every
// sample, regardless of enabled state.
static int16_t __uninitialized_psram("fx_beatrepeat_record") record_buf[FX_BEATREPEAT_MAX_SAMPLES];
static uint32_t record_write_pos = 0;

// Frozen snapshot buffer: only touched during a SNAPSHOTTING copy or a
// LOOPING playback read, never written to by the always-on recorder.
static int16_t __uninitialized_psram("fx_beatrepeat_loop") loop_buf[FX_BEATREPEAT_MAX_SAMPLES];

static bool psram_ok = false;
static bool was_enabled = false;

typedef enum { PHASE_IDLE, PHASE_SNAPSHOTTING, PHASE_LOOPING } Phase;
static Phase phase = PHASE_IDLE;

static uint32_t loop_len_samples   = 0;   // locked in at the enable-edge
static uint32_t snapshot_src_start = 0;   // record_buf index the copy started reading from
static uint32_t snapshot_copied    = 0;   // 0..loop_len_samples, progress of the current copy

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
               psram_check_address(&record_buf[FX_BEATREPEAT_MAX_SAMPLES - 1]) &&
               psram_check_address(&loop_buf[FX_BEATREPEAT_MAX_SAMPLES - 1]);
    if (psram_ok) {
        memset(record_buf, 0, sizeof(record_buf));
        memset(loop_buf, 0, sizeof(loop_buf));
    }
    record_write_pos = 0;
    was_enabled = false;
    phase = PHASE_IDLE;
    loop_len_samples = 0;
    snapshot_src_start = 0;
    snapshot_copied = 0;
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
// when a snapshot copy completes and once per full loop-cycle
// wraparound (so param2 changes take effect at a clean boundary, not
// mid-slice).
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
    bool rising_edge = (now_enabled && !was_enabled);
    was_enabled = now_enabled;   // update BEFORE any early-return so the edge is seen next call

    if (rising_edge) {
        // Lock in the loop length for this session from param1 + current
        // BPM; changes to either afterward only take effect on the NEXT
        // enable-edge. Snapshot the most recent loop_len_samples of
        // record_buf, ending at the write position AS OF NOW (this block
        // hasn't written any new samples yet) -- at most one block
        // (AUDIO_BUFFER_SAMPLES, ~4ms) stale, imperceptible.
        uint16_t bpm_x100 = fx_control_get_bpm();
        uint32_t len = tempo_sync_bar_fraction_samples(st.param1, 16u, bpm_x100, sample_rate_hz);
        if (len < 1u) len = 1u;
        if (len > FX_BEATREPEAT_MAX_SAMPLES) len = FX_BEATREPEAT_MAX_SAMPLES;
        loop_len_samples = len;
        snapshot_src_start = (record_write_pos + FX_BEATREPEAT_MAX_SAMPLES - len) % FX_BEATREPEAT_MAX_SAMPLES;
        snapshot_copied = 0;
        phase = PHASE_SNAPSHOTTING;
    }

    // Spread the record_buf -> loop_buf copy across blocks in a bounded
    // chunk per call -- see fx_beatrepeat.h's top comment for why this
    // can't just be one memcpy done synchronously on the trigger block
    // for the worst case, even though it now uses memcpy() internally.
    // At FX_BEATREPEAT_COPY_CHUNK_SAMPLES=96000, the default 12-sixteenth
    // loop length (72000 samples) completes within this SINGLE call --
    // no added latency beyond the block's own ~4ms period, same as any
    // other effect. Only loop lengths longer than the chunk (approaching
    // the 768000-sample maximum) still spread across a few blocks.
    if (now_enabled && phase == PHASE_SNAPSHOTTING) {
        uint32_t remaining = loop_len_samples - snapshot_copied;
        uint32_t chunk = remaining < FX_BEATREPEAT_COPY_CHUNK_SAMPLES
                              ? remaining : FX_BEATREPEAT_COPY_CHUNK_SAMPLES;

        // At most two linear runs to cover the circular source range
        // [src_start, src_start+chunk) mod SIZE -- avoids a per-element
        // modulo (each source index computed once here, not once per
        // sample), letting memcpy() do the actual transfer.
        uint32_t src_start = (snapshot_src_start + snapshot_copied) % FX_BEATREPEAT_MAX_SAMPLES;
        uint32_t first_run = FX_BEATREPEAT_MAX_SAMPLES - src_start;
        if (first_run > chunk) first_run = chunk;
        memcpy(&loop_buf[snapshot_copied], &record_buf[src_start],
               first_run * sizeof(int16_t));
        uint32_t second_run = chunk - first_run;
        if (second_run > 0u) {
            memcpy(&loop_buf[snapshot_copied + first_run], &record_buf[0],
                   second_run * sizeof(int16_t));
        }

        snapshot_copied += chunk;
        if (snapshot_copied >= loop_len_samples) {
            phase = PHASE_LOOPING;
            seq_idx = 0;
            samples_into_slice = 0;
            compute_slice_order(st.param2);
        }
    }

    float wet = (float)st.dry_wet / 255.0f;
    float dry = 1.0f - wet;

    for (uint32_t i = 0; i < sample_count; i++) {
        float in_l = out_l[i];
        float in_r = out_r[i];

        // Always record -- unconditional, regardless of enabled state or
        // phase, so a re-trigger always has fresh audio ready.
        float mono_in = 0.5f * (in_l + in_r);
        if (mono_in > 1.0f) mono_in = 1.0f;
        else if (mono_in < -1.0f) mono_in = -1.0f;
        record_buf[record_write_pos] = (int16_t)(mono_in * 32767.0f);
        record_write_pos++;
        if (record_write_pos >= FX_BEATREPEAT_MAX_SAMPLES) record_write_pos = 0;

        if (!now_enabled || phase != PHASE_LOOPING) {
            continue;   // disabled, or still mid-snapshot: passthrough (out_l/out_r left as-is)
        }

        // PHASE_LOOPING
        uint8_t orig_slice = slice_order[seq_idx];
        uint32_t sample_offset = slice_info[orig_slice].start + samples_into_slice;
        float wet_sample = (float)loop_buf[sample_offset] * (1.0f / 32767.0f);

        out_l[i] = in_l * dry + wet_sample * wet;
        out_r[i] = in_r * dry + wet_sample * wet;

        samples_into_slice++;
        if (samples_into_slice >= slice_info[orig_slice].len) {
            samples_into_slice = 0;
            seq_idx++;
            if (seq_idx >= FX_BEATREPEAT_NUM_SLICES) {
                seq_idx = 0;
                // Full cycle complete: re-read param2 for the next
                // cycle (a clean boundary to change them at, not mid-slice).
                compute_slice_order(st.param2);
            }
        }
    }
}
