/*
 * fx_delay.h - Effect slot 0: tempo-synced feedback delay
 *
 * The first effect wired from fx_control's control-plane state into the
 * actual audio path. Applied to the main S/PDIF 1 L/R pair (buf_out[0]/[1])
 * as a mono-summed feedback delay: both channels feed the same delay line
 * and receive the same wet signal back (a simple, symmetric echo rather
 * than a stereo ping-pong).
 *
 * Parameter mapping (see tempo_sync.h for the shared convention):
 *   param1  - time division, 1-16 (raw byte IS the step number directly,
 *             clamped at the edges via tempo_sync_step_from_raw()), converted to samples via
 *             tempo_sync_samples() against fx_control_get_bpm().
 *   param2  - feedback, raw byte scaled via fx_feedback_from_raw()
 *             (0.0-0.95).
 *   dry_wet - standard FxState field (0-255), used as the wet mix amount.
 *   param3  - unused by this effect.
 *
 * BUFFER: lives in the board's 8MB QSPI PSRAM (__uninitialized_psram(),
 * see memmap_dspi_rp2350_xip.ld's .psram_noload section and
 * boards/rp2350b_audio_efx.h's PICO_PSRAM_SIZE_BYTES/PICO_PSRAM_CS_PIN),
 * not the on-chip SRAM an earlier revision of this file used -- SRAM was
 * too tight (~80KB free) to cover the full 16-step tempo_sync range at
 * musically reasonable tempos.
 *
 * FX_DELAY_MAX_SAMPLES is sized for the slowest tempo this module treats as
 * "practical" (see FX_DELAY_MIN_PRACTICAL_BPM) at the longest step (16:
 * 8-bar-equivalent triplet). Slower than that gets clamped rather than
 * refused, same policy as before, just a far more generous ceiling: at
 * 48kHz the cap is now measured in seconds, not milliseconds, and halves at
 * 96kHz since it is fixed in samples.
 *
 * PSRAM clock is capped at 40MHz (firmware/CMakeLists.txt,
 * PICO_DEFAULT_PSRAM_MAX_FREQ) rather than the SDK's 133MHz default. An
 * earlier session's attempt to use this board's PSRAM (different driver,
 * before hardware_psram existed in the pinned SDK) hit audible write
 * corruption at 133MHz, then again at 100MHz (that time only at the delay
 * buffer's first wraparound), and became reliable at 66MHz THERE -- but on
 * this build, noise persisted at 66MHz even after confirming (onboard LED /
 * fx_delay_psram_ok()) that PSRAM is correctly detected and mapped, ruling
 * out wiring and narrowing it to clock/timing margin specifically. Lowered
 * to 40MHz on that basis. This board+PSRAM-chip combination (RP2350B +
 * AP6404L-class) has public reports of needing a hand-tuned, lower
 * divisor; if noise persists at 40MHz too, see firmware/CMakeLists.txt's
 * comment for the next steps (lower further, then hardware/psram.h's
 * psram_set_params() for manual divisor/rxdelay tuning).
 *
 * fx_delay_process_block() (and the fx_control_get() / fx_control_get_bpm() /
 * tempo_sync_ family / fx_feedback_from_raw() call chain it uses) is
 * RAM-resident (DSP_TIME_CRITICAL), not just flagged for -O3: on RP2350,
 * PSRAM and flash share the same physical QMI bus, so code that stayed in
 * flash would have its instruction fetches compete with this function's
 * own PSRAM data traffic -- the other half of that earlier session's
 * noise, alongside the clock speed.
 */

#ifndef FX_DELAY_H
#define FX_DELAY_H

#include <stdint.h>
#include <stdbool.h>

// The slowest BPM this module sizes its buffer for. Below this, a request
// for the longest step (16) gets clamped to the buffer cap instead of
// getting the full musical length. 20 BPM is already an unusually slow
// tempo for a delay to be tracking; going lower is rare enough to accept
// the clamp rather than spend PSRAM on it.
#define FX_DELAY_MIN_PRACTICAL_BPM 20u

// 8 quarters (step 8) * 2/3 (triplet, step 16) at 20 BPM (quarter = 3000ms
// @ 20 BPM) = 16000ms = 16s @ 48kHz -> 768000 samples. int16 mono:
// 768000 * 2 bytes = 1.5MB (comfortably inside the 8MB PSRAM budget).
#define FX_DELAY_MAX_SAMPLES 768000u

// Zero the delay buffer. Call once at boot, after PSRAM is up (i.e.
// anywhere in main() -- runtime_init brings PSRAM up before main() runs)
// and before the pipeline starts.
void fx_delay_init(void);

// True iff fx_delay_init() confirmed delay_buf's PSRAM address range is
// actually mapped (psram_is_available() + psram_check_address()), false if
// PSRAM auto-detection didn't find a chip on any candidate CS pin. main.c
// uses this to drive the onboard LED as a physical go/no-go signal for
// PSRAM bring-up, separate from (and available before) any UART traffic.
bool fx_delay_psram_ok(void);

// Process sample_count samples of the main stereo pair in place. Reads
// effect slot 0's state via fx_control_get(0, ...); no-op (passthrough) if
// that slot is disabled. Safe to call every packet regardless of state.
void fx_delay_process_block(float *out_l, float *out_r, uint32_t sample_count,
                             uint32_t sample_rate_hz);

#endif // FX_DELAY_H

