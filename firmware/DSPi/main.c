/*
 * main.c - DSPi minimal build entry point
 *
 * USB audio in + I2S audio in -> FX chain -> I2S audio out. Trimmed from
 * the original DSPi's main.c (3758 lines: S/PDIF, ADAT, PDM, control
 * surfaces, IR remote, bulk params, flash-persisted presets, the vendor
 * USB control protocol, and several DSP-island features this build
 * doesn't have) down to just what that path needs.
 *
 * Architecture: two cores. Core 1 runs the audio processing loop
 * (fill a block via audio_pipeline_fill_block(), write it to the I2S
 * output's own TX ring via i2s_output_write_block(), repeat). Core 0
 * does nothing but call tud_task() in a tight loop.
 *
 * This split exists because an earlier version of this file's I2S
 * output used pico_audio_i2s_multi's high-level API, whose
 * give_audio_buffer() -> i2s_wrap_producer_give() calls
 * get_free_audio_buffer(consumer_pool, true) with a HARDCODED true --
 * unconditionally blocking until the I2S output DMA frees a consumer
 * buffer, regardless of how take_audio_buffer() was called. A single-
 * core revision of this file that called both tud_task() and
 * give_audio_buffer() from the same loop blocked on that call for close
 * to a full audio block period (~4.3ms at 44.1kHz/192 samples) every
 * time, so tud_task() -- and therefore all USB packet servicing -- was
 * only reached ~230 times/sec instead of the ~1000/sec USB full-speed
 * isochronous transfers need. Confirmed directly: instrumented builds
 * showed the main loop itself, xfer_cb, and the fraction of genuinely
 * non-silent audio frames all converging on the same ~22-23% figure
 * (roughly 1 packet serviced per audio block instead of the ~4 that
 * actually arrive), which was audible as severe, "bitcrusher"-like
 * distortion -- not a bug in any DSP/decode logic (extensively verified
 * correct beforehand), but USB simply being starved of CPU time by this
 * blocking call.
 *
 * i2s_output.c no longer uses pico_audio_i2s_multi's high-level API at
 * all (see its own top comment for why -- this device is now clocked
 * by an external PCM1808 breakout board, not itself), so
 * give_audio_buffer()'s specific blocking call is gone -- but the two-
 * core split remains necessary regardless: i2s_output_write_block()
 * still spin-waits for TX ring space, paced by that external clock, and
 * core 0 must never be the one blocked on that wait.
 *
 * usb_audio_ring.h's SPSC ring (already using __dmb() for RP2350/
 * Cortex-M33 write-buffer safety, not just same-core ISR-vs-mainloop
 * ordering) is what makes the cross-core split safe: the producer side
 * (usb_audio_ring_push(), called from the USB ISR) still runs on core 0,
 * the consumer side (usb_audio_drain_ring(), called from
 * audio_pipeline_fill_block()) now runs on core 1.
 */

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "pico/multicore.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/gpio.h"

#include "tusb.h"

#include "config.h"
#include "usb_descriptors.h"
#include "usb_audio.h"
#include "i2s_output.h"
#include "i2s_input.h"
#include "audio_pipeline.h"
#include "flash_clkdiv.h"

#include "fx_control.h"
#include "fx_delay.h"
#include "fx_reverb.h"
#include "fx_stutter.h"
#include "fx_phaser.h"
#include "fx_djfilter.h"
#include "fx_beatrepeat.h"
#include "leveller.h"

// PCM1808 clock settling: both i2s_output.c's TX and i2s_input.c's RX
// PIO programs sync to the PCM1808's own BCK/LRCLK via a preamble that
// must catch a clean edge to lock correctly. Calling either init
// function immediately at boot does not reliably give the PCM1808's own
// clock generation time to settle after power-on -- confirmed directly:
// omitting this entirely produced constant, un-self-healing garbage
// (or, in a later variant, i2s_output_write_block() spinning forever
// because the TX side never started draining) with no other code change
// involved. This explicitly initializes both pins as plain GPIO inputs
// and waits (bounded, so USB-only operation with no PCM1808 connected
// still works) for LRCLK to actually start toggling, then gives it a
// further settling margin, before either init function runs and
// reconfigures these same pins for their own PIO use.
static void wait_for_pcm1808_clock(void)
{
    gpio_init(I2S_BCK_PIN);
    gpio_set_dir(I2S_BCK_PIN, GPIO_IN);
    gpio_init(I2S_BCK_PIN + 1u);
    gpio_set_dir(I2S_BCK_PIN + 1u, GPIO_IN);

    uint32_t start_us = time_us_32();
    bool last = gpio_get(I2S_BCK_PIN + 1u);
    while (time_us_32() - start_us < 500000u) {   // 500ms timeout
        bool cur = gpio_get(I2S_BCK_PIN + 1u);
        if (cur != last) break;   // LRCLK is toggling
        last = cur;
    }
    sleep_ms(15);   // further settling margin, matching what was observed
                     // to work in practice
}

static void core0_init(void)
{
#if PICO_RP2350
    // Enable flush-to-zero and default-NaN for audio processing. Prevents
    // denormal performance penalty in the FX chain's feedback filters
    // (fx_reverb's comb/allpass, fx_phaser's allpass, fx_djfilter's
    // biquad) as their state decays toward silence -- carried over
    // unchanged from the original main.c, which needed this for the
    // same reason (its own SVF/biquad EQ engine).
    {
        uint32_t fpscr;
        __asm__ volatile("vmrs %0, fpscr" : "=r"(fpscr));
        fpscr |= (1 << 24) | (1 << 25);  // FZ + DN bits
        __asm__ volatile("vmsr fpscr, %0" : : "r"(fpscr));
    }

    // 307.2MHz (VCO 1536 / 5 / 1). Carried over unchanged from the
    // original: every FX effect's PSRAM clock-divisor comments
    // (fx_delay.h, fx_reverb.h, fx_beatrepeat.h) and the 40MHz
    // PICO_DEFAULT_PSRAM_MAX_FREQ setting in this file's sibling
    // CMakeLists.txt assume this exact system clock -- changing it
    // would silently retune every PSRAM QMI divisor to a different
    // real-world frequency than the one actually validated on hardware.
    // (This clock gives an exact-integer PIO clock divider for 48kHz-
    // family rates specifically; config.h currently runs at 96000Hz --
    // see its comment for why -- which also divides this system clock
    // exactly.)
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    busy_wait_ms(10);
    if (!set_sys_clock_hz(307200000, false)) {
        set_sys_clock_hz(150000000, false);
    }

    // Drop flash clock from the ROM default (~102MHz) -- see
    // flash_clkdiv.c. Subsequent flash access goes through this file's
    // own QMI-register wrapper.
    dspi_flash_apply_clkdiv();
#else
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    busy_wait_ms(10);
    set_sys_clock_pll(1536000000, 5, 1);
#endif

    pico_get_unique_board_id_string(usb_descriptor_str_serial, 17);

    // Carried over unchanged from the original main.c, which set this
    // right after boot with no further explanation anywhere in that
    // codebase (no comment, no later read-back, just init-as-output and
    // drive high once). That single-write pattern is typical of a
    // hardware enable/reset-release line (e.g. many I2S DAC/amp boards
    // need their shutdown pin driven high to produce any output at all).
    // Kept out of caution: dropping it risks silently dead audio output
    // on real hardware with no obvious diagnostic, for the cost of one
    // harmless GPIO write if it turns out to be unrelated.
    gpio_init(23); gpio_set_dir(23, GPIO_OUT); gpio_put(23, 1);

    // Bus priority: give DMA precedence over the CPU on shared buses.
    // Carried over unchanged -- matters for this build's own DMA-heavy
    // paths (I2S input's IRQ-less ring, I2S output's own TX ring, and
    // every PSRAM-backed FX effect's QMI traffic).
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    // Both i2s_output.c and i2s_input.c now just watch the PCM1808's
    // externally-driven BCK/LRCLK as inputs (see i2s_output.h's top
    // comment) -- order between them no longer matters for clock
    // generation the way it did when this device generated its own I2S
    // clock. See wait_for_pcm1808_clock()'s own comment for why it runs
    // first.
    wait_for_pcm1808_clock();
    i2s_output_init();
    i2s_input_init();

    usb_sound_card_init();

    // FX chain. fx_control_init() (the dedicated FX UART) must run
    // before any fx_*_init() that could touch PSRAM, matching this
    // ordering's precedent from the original build.
    fx_control_init();
    fx_delay_init();
    fx_reverb_init();
    fx_stutter_init();
    fx_phaser_init();
    fx_djfilter_init();
    fx_beatrepeat_init();

    // Volume Leveller (leveller.h) -- no PSRAM use, ordering relative to
    // the PSRAM-backed effects above doesn't matter, but must run before
    // core 1 launches (multicore_launch_core1(), this function's caller)
    // since audio_pipeline_fill_block() calls leveller_process_block()
    // from the very first block.
    leveller_init();

    // Onboard LED as a physical PSRAM go/no-go signal: solid on only if
    // every PSRAM-backed FX effect confirmed its allocation is actually
    // mapped. Same convention this codebase has used since fx_delay's
    // PSRAM bring-up (see fx_delay.h's history comment) -- carried over
    // unchanged, just with fx_djfilter/fx_stutter/fx_phaser added to the
    // list since they don't use PSRAM and have no psram_ok() of their own.
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN,
             fx_delay_psram_ok() && fx_reverb_psram_ok() && fx_beatrepeat_psram_ok());
}

// Core 1: the audio processing loop. Spin-waiting inside
// i2s_output_write_block() for TX ring space is fine here -- see the
// top-of-file comment for why this can no longer share a core with USB
// servicing (that reasoning applied to pico_audio_i2s_multi's hardcoded-
// blocking give_audio_buffer(); this build no longer uses that API at
// all -- see i2s_output.h's top comment for why -- but the same
// principle holds: whatever this core blocks on, it must never be USB
// servicing).
static void core1_main(void)
{
    static int32_t block_buf[AUDIO_BUFFER_SAMPLES * 2];   // interleaved L,R

    while (true) {
        audio_pipeline_fill_block(block_buf, AUDIO_BUFFER_SAMPLES, SAMPLE_RATE_HZ);
        i2s_output_write_block(block_buf, AUDIO_BUFFER_SAMPLES);
    }
}

int main(void)
{
    core0_init();

    // Core 1 owns the entire audio processing loop (see core1_main() and
    // this file's top comment for why). Core 0 is now free to do
    // nothing but service USB and the FX UART control protocol.
    multicore_launch_core1(core1_main);

    while (true) {
        tud_task();
        // fx_control.c's UART command parser (Set FX/Query FX/etc. over
        // the dedicated FX UART, UART0 GPIO 16/17) is polling-based, not
        // interrupt-driven -- fx_control_poll() must be called
        // regularly for it to process incoming bytes at all. Missing
        // from an earlier revision of this file (the FX UART simply
        // never responded to anything, having no poll call anywhere).
        // Cheap (a few conditional checks and, when a full frame is
        // ready, byte-level parsing of a 9600 baud stream), so calling
        // it every core-0 loop iteration alongside tud_task() costs
        // nothing meaningful against USB servicing.
        fx_control_poll();
    }

    return 0;
}
