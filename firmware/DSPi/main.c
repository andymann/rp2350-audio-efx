/*
 * main.c - DSPi minimal build entry point
 *
 * USB audio in + I2S audio in -> FX chain -> I2S audio out. Trimmed from
 * the original DSPi's main.c (3758 lines: S/PDIF, ADAT, PDM, control
 * surfaces, IR remote, bulk params, flash-persisted presets, the vendor
 * USB control protocol, and several DSP-island features this build
 * doesn't have) down to just what that path needs.
 *
 * Architecture: a PULL loop, not the original's push/event-driven one.
 * The original's various outputs (S/PDIF/ADAT/I2S, runtime-switchable)
 * could each be a clock slave to whichever input was active, so
 * process_input_block() was driven by "whichever input source delivers
 * samples". This build's I2S output is always the system's own clock
 * master (see i2s_output.h), so the natural driver is the output's own
 * need for the next buffer: take a producer buffer, fill it via
 * audio_pipeline_fill_block(), give it back, repeat.
 */

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "pico/audio.h"
#include "pico/stdio_uart.h"
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

    // 307.2MHz (VCO 1536 / 5 / 1) -- integer I2S dividers at 48kHz.
    // Carried over unchanged from the original: every FX effect's PSRAM
    // clock-divisor comments (fx_delay.h, fx_reverb.h, fx_beatrepeat.h)
    // and the 40MHz PICO_DEFAULT_PSRAM_MAX_FREQ setting in this file's
    // sibling CMakeLists.txt assume this exact system clock -- changing
    // it would silently retune every PSRAM QMI divisor to a different
    // real-world frequency than the one actually validated on hardware.
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

    // Initializing UART1 (unrelated pins/peripheral to fx_control.c's FX
    // UART, which uses UART0 on GPIO 16/17) here is required for reliable
    // USB enumeration -- without it, the device intermittently fails to
    // enumerate at all (confirmed not visible as any USB device on
    // macOS). The mechanism is not understood: extensive bisection ruled
    // out the more obvious explanations (see the git history around this
    // line for the full investigation) -- it is not about elapsed time
    // (an equal-duration busy_wait_us() at the points that first seemed
    // load-bearing did not help), and no data is ever transmitted on
    // this UART (write pins otherwise unused). Do not remove this call
    // without re-testing enumeration reliability on real hardware.
    stdio_uart_init_full(uart1, 115200, 4, 5);

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
    // paths (I2S input's IRQ-less ring, I2S output's consumer DMA, and
    // every PSRAM-backed FX effect's QMI traffic).
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    // USB init BEFORE any I2S DMA/PIO activity starts -- reordered from an
    // earlier revision of this file, which called usb_sound_card_init()
    // AFTER i2s_output_init()/i2s_input_init() and failed to enumerate at
    // all on macOS (LED confirmed boot completed fine; the device was
    // simply invisible on the USB bus). The original DSPi calls
    // usb_sound_card_init() as its very first peripheral init, well
    // before i2s_input_init() -- with a comment there noting USB/S/PDIF
    // must come before PDM for an unrelated DMA-channel-claiming reason,
    // but the ordering also has this effect: USB gets to complete its
    // enumeration handshake before any other peripheral's DMA/PIO
    // interrupt traffic starts competing for CPU time. macOS in
    // particular appears to be strict enough about enumeration timing
    // that steady I2S DMA IRQ load already running during the initial
    // SETUP/DATA/STATUS exchange was enough to prevent it from ever
    // completing.
    usb_sound_card_init();

    // I2S output before I2S input: the input's receiver PIO program
    // watches BCK/LRCLK pads that only carry a real clock once the
    // output side is driving them (see i2s_input.h's top comment). This
    // relationship is independent of the USB-ordering fix above and
    // still holds.
    i2s_output_init();
    i2s_input_init();

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

int main(void)
{
    core0_init();

    audio_buffer_pool_t *out_pool = i2s_output_pool();

    while (true) {
        tud_task();

        // Blocking take: normally returns quickly, since the output DMA
        // is steadily consuming and returning buffers to the free list
        // at a fixed ~4ms cadence (AUDIO_BUFFER_SAMPLES @ SAMPLE_RATE_HZ).
        // tud_task() above still runs at least once per loop iteration
        // regardless of how long this blocks.
        audio_buffer_t *buf = take_audio_buffer(out_pool, true);
        if (!buf) continue;

        uint32_t frames = buf->max_sample_count;
        if (frames > AUDIO_BUFFER_SAMPLES) frames = AUDIO_BUFFER_SAMPLES;

        audio_pipeline_fill_block((int32_t *)buf->buffer->bytes, frames, SAMPLE_RATE_HZ);
        buf->sample_count = frames;

        give_audio_buffer(out_pool, buf);
    }

    return 0;
}
