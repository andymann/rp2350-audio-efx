/*
 * i2s_output.c - see i2s_output.h
 */

#include "i2s_output.h"
#include "config.h"

#include "pico/audio_i2s_multi.h"
#include "hardware/dma.h"

// Producer buffer count/size: same values the original DSPi used for its
// S/PDIF/I2S producer pools (audio_new_producer_pool(&format,
// AUDIO_BUFFER_COUNT, 192) in usb_audio.c's usb_sound_card_init()).
#define I2S_OUT_PRODUCER_BUFFER_COUNT  8u
#define I2S_OUT_PRODUCER_BUFFER_SAMPLES AUDIO_BUFFER_SAMPLES

// Consumer buffer count: matches the original's SPDIF_CONSUMER_BUFFER_COUNT
// (16, the DMA-side buffer depth pico_audio_i2s_multi actually feeds from).
// Buffer sample count is fixed by the library at
// PICO_AUDIO_I2S_DMA_SAMPLE_COUNT (48) -- audio_i2s_connect_extra()
// reformats whatever pool it's given to that size.
#define I2S_OUT_CONSUMER_BUFFER_COUNT  16u

// This device is the sole owner of PIO0 SM0 / DMA channel 0 -- no S/PDIF,
// ADAT or PDM output exists to share (or contend for) hardware with.
// I2S_OUT_PIO is the BLOCK INDEX (0/1/2), matching audio_i2s_config_t's
// uint8_t pio field -- NOT the PIO pointer type (pio0/pio1/pio2) used
// elsewhere in the SDK for direct pio_*() calls. An earlier revision of
// this file passed the pointer here directly; it happened to produce
// the numerically-correct value after truncation (PIO0_BASE's low byte
// is 0x00) but triggered an int-conversion warning and was fixed to
// remove any doubt about correctness rather than rely on that
// coincidence.
#define I2S_OUT_PIO       0u
#define I2S_OUT_SM        0u
#define I2S_OUT_DMA_CH    0u

static audio_i2s_instance_t i2s_out_inst;
static audio_buffer_pool_t *producer_pool = NULL;

// This project's pico-extras (audio_i2s_multi.c) is a customized fork
// that expects these three globals to exist in the application, not
// upstream pico-extras -- see its i2s_audio_start_dma_transfer() and
// audio_i2s_dma_irq_handler(), which `extern` them directly rather than
// taking them as parameters. All three are diagnostic/metering counters,
// not functionally required for audio to flow correctly:
//   preset_loading  - suppresses overrun counting during a flash preset
//                      load (this build has no preset system, so this
//                      condition can never be true; always false).
//   overruns        - incremented on a detected DMA underrun.
//   pio_samples_dma - incremented once per DMA IRQ (a raw activity
//                      counter). Neither is read anywhere in this
//                      build; they exist purely so the linker is happy.
volatile bool preset_loading = false;
int overruns = 0;
volatile uint32_t pio_samples_dma = 0;

audio_buffer_pool_t *i2s_output_pool(void)
{
    return producer_pool;
}

void i2s_output_init(void)
{
    static audio_format_t producer_format = {
        .format = AUDIO_BUFFER_FORMAT_PCM_S32,
        .sample_freq = SAMPLE_RATE_HZ,
        .channel_count = 2,
    };
    static audio_buffer_format_t producer_buffer_format = {
        .format = &producer_format,
        .sample_stride = 2 * sizeof(int32_t),
    };
    producer_pool = audio_new_producer_pool(&producer_buffer_format,
                                             I2S_OUT_PRODUCER_BUFFER_COUNT,
                                             I2S_OUT_PRODUCER_BUFFER_SAMPLES);

    // Consumer pool: initial format is irrelevant (audio_i2s_connect_extra
    // reformats it for PIO_I2S immediately below); AUDIO_BUFFER_FORMAT_PCM_S32
    // is just a placeholder that satisfies audio_new_consumer_pool()'s
    // parameter, matching pico-extras' own examples.
    static audio_format_t consumer_format_placeholder = {
        .format = AUDIO_BUFFER_FORMAT_PCM_S32,
        .sample_freq = SAMPLE_RATE_HZ,
        .channel_count = 2,
    };
    static audio_buffer_format_t consumer_buffer_format_placeholder = {
        .format = &consumer_format_placeholder,
        .sample_stride = 2 * sizeof(int32_t),
    };
    audio_buffer_pool_t *consumer_pool = audio_new_consumer_pool(
        &consumer_buffer_format_placeholder, I2S_OUT_CONSUMER_BUFFER_COUNT, 48);

    audio_i2s_config_t i2s_cfg = {
        .data_pin       = I2S_DATA_OUT_PIN,
        .clock_pin_base = I2S_BCK_PIN,
        .dma_channel    = I2S_OUT_DMA_CH,
        .pio_sm         = I2S_OUT_SM,
        .pio            = I2S_OUT_PIO,
        .dma_irq        = PICO_AUDIO_I2S_DMA_IRQ,
        .clock_master   = true,     // this device generates BCK/LRCLK
        .external_clock = false,
    };
    audio_i2s_setup(&i2s_out_inst, &producer_format, &i2s_cfg);
    audio_i2s_connect_extra(&i2s_out_inst, producer_pool, false, consumer_pool, NULL);
    audio_i2s_set_enabled(&i2s_out_inst, true);
}

void i2s_output_set_enabled(bool enabled)
{
    audio_i2s_set_enabled(&i2s_out_inst, enabled);
}

uint32_t i2s_output_words_consumed(void)
{
    return i2s_out_inst.words_consumed;
}

uint32_t i2s_output_current_transfer_words(void)
{
    return i2s_out_inst.current_transfer_words;
}

uint8_t i2s_output_dma_channel(void)
{
    return i2s_out_inst.dma_channel;
}
