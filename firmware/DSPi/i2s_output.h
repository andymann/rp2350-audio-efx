/*
 * i2s_output.h - Fixed single-instance I2S output for DSPi (minimal build)
 *
 * The original DSPi's output path supported S/PDIF, ADAT, PDM and I2S,
 * runtime-switchable per output slot, with shared DMA channels and a
 * multi-pass teardown/setup dance to move between formats. This build
 * only ever has one output (I2S), so all of that is unnecessary: this is
 * a direct, one-time pico_audio_i2s_multi setup (audio_i2s_setup() +
 * audio_i2s_connect_extra()) with no reconfiguration path.
 *
 * This device is the I2S CLOCK MASTER (drives BCK/LRCLK) -- see
 * config.h's I2S_BCK_PIN/I2S_DATA_OUT_PIN. i2s_input.c's receiver relies
 * on this: it uses the wait-driven "slave" PIO program that watches
 * these same BCK/LRCLK pads rather than generating its own clock, so
 * i2s_output_init() must run before i2s_input_init().
 */

#ifndef I2S_OUTPUT_H
#define I2S_OUTPUT_H

#include <stdint.h>
#include "pico/audio.h"

// Producer buffer pool: fill these with the FX chain's output and give
// them to i2s_output_pool() -- same audio_buffer_pool producer/consumer
// pattern pico-extras uses throughout. PCM_S32 (pico_audio_i2s_multi's
// required producer format), 2 channels, AUDIO_BUFFER_SAMPLES samples
// per buffer.
audio_buffer_pool_t *i2s_output_pool(void);

void i2s_output_init(void);
void i2s_output_set_enabled(bool enabled);

// Feedback-servo inputs (see usb_audio.c's SOF handler): total DMA words
// consumed so far and the current in-flight transfer's word count, read
// directly off the underlying audio_i2s_instance_t exactly like the
// original DSPi's SOF handler does for its own I2S output slots.
uint32_t i2s_output_words_consumed(void);
uint32_t i2s_output_current_transfer_words(void);
uint8_t  i2s_output_dma_channel(void);

#endif // I2S_OUTPUT_H
