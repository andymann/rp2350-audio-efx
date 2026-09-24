/*
 * i2s_output.h - Externally-clocked I2S output for DSPi (minimal build)
 *
 * This device does NOT generate BCK/LRCLK -- see config.h's
 * I2S_BCK_PIN comment. The connected PCM1808 ADC breakout carries its
 * own onboard crystal and drives BCK/LRCLK itself regardless of its
 * MD0/MD1 header pins' state (confirmed on real hardware: both pins
 * measured ~1.6V DC-average, i.e. actively toggling, with this board's
 * GPIO 14/15 completely disconnected). Having this device ALSO drive
 * those same physical pins -- an earlier, self-clocking architecture
 * this file replaced -- was direct electrical bus contention between
 * two active output drivers, which explained a long-unresolved,
 * persistent PCM1808 noise investigation: constant garbage independent
 * of any capture-logic correctness, unaffected by sample rate or clock-
 * divider precision, because the corruption was on BCK/LRCLK themselves,
 * not anything downstream.
 *
 * This build slaves BOTH the I2S output and i2s_input.c's capture to
 * that external clock -- pico_audio_i2s_multi's audio_i2s_setup()/
 * audio_i2s_connect_extra() (an earlier revision of this file's
 * implementation) assume clock-master operation and can't do this, so
 * this file bypasses that high-level API entirely for a manual PIO+DMA
 * ring, structured like i2s_input.c's own RX ring but for TX:
 * audio_i2s_dataout_extclk.pio (vendored in pico_audio_i2s_multi, wait-
 * driven at divider 1.0, data-pin only) shifts data out on the external
 * BCK falling edges; a self-retriggering DMA ring feeds its TX FIFO from
 * a buffer this file owns, filled by i2s_output_write_block() from core
 * 1's audio processing loop.
 */

#ifndef I2S_OUTPUT_H
#define I2S_OUTPUT_H

#include <stdint.h>

void i2s_output_init(void);

// Write sample_count STEREO FRAMES (interleaved L,R int32_t, 24-in-
// high-bits format -- see audio_pipeline.c's final conversion loop)
// into the TX ring. Spin-waits for free space as needed -- acceptable
// on core 1 (see main.c), which has no other work while doing this.
void i2s_output_write_block(const int32_t *stereo_samples, uint32_t sample_count);

// Feedback-servo inputs (see usb_audio.c's SOF handler): same "total
// words consumed so far" concept as this file's previous, self-clocked
// implementation, now tracking the TX ring's own DMA read position
// (paced by the PCM1808's external BCK, not this device's own clock)
// instead of pico_audio_i2s_multi's consumer pool.
uint32_t i2s_output_words_consumed(void);
uint32_t i2s_output_current_transfer_words(void);
uint8_t  i2s_output_dma_channel(void);

#endif // I2S_OUTPUT_H
