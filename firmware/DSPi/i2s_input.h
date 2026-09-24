/*
 * i2s_input.h - Fixed single-pair I2S input for DSPi (minimal build)
 *
 * Trimmed from the original: one stereo pair only (no 4/6/8-channel
 * fan-out). Runs the self-verifying "checked" wait-driven PIO program
 * (audio_i2s_rx_slave_checked), watching BCK/LRCLK genuinely driven by
 * the external PCM1808 breakout board -- see i2s_output.h's top comment
 * for why this device does not generate its own I2S clocks. No dynamic
 * start/stop/resync beyond what the checked program's own per-frame
 * LRCLK re-verification provides, no rate detection/servo -- the rate
 * is always SAMPLE_RATE_HZ (config.h), which must match whatever rate
 * the PCM1808's own fixed-frequency onboard crystal actually produces.
 */

#ifndef I2S_INPUT_H
#define I2S_INPUT_H

#include <stdint.h>
#include <stdbool.h>

void i2s_input_init(void);

// Pull up to max_frames stereo frames out of the capture ring into
// out_l/out_r (scaled to [-1, 1] float). Returns the number of frames
// actually written (0 if nothing new has arrived).
uint32_t i2s_input_poll(float *out_l, float *out_r, uint32_t max_frames);

#endif // I2S_INPUT_H
