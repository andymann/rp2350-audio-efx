/*
 * i2s_input.h - Fixed single-pair I2S input for DSPi (minimal build)
 *
 * Trimmed from the original: one stereo pair only (no 4/6/8-channel
 * fan-out), always runs the plain wait-driven "slave" PIO program
 * (audio_i2s_rx_slave, watching this device's OWN I2S output clock --
 * see i2s_output.h's top comment) rather than the clkmaster or
 * external-clock "checked" variants, which existed for input
 * configurations this build doesn't have (I2S input with no I2S output
 * elsewhere in the system; an externally-clocked I2S source needing
 * framing-slip detection). No dynamic start/stop/resync, no rate
 * detection/servo -- the rate is always SAMPLE_RATE_HZ (config.h),
 * because it's derived from the SAME clock i2s_output.c generates.
 *
 * i2s_output_init() MUST run before i2s_input_init() -- this receiver
 * watches BCK/LRCLK pads that only carry a real clock once the output
 * side is driving them.
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
