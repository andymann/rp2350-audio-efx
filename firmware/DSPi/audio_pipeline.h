/*
 * audio_pipeline.h - USB + I2S input, FX chain, I2S output (minimal build)
 *
 * Pull architecture: unlike the original DSPi (input-driven -- whichever
 * source delivered samples called into the pipeline), this build's I2S
 * output is the system's own clock master, so the output's need for the
 * next buffer drives everything. main.c's loop calls
 * audio_pipeline_fill_block() once per output buffer.
 */

#ifndef AUDIO_PIPELINE_H
#define AUDIO_PIPELINE_H

#include <stdint.h>

// Drain whatever's available from USB + I2S input (summed together --
// see audio_pipeline.c's top comment for why there's no source-select
// here), run it through the FX chain (fx_control.h's slot registry, in
// ascending effect_num order), and write sample_count frames of 24-bit-
// in-32 PCM (pico_audio_i2s_multi's expected producer format -- see
// i2s_output.h) into out_stereo, interleaved L,R,L,R,...
void audio_pipeline_fill_block(int32_t *out_stereo, uint32_t sample_count,
                                uint32_t sample_rate_hz);

#endif // AUDIO_PIPELINE_H
