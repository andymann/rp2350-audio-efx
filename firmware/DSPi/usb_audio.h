/*
 * USB Audio Interface Header for DSPi (minimal build)
 *
 * Trimmed from the original: no loudness/crossfeed/psybass/leveller (DSP-
 * island features this build doesn't have), no per-channel gain/mute
 * state, no vendor-protocol deferred-command flags, no flash-persisted
 * config, no bulk params, no notification endpoint. Matrix mixer removed
 * too -- USB and I2S input are unconditionally summed together in
 * audio_pipeline.c, there's no routing to configure.
 *
 * Also simplified versus the original: fixed single sample rate
 * (SAMPLE_RATE_HZ in config.h, no runtime negotiation), no USB feedback
 * servo (a fixed nominal feedback value is reported -- see usb_audio.c's
 * top comment for the reasoning and what a real adaptive servo would
 * need). No volume/mute persistence: SET_CUR requests are accepted
 * (so USB Audio Class hosts don't see errors) but don't change anything
 * audible; the FX chain's own dry_wet/param controls are this device's
 * real gain controls.
 */

#ifndef USB_AUDIO_H
#define USB_AUDIO_H

#include "config.h"
#include "pico/types.h"
#include <stdbool.h>

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

void usb_sound_card_init(void);

// True if USB audio packets have arrived recently (within the ring's
// jitter-absorption window) -- used by main.c to decide whether USB is
// currently a live contributor to the mixed input, or silent/disconnected.
bool usb_audio_stream_active(void);

// Drain any USB audio packets waiting in the ring, deinterleaving each
// into out_l/out_r (already-scaled to [-1, 1] float, matching every FX
// effect's expected sample range). Returns the number of stereo frames
// written (0 if the ring was empty or no full block was available).
// max_frames bounds how many frames the caller's out_l/out_r buffers can
// hold (normally AUDIO_BUFFER_SAMPLES).
uint32_t usb_audio_drain_ring(float *out_l, float *out_r, uint32_t max_frames);

#endif // USB_AUDIO_H
