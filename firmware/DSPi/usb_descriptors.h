/*
 * USB Descriptors for DSPi (minimal build) — TinyUSB / UAC1
 *
 * Trimmed from the original DSPi's usb_descriptors.h: a single Audio
 * Control + Audio Streaming interface pair under an IAD, one alt setting
 * (stereo, 16-bit, fixed 48kHz -- SAMPLE_RATE_HZ in config.h), async
 * isochronous OUT with a feedback IN endpoint. Removed entirely: the
 * vendor interface (WinUSB/MS OS 2.0 auto-binding, notification bulk
 * endpoint -- all in service of the vendor USB control protocol this
 * build doesn't have), the 24-bit alt setting, the RP2350 4/6/8-channel
 * alts, multi-rate support (44.1/48/96kHz), and the debug loopback
 * capture function. See usb_descriptors.c for the full layout comment.
 */

#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include <stdint.h>

#include "tusb.h"
#include "class/audio/audio.h"
#include "config.h"   // SAMPLE_RATE_HZ

// ----------------------------------------------------------------------------
// USB IDs
// ----------------------------------------------------------------------------

#define USB_VENDOR_ID   0x2E8B
#define USB_PRODUCT_ID  0xFEAA
// Distinct from the original DSPi's USB_BCD_DEVICE (0x0203) -- this is a
// different (much smaller) descriptor set, so a host that cached the
// original's descriptors under the same VID/PID/bcdDevice should be made
// to re-read rather than reuse a stale cached descriptor.
#define USB_BCD_DEVICE  0x0300

// ----------------------------------------------------------------------------
// ENDPOINT ADDRESSES
// ----------------------------------------------------------------------------

#define AUDIO_OUT_ENDPOINT  0x01U
#define AUDIO_IN_ENDPOINT   0x82U   // feedback (async iso sync pipe)

// Max iso OUT packet size: stereo, 16-bit, at this build's actual
// SAMPLE_RATE_HZ (config.h), +1 frame of jitter headroom for async rate
// adaptation. Computed from SAMPLE_RATE_HZ rather than a fixed value --
// a fixed value here previously went stale silently every time
// SAMPLE_RATE_HZ changed over the course of this project (44.1kHz, then
// 48kHz, then 96kHz for the PCM1808), since nothing else in the build
// cross-checks the two. At 96kHz specifically, stereo 16-bit audio needs
// 384 bytes/ms -- double a value sized for 48kHz -- which silently caps
// every USB packet at less than half of what a real 96kHz stream needs
// to fit in it. The visible symptom of that specific mismatch was macOS
// reporting "no valid formats available" and 0 input/output channels
// for the whole device, not distorted or truncated audio: a UAC1 format
// descriptor whose endpoint is too small to physically carry its own
// advertised rate is internally inconsistent, and macOS correctly
// refuses the entire format outright rather than trying to stream
// something impossible. Comfortably under the 1023-byte full-speed
// isochronous ceiling at every rate this build supports, and far
// smaller than the original DSPi's own 788/582 (sized for its
// 8-channel/24-bit/96kHz multichannel support, which this build
// doesn't have).
#define AUDIO_EP_MAX_PKT    (((SAMPLE_RATE_HZ + 999u) / 1000u + 1u) * 4u)

// ----------------------------------------------------------------------------
// INTERFACE NUMBERS
// ----------------------------------------------------------------------------

#define ITF_NUM_AUDIO_CONTROL   0
#define ITF_NUM_AUDIO_STREAMING 1
#define ITF_NUM_TOTAL           2

// ----------------------------------------------------------------------------
// UAC1 ENTITY IDs
// ----------------------------------------------------------------------------

#define UAC1_INPUT_TERMINAL_ID   1
#define UAC1_FEATURE_UNIT_ID     2
#define UAC1_OUTPUT_TERMINAL_ID  3

// ----------------------------------------------------------------------------
// UAC1 REQUEST OPCODES (not exposed by TinyUSB — UAC2 constants are UAC2-only)
// ----------------------------------------------------------------------------

#define UAC1_REQ_SET_CUR    0x01
#define UAC1_REQ_GET_CUR    0x81
#define UAC1_REQ_GET_MIN    0x82
#define UAC1_REQ_GET_MAX    0x83
#define UAC1_REQ_GET_RES    0x84

// UAC1 feature unit control selectors
#define UAC1_FU_CTRL_MUTE   0x01
#define UAC1_FU_CTRL_VOLUME 0x02

// UAC1 endpoint control selector
#define UAC1_EP_CTRL_SAMPLING_FREQ 0x01

// ----------------------------------------------------------------------------
// STRING INDICES
// ----------------------------------------------------------------------------

#define STRID_LANGID        0
#define STRID_MANUFACTURER  1
#define STRID_PRODUCT       2
#define STRID_SERIAL        3

// Exported for main.c — populated from chip unique ID at boot.
extern char usb_descriptor_str_serial[17];

// Full configuration descriptor as packed bytes.  Defined in usb_descriptors.c.
extern const uint8_t usb_config_descriptor[];
extern const uint16_t usb_config_descriptor_len;

// Alt 1's endpoint descriptor pointers, resolved at link time so the UAC1
// class driver can call usbd_edpt_iso_activate() without re-walking the
// config on every SET_INTERFACE. Only one alt setting now, so these are
// plain pointers rather than the original's per-alt arrays.
extern const uint8_t *const usb_audio_data_ep_desc;
extern const uint8_t *const usb_audio_fb_ep_desc;

#endif // USB_DESCRIPTORS_H
