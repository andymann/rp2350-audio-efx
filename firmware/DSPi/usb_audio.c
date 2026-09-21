/*
 * USB Audio Implementation for DSPi (minimal build)
 *
 * Custom UAC1 class driver (TinyUSB's built-in audio class is UAC2-only),
 * following the same driver-callback structure as the original DSPi
 * (init/deinit/reset/open/control_xfer_cb/xfer_cb/sof registered via
 * usbd_app_driver_get_cb()) but drastically simplified: one alt setting
 * (stereo, 16-bit, fixed SAMPLE_RATE_HZ -- see usb_descriptors.h), no
 * vendor interface, no volume/mute persistence (SET_CUR is accepted so
 * hosts don't see errors, but nothing audible changes -- the FX chain's
 * own controls are this device's real gain controls), no 24-bit or
 * multichannel alts.
 *
 * Feedback: a real adaptive servo (usb_feedback_controller.c, reused
 * verbatim from the original -- it's a pure module with no dependencies
 * on anything cut), fed from i2s_output.c's DMA word-consumed counter
 * exactly the way the original's SOF handler fed it from its own I2S/
 * S/PDIF output slots. The fill-servo half of that controller (a
 * secondary correction using live consumer-buffer fill level) is left
 * at a constant "at target" input rather than wired to a real fill
 * count -- the original tracked that via a `get_slot_consumer_fill()`
 * helper built for its multi-slot output architecture, which this
 * single-output build doesn't have an equivalent of. The rate-estimator
 * half (the primary correction, average-drift matching) is fully wired
 * and does the real work.
 */

#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/dma.h"

#include "tusb.h"
#include "device/usbd_pvt.h"
#include "class/audio/audio.h"

#include "usb_audio.h"
#include "usb_descriptors.h"
#include "usb_audio_ring.h"
#include "usb_feedback_controller.h"
#include "i2s_output.h"

// ----------------------------------------------------------------------------
// STATE
// ----------------------------------------------------------------------------

static usb_audio_ring_t audio_ring;
static uint32_t last_push_us = 0;

static usb_feedback_ctrl_t fb_ctrl;
// Nominal feedback value (Q10.14) for SAMPLE_RATE_HZ, used before the servo
// has produced its first real estimate.
static uint32_t nominal_feedback_10_14;

// Endpoint buffers. Must live in RAM; reused across every transfer.
static uint8_t __attribute__((aligned(4))) __not_in_flash("audio_scratch") ep_out_buf[AUDIO_EP_MAX_PKT];
static uint8_t __attribute__((aligned(4))) __not_in_flash("audio_scratch") ep_fb_buf[4];

static uint8_t uac1_ctrl_buf[8];

static struct {
    uint8_t ac_itf;
    uint8_t as_itf;
    uint8_t cur_alt;      // 0 or 1 (only one streaming alt in this build)
    bool    ep_data_open;
    bool    ep_fb_open;
    uint8_t pending_cs;
    uint8_t pending_recipient;
    uint8_t pending_len;
    // Feature-unit state: accepted but not meaningfully applied (see
    // top-of-file comment).
    bool    mute;
    int16_t volume;   // 8.8 fixed-point dB, matches UAC1's wire format
} uac1;

#define MIN_VOLUME       ((int16_t)(-32768))   // -128.0 dB (8.8 fixed-point)
#define MAX_VOLUME       ((int16_t)0)          //    0.0 dB
#define VOLUME_RESOLUTION ((int16_t)256)        //    1.0 dB steps

bool usb_audio_stream_active(void)
{
    // "Active" = a packet arrived within the last 100ms. Generous relative
    // to the ~1ms nominal packet rate; only meant to distinguish "host is
    // actually streaming" from "USB connected but host hasn't started
    // (or has paused) the stream".
    return (time_us_32() - last_push_us) < 100000u;
}

uint32_t usb_audio_drain_ring(float *out_l, float *out_r, uint32_t max_frames)
{
    uint32_t frames_written = 0;

    while (frames_written < max_frames) {
        usb_audio_slot_t *slot = usb_audio_ring_peek(&audio_ring);
        if (!slot) break;

        // 16-bit stereo PCM, little-endian, interleaved L,R,L,R,...
        uint32_t bytes_per_frame = 2u * sizeof(int16_t);
        uint32_t frames_in_slot = slot->data_len / bytes_per_frame;
        uint32_t to_copy = frames_in_slot;
        if (frames_written + to_copy > max_frames) to_copy = max_frames - frames_written;

        const int16_t *pcm = (const int16_t *)slot->data;
        for (uint32_t i = 0; i < to_copy; i++) {
            out_l[frames_written + i] = (float)pcm[2 * i]     * (1.0f / 32768.0f);
            out_r[frames_written + i] = (float)pcm[2 * i + 1] * (1.0f / 32768.0f);
        }
        frames_written += to_copy;

        // Only fully-consumed slots are popped; a slot larger than the
        // remaining space in this call is left for the next call to
        // finish draining (rare: only happens if max_frames doesn't
        // divide evenly into whole packets, which AUDIO_BUFFER_SAMPLES
        // vs. AUDIO_EP_MAX_PKT's framing shouldn't normally produce).
        if (to_copy == frames_in_slot) {
            usb_audio_ring_consume(&audio_ring);
        } else {
            break;
        }
    }

    return frames_written;
}

// ----------------------------------------------------------------------------
// UAC1 CLASS DRIVER
// ----------------------------------------------------------------------------

static void uac1_driver_init(void);
static bool uac1_driver_deinit(void);
static void uac1_driver_reset(uint8_t rhport);
static uint16_t uac1_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len);
static bool uac1_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req);
static bool uac1_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
static void uac1_driver_sof(uint8_t rhport, uint32_t frame_count);

static const usbd_class_driver_t uac1_driver = {
    .name            = "DSPi_UAC1",
    .init            = uac1_driver_init,
    .deinit          = uac1_driver_deinit,
    .reset           = uac1_driver_reset,
    .open            = uac1_driver_open,
    .control_xfer_cb = uac1_driver_control_xfer_cb,
    .xfer_cb         = uac1_driver_xfer_cb,
    .sof             = uac1_driver_sof,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &uac1_driver;
}

static void uac1_driver_init(void) {
    memset(&uac1, 0, sizeof(uac1));
}

static bool uac1_driver_deinit(void) {
    return true;
}

static void uac1_driver_reset(uint8_t rhport) {
    (void)rhport;
    uac1.ep_data_open = false;
    uac1.ep_fb_open = false;
    uac1.cur_alt = 0;
    fb_ctrl_stream_stop(&fb_ctrl);
}

static uint16_t uac1_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    (void)rhport;

    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_AUDIO);
    TU_VERIFY(itf_desc->bInterfaceSubClass == AUDIO_SUBCLASS_CONTROL);
    TU_VERIFY(itf_desc->bAlternateSetting == 0);

    uac1.ac_itf = itf_desc->bInterfaceNumber;

    uint8_t const *p_desc = (uint8_t const *)itf_desc;
    uint8_t const *p_end  = p_desc + max_len;
    uint16_t drv_len = 0;

    drv_len += tu_desc_len(p_desc);
    p_desc += tu_desc_len(p_desc);

    while (p_desc < p_end && tu_desc_type(p_desc) == TUSB_DESC_CS_INTERFACE) {
        drv_len += tu_desc_len(p_desc);
        p_desc += tu_desc_len(p_desc);
    }

#ifdef TUP_DCD_EDPT_ISO_ALLOC
    bool allocated_out = false;
    bool allocated_fb = false;
#endif

    while (p_desc < p_end && tu_desc_type(p_desc) == TUSB_DESC_INTERFACE) {
        tusb_desc_interface_t const *as = (tusb_desc_interface_t const *)p_desc;
        if (as->bInterfaceClass != TUSB_CLASS_AUDIO ||
            as->bInterfaceSubClass != AUDIO_SUBCLASS_STREAMING) {
            break;
        }
        uac1.as_itf = as->bInterfaceNumber;

        drv_len += tu_desc_len(p_desc);
        p_desc += tu_desc_len(p_desc);

        while (p_desc < p_end && tu_desc_type(p_desc) != TUSB_DESC_INTERFACE) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
            if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
                tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p_desc;
                if (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
                    uint8_t ep_addr = ep->bEndpointAddress;
                    if (ep_addr == AUDIO_OUT_ENDPOINT && !allocated_out) {
                        usbd_edpt_iso_alloc(rhport, ep_addr, AUDIO_EP_MAX_PKT);
                        allocated_out = true;
                    } else if (ep_addr == AUDIO_IN_ENDPOINT && !allocated_fb) {
                        usbd_edpt_iso_alloc(rhport, ep_addr, 4);
                        allocated_fb = true;
                    }
                }
            }
#endif
            drv_len += tu_desc_len(p_desc);
            p_desc += tu_desc_len(p_desc);
        }
    }

    usbd_sof_enable(rhport, SOF_CONSUMER_AUDIO, true);
    return drv_len;
}

static inline void uac1_arm_data_out(uint8_t rhport) {
    usbd_edpt_xfer(rhport, AUDIO_OUT_ENDPOINT, ep_out_buf, AUDIO_EP_MAX_PKT);
}

static inline void uac1_arm_feedback(uint8_t rhport) {
    uint32_t fb = fb_ctrl_get_10_14(&fb_ctrl);
    if (fb == 0) fb = nominal_feedback_10_14;
    ep_fb_buf[0] = (uint8_t)(fb & 0xFF);
    ep_fb_buf[1] = (uint8_t)((fb >> 8) & 0xFF);
    ep_fb_buf[2] = (uint8_t)((fb >> 16) & 0xFF);
    ep_fb_buf[3] = 0;
    usbd_edpt_xfer(rhport, AUDIO_IN_ENDPOINT, ep_fb_buf, 3);
}

static bool uac1_apply_alt(uint8_t rhport, uint8_t alt) {
    if (alt > 1) return false;
    if (alt == uac1.cur_alt) return true;   // idempotent SET_INTERFACE

    uac1.cur_alt = alt;

    if (alt == 0) {
        if (uac1.ep_data_open) { usbd_edpt_close(rhport, AUDIO_OUT_ENDPOINT); uac1.ep_data_open = false; }
        if (uac1.ep_fb_open)   { usbd_edpt_close(rhport, AUDIO_IN_ENDPOINT);  uac1.ep_fb_open = false; }
        fb_ctrl_stream_stop(&fb_ctrl);
        usb_audio_ring_flush(&audio_ring);
        return true;
    }

    // alt == 1: start streaming.
    usb_audio_ring_flush(&audio_ring);   // discard any stale packets from before

#ifdef TUP_DCD_EDPT_ISO_ALLOC
    TU_ASSERT(usbd_edpt_iso_activate(rhport, (tusb_desc_endpoint_t const *)usb_audio_data_ep_desc));
    TU_ASSERT(usbd_edpt_iso_activate(rhport, (tusb_desc_endpoint_t const *)usb_audio_fb_ep_desc));
#else
    TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *)usb_audio_data_ep_desc));
    TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *)usb_audio_fb_ep_desc));
#endif
    // Clear stale busy/AVAIL state left over from a previous alt-1 session
    // (see the original DSPi's identical comment/workaround in usb_audio.c).
    usbd_edpt_stall(rhport, AUDIO_OUT_ENDPOINT);
    usbd_edpt_clear_stall(rhport, AUDIO_OUT_ENDPOINT);
    usbd_edpt_stall(rhport, AUDIO_IN_ENDPOINT);
    usbd_edpt_clear_stall(rhport, AUDIO_IN_ENDPOINT);

    uac1.ep_data_open = true;
    uac1.ep_fb_open = true;

    fb_ctrl_reset(&fb_ctrl, nominal_feedback_10_14 << 2);   // Q10.14 -> Q16.16-ish nominal seed

    uac1_arm_data_out(rhport);
    uac1_arm_feedback(rhport);
    return true;
}

static bool uac1_handle_fu_get(uint8_t rhport, tusb_control_request_t const *req) {
    uint8_t cs = TU_U16_HIGH(req->wValue);
    switch (req->bRequest) {
        case UAC1_REQ_GET_CUR:
            if (cs == UAC1_FU_CTRL_MUTE) {
                static uint8_t m; m = uac1.mute ? 1 : 0;
                return tud_control_xfer(rhport, req, &m, 1);
            }
            if (cs == UAC1_FU_CTRL_VOLUME) {
                static int16_t v; v = uac1.volume;
                return tud_control_xfer(rhport, req, &v, 2);
            }
            break;
        case UAC1_REQ_GET_MIN:
            if (cs == UAC1_FU_CTRL_VOLUME) { static int16_t v = MIN_VOLUME; return tud_control_xfer(rhport, req, &v, 2); }
            break;
        case UAC1_REQ_GET_MAX:
            if (cs == UAC1_FU_CTRL_VOLUME) { static int16_t v = MAX_VOLUME; return tud_control_xfer(rhport, req, &v, 2); }
            break;
        case UAC1_REQ_GET_RES:
            if (cs == UAC1_FU_CTRL_VOLUME) { static int16_t v = VOLUME_RESOLUTION; return tud_control_xfer(rhport, req, &v, 2); }
            break;
    }
    return false;
}

static bool uac1_handle_ep_get(uint8_t rhport, tusb_control_request_t const *req) {
    uint8_t cs = TU_U16_HIGH(req->wValue);
    if (req->bRequest == UAC1_REQ_GET_CUR && cs == UAC1_EP_CTRL_SAMPLING_FREQ) {
        static uint8_t freq_bytes[3];
        uint32_t f = SAMPLE_RATE_HZ;
        freq_bytes[0] = (uint8_t)(f & 0xFF);
        freq_bytes[1] = (uint8_t)((f >> 8) & 0xFF);
        freq_bytes[2] = (uint8_t)((f >> 16) & 0xFF);
        return tud_control_xfer(rhport, req, freq_bytes, 3);
    }
    return false;
}

static bool uac1_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (stage == CONTROL_STAGE_SETUP) {
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
            if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
                uint8_t itf = TU_U16_LOW(req->wIndex);
                uint8_t alt = TU_U16_LOW(req->wValue);
                if (itf == uac1.ac_itf) {
                    if (alt != 0) return false;
                    return tud_control_status(rhport, req);
                }
                if (itf == uac1.as_itf) {
                    if (!uac1_apply_alt(rhport, alt)) return false;
                    return tud_control_status(rhport, req);
                }
                return false;
            }
            if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
                uint8_t itf = TU_U16_LOW(req->wIndex);
                static uint8_t alt_resp;
                if (itf == uac1.ac_itf) alt_resp = 0;
                else if (itf == uac1.as_itf) alt_resp = uac1.cur_alt;
                else return false;
                return tud_control_xfer(rhport, req, &alt_resp, 1);
            }
            return false;
        }

        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
            uint8_t recipient = req->bmRequestType_bit.recipient;
            bool is_get = (req->bmRequestType_bit.direction == TUSB_DIR_IN);

            if (recipient == TUSB_REQ_RCPT_INTERFACE) {
                uint8_t itf      = TU_U16_LOW(req->wIndex);
                uint8_t entityID = TU_U16_HIGH(req->wIndex);
                if (itf != uac1.ac_itf || entityID != UAC1_FEATURE_UNIT_ID) return false;
                if (is_get) return uac1_handle_fu_get(rhport, req);
                if (req->bRequest == UAC1_REQ_SET_CUR) {
                    uint16_t len = req->wLength;
                    if (len == 0 || len > sizeof(uac1_ctrl_buf)) return false;
                    uac1.pending_cs        = TU_U16_HIGH(req->wValue);
                    uac1.pending_recipient = TUSB_REQ_RCPT_INTERFACE;
                    uac1.pending_len       = (uint8_t)len;
                    return tud_control_xfer(rhport, req, uac1_ctrl_buf, len);
                }
                return false;
            }

            if (recipient == TUSB_REQ_RCPT_ENDPOINT) {
                uint8_t ep = TU_U16_LOW(req->wIndex);
                if (ep != AUDIO_OUT_ENDPOINT) return false;
                if (is_get) return uac1_handle_ep_get(rhport, req);
                if (req->bRequest == UAC1_REQ_SET_CUR) {
                    uint16_t len = req->wLength;
                    if (len == 0 || len > sizeof(uac1_ctrl_buf)) return false;
                    uac1.pending_cs        = TU_U16_HIGH(req->wValue);
                    uac1.pending_recipient = TUSB_REQ_RCPT_ENDPOINT;
                    uac1.pending_len       = (uint8_t)len;
                    return tud_control_xfer(rhport, req, uac1_ctrl_buf, len);
                }
                return false;
            }
            return false;
        }
        return false;
    }

    if (stage == CONTROL_STAGE_DATA) {
        if (req->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) return true;
        if (uac1.pending_recipient == TUSB_REQ_RCPT_INTERFACE) {
            if (uac1.pending_cs == UAC1_FU_CTRL_MUTE) {
                uac1.mute = uac1_ctrl_buf[0] != 0;
            } else if (uac1.pending_cs == UAC1_FU_CTRL_VOLUME) {
                memcpy(&uac1.volume, uac1_ctrl_buf, sizeof(uac1.volume));
                // Not applied to the audio path -- see top-of-file comment.
            }
        } else if (uac1.pending_recipient == TUSB_REQ_RCPT_ENDPOINT) {
            if (uac1.pending_cs == UAC1_EP_CTRL_SAMPLING_FREQ) {
                uint32_t new_freq = (uint32_t)uac1_ctrl_buf[0]
                                  | ((uint32_t)uac1_ctrl_buf[1] << 8)
                                  | ((uint32_t)uac1_ctrl_buf[2] << 16);
                // Only one rate exists in this build; anything else is a
                // genuine UAC1 protocol violation from the host (it
                // shouldn't be offering any other rate to SET), so stall
                // rather than silently accept.
                if (new_freq != SAMPLE_RATE_HZ) {
                    uac1.pending_recipient = 0;
                    return false;
                }
            }
        }
        uac1.pending_recipient = 0;
        return true;
    }

    return true;
}

static bool __not_in_flash_func(uac1_driver_xfer_cb)(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    if (ep_addr == AUDIO_OUT_ENDPOINT) {
        if (result == XFER_RESULT_SUCCESS && xferred_bytes > 0) {
            last_push_us = time_us_32();
            usb_audio_ring_push(&audio_ring, ep_out_buf,
                                xferred_bytes > 0xFFFFu ? 0xFFFFu : (uint16_t)xferred_bytes);
        }
        if (uac1.ep_data_open) uac1_arm_data_out(rhport);
        return true;
    }
    if (ep_addr == AUDIO_IN_ENDPOINT) {
        if (uac1.ep_fb_open) uac1_arm_feedback(rhport);
        return true;
    }
    return false;
}

static void __not_in_flash_func(uac1_driver_sof)(uint8_t rhport, uint32_t frame_count) {
    (void)rhport;
    (void)frame_count;

    uint32_t consumed = i2s_output_words_consumed();
    uint32_t xfer_words = i2s_output_current_transfer_words();
    uint32_t remaining = dma_channel_hw_addr(i2s_output_dma_channel())->transfer_count;
    uint32_t current_total = consumed + (xfer_words - remaining);

    // rate_shift = 13 matches the original's I2S case (see its SOF
    // handler comment). consumer_fill is a constant "at target" value --
    // see this file's top comment for why the fill-servo half of the
    // controller isn't wired to a real measurement in this build.
    fb_ctrl_sof_update(&fb_ctrl, current_total, 13u, 8u);
}

// ----------------------------------------------------------------------------
// INIT
// ----------------------------------------------------------------------------

void usb_sound_card_init(void) {
    memset(&audio_ring, 0, sizeof(audio_ring));

    fb_ctrl_init(&fb_ctrl);
    // Q10.14 nominal feedback = SAMPLE_RATE_HZ * 2^14 / 1000 (samples per
    // USB frame, full-speed 1ms frames), matching the standard UAC1
    // async feedback encoding.
    nominal_feedback_10_14 = (uint32_t)(((uint64_t)SAMPLE_RATE_HZ << 14) / 1000u);

    tusb_init();

    // Brief settle delay after USB stack init, before anything else in
    // boot touches the bus or starts other DMA/PIO activity. Added after
    // a reported enumeration failure on macOS (device booted fine --
    // LED confirmed -- but never appeared as any USB device) turned out
    // to be a timing-sensitive issue: a diagnostic build with printf()
    // tracing sprinkled through the whole SETUP-handling path (each
    // print costing a few ms over a 115200-baud debug UART) enumerated
    // successfully every time, all the way through SET_CONFIGURATION,
    // interface open, and volume/frequency control negotiation. Removing
    // the tracing reproduced the original failure, confirming a real
    // race rather than a descriptor or driver-logic bug (which had
    // already been verified byte-correct and logically correct via that
    // same tracing). This is the minimal, targeted version of "give it
    // time to settle" rather than leaving debug-print latency in the
    // shipped firmware -- if 2ms here isn't sufficient on its own, the
    // race is more likely to be about spacing between individual SETUP
    // responses than a one-time post-init settle window, which would
    // need a different fix.
    sleep_ms(2);
}
