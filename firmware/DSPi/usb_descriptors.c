/*
 * USB Descriptors for DSPi (minimal build) — TinyUSB / UAC1
 *
 * Hand-rolled UAC1 config descriptor as a packed byte array (TinyUSB's
 * TUD_AUDIO_DESC_* macros emit UAC2-shaped descriptors, so they cannot be
 * used here). Trimmed from the original: single alt setting (stereo,
 * 16-bit, fixed 48kHz), no vendor interface/notification endpoint, no MS
 * OS 2.0 / BOS descriptor (both existed solely to auto-bind WinUSB to the
 * vendor interface, which no longer exists), no 24-bit or multichannel
 * alts, no loopback capture function. The byte-level layout of what
 * remains (AC header/input terminal/feature unit/output terminal, AS
 * alt0/alt1, iso OUT + feedback IN) is unchanged from the original,
 * proven UAC1 structure -- only the removed pieces and the alt count are
 * different.
 *
 * Layout (offsets from start of usb_config_descriptor[]):
 *
 *   0  Config descriptor                               (9 bytes)
 *   9  IAD (covers AC + AS)                             (8 bytes)
 *  17  AC std interface (itf 0, 0 EPs, UAC1)             (9 bytes)
 *  26  AC CS header                                      (9 bytes)
 *  35  AC CS input terminal (ID 1, USB streaming)       (12 bytes)
 *  47  AC CS feature unit (ID 2, mute+volume, 2ch)      (10 bytes)
 *  57  AC CS output terminal (ID 3, speaker)             (9 bytes)
 *  66  AS std interface alt 0 (zero-bw)                  (9 bytes)
 *  75  AS std interface alt 1 (16-bit)                   (9 bytes)
 *  84  AS CS general (wFormatTag=PCM)                    (7 bytes)
 *  91  AS CS format type I (16-bit, 1 fixed rate)       (11 bytes)
 * 102  Std iso EP OUT 0x01                                (9 bytes)
 * 111  CS iso data EP (sampling freq control)             (7 bytes)
 * 118  Std iso feedback EP IN 0x82                        (9 bytes)
 * 127  total
 */

#include <string.h>

#include "tusb.h"
#include "class/audio/audio.h"
#include "pico/unique_id.h"

#include "usb_descriptors.h"
#include "config.h"

// ----------------------------------------------------------------------------
// STRINGS
// ----------------------------------------------------------------------------

char usb_descriptor_str_serial[17] = "0123456789ABCDEF";

static const char *const string_table[] = {
    "",                    // 0 — placeholder; langid returned separately
    "GitHub.com/WeebLabs", // 1 — manufacturer
    "Weeb Labs DSPi (FX)", // 2 — product
    usb_descriptor_str_serial, // 3 — serial (populated at boot)
};

// ----------------------------------------------------------------------------
// DEVICE DESCRIPTOR
// ----------------------------------------------------------------------------

static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    // 0x0200 (plain USB 2.0), not the original's 0x0210: that value
    // existed specifically to make Windows query the BOS descriptor for
    // the MS OS 2.0 Platform Capability (WinUSB auto-binding to the
    // vendor interface). No vendor interface here, so no BOS descriptor
    // is implemented -- advertising 0x0210 would tell a host it's safe
    // to query BOS and then get no response.
    .bcdUSB             = 0x0200,
    // IAD signaling triplet: still correct/expected for a device using
    // an Interface Association Descriptor (our AC+AS pair), independent
    // of the vendor-interface question.
    .bDeviceClass       = TUSB_CLASS_MISC,        // 0xEF
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,   // 0x02
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,      // 0x01
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VENDOR_ID,
    .idProduct          = USB_PRODUCT_ID,
    .bcdDevice          = USB_BCD_DEVICE,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 0x01,
};

// ----------------------------------------------------------------------------
// CONFIGURATION DESCRIPTOR (UAC1 with IAD) — see top-of-file layout comment
// ----------------------------------------------------------------------------

#define UAC1_IN_CHANNELS     2
#define UAC1_CHANNEL_CONFIG  0x0003   // FRONT_LEFT | FRONT_RIGHT

// Feature Unit length = 7 fixed bytes + bControlSize(1) × (1 master + N ch).
#define UAC1_FU_LEN      (7 + (1 + UAC1_IN_CHANNELS))
// CS AC total = header(9) + input terminal(12) + feature unit + output(9).
#define AC_CS_TOTAL_LEN  (9 + 12 + UAC1_FU_LEN + 9)

// One stereo AS alt block, single fixed rate: std(9) + general(7) +
// format-type-I(11, one rate) + iso OUT(9) + CS iso(7) + iso feedback(9).
#define AS_STEREO_ALT_LEN  (9 + 7 + 11 + 9 + 7 + 9)

// config(9) + IAD(8) + AC std(9) + AC CS header(9) + input(12) + FU +
// output(9) + AS alt0(9) + one stereo alt.
#define CONFIG_TOTAL_LEN  (9 + 8 + 9 + 9 + 12 + UAC1_FU_LEN + 9 \
                           + 9 \
                           + AS_STEREO_ALT_LEN)

#define OFFSET_ALT1_DATA_EP (9 + 8 + 9 + 9 + 12 + UAC1_FU_LEN + 9 + 9 + 9 + 7 + 11)
#define OFFSET_ALT1_FB_EP   (OFFSET_ALT1_DATA_EP + 9 + 7)

// Sample rate little-endian expansion
#define RATE_LE(r) ((r) & 0xFF), (((r) >> 8) & 0xFF), (((r) >> 16) & 0xFF)

const uint8_t usb_config_descriptor[] = {
    // --- 0: Config descriptor ---------------------------------------------
    9,                                  // bLength
    TUSB_DESC_CONFIGURATION,            // bDescriptorType
    U16_TO_U8S_LE(CONFIG_TOTAL_LEN),    // wTotalLength
    ITF_NUM_TOTAL,                      // bNumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0x80,                               // bmAttributes (bus-powered)
    0x32,                               // bMaxPower (100 mA)

    // --- 9: IAD grouping AC + AS into one audio function ------------------
    8,                                  // bLength
    TUSB_DESC_INTERFACE_ASSOCIATION,    // 0x0B
    ITF_NUM_AUDIO_CONTROL,              // bFirstInterface (= 0)
    0x02,                               // bInterfaceCount (AC + AS)
    TUSB_CLASS_AUDIO,                   // bFunctionClass
    AUDIO_SUBCLASS_CONTROL,             // bFunctionSubClass
    0x00,                               // bFunctionProtocol (UAC1)
    0x00,                               // iFunction

    // --- 17: AC std interface (itf 0, 0 EPs, UAC1 protocol 0x00) ---------
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_CONTROL,
    0x00,                               // bAlternateSetting
    0x00,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_CONTROL,
    0x00,                               // bInterfaceProtocol (UAC1 = 0x00)
    0x00,

    // --- 26: AC CS header ------------------------------------------------
    9,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_HEADER,       // 0x01
    U16_TO_U8S_LE(0x0100),              // bcdADC (UAC1.0)
    U16_TO_U8S_LE(AC_CS_TOTAL_LEN),
    0x01,                               // bInCollection
    ITF_NUM_AUDIO_STREAMING,            // baInterfaceNr[0]

    // --- 35: AC CS input terminal (ID 1, USB streaming) ------------------
    12,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL, // 0x02
    UAC1_INPUT_TERMINAL_ID,
    U16_TO_U8S_LE(AUDIO_TERM_TYPE_USB_STREAMING),  // 0x0101
    0x00,                               // bAssocTerminal
    UAC1_IN_CHANNELS,
    U16_TO_U8S_LE(UAC1_CHANNEL_CONFIG),
    0x00,                               // iChannelNames
    0x00,                               // iTerminal

    // --- 47: AC CS feature unit (ID 2, master mute+volume, 2ch) ----------
    UAC1_FU_LEN,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, // 0x06
    UAC1_FEATURE_UNIT_ID,
    UAC1_INPUT_TERMINAL_ID,             // bSourceID
    0x01,                               // bControlSize
    0x03,                               // bmaControls[0] master: MUTE|VOLUME
    0x00, 0x00,                         // bmaControls ch 1, ch 2
    0x00,                               // iFeature

    // --- 57: AC CS output terminal (ID 3, speaker) -----------------------
    9,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL, // 0x03
    UAC1_OUTPUT_TERMINAL_ID,
    U16_TO_U8S_LE(AUDIO_TERM_TYPE_OUT_GENERIC_SPEAKER),  // 0x0301
    0x00,                               // bAssocTerminal
    UAC1_FEATURE_UNIT_ID,               // bSourceID
    0x00,

    // --- 66: AS std interface alt 0 (zero-bw) ----------------------------
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING,
    0x00,                               // bAlternateSetting
    0x00,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // --- 75: AS std interface alt 1 (16-bit, only alt) -------------------
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING,
    0x01,                               // bAlternateSetting
    0x02,                               // bNumEndpoints (data + feedback)
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // --- 84: AS CS general -------------------------------------------------
    7,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_AS_GENERAL,   // 0x01
    UAC1_INPUT_TERMINAL_ID,             // bTerminalLink
    0x01,                               // bDelay
    U16_TO_U8S_LE(0x0001),              // wFormatTag = PCM

    // --- 91: AS CS format type I (16-bit, single fixed rate) -------------
    11,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,  // 0x02
    0x01,                               // bFormatType = TYPE_I
    UAC1_IN_CHANNELS,                   // bNrChannels
    0x02,                               // bSubFrameSize (16-bit = 2 bytes)
    16,                                 // bBitResolution
    0x01,                               // bSamFreqType (1 discrete rate)
    RATE_LE(SAMPLE_RATE_HZ),

    // --- 102: Std iso EP OUT 0x01 -----------------------------------------
    9,
    TUSB_DESC_ENDPOINT,
    AUDIO_OUT_ENDPOINT,
    0x05,                               // bmAttributes: iso, async
    U16_TO_U8S_LE(AUDIO_EP_MAX_PKT),
    0x01,                               // bInterval
    0x00,                               // bRefresh
    AUDIO_IN_ENDPOINT,                  // bSynchAddress (feedback EP)

    // --- 111: CS iso data EP -----------------------------------------------
    7,
    TUSB_DESC_CS_ENDPOINT,              // 0x25
    AUDIO_CS_EP_SUBTYPE_GENERAL,        // 0x01
    0x01,                               // bmAttributes: sampling freq control
    0x00,                               // bLockDelayUnits
    U16_TO_U8S_LE(0x0000),

    // --- 118: Std iso feedback EP IN 0x82 ----------------------------------
    9,
    TUSB_DESC_ENDPOINT,
    AUDIO_IN_ENDPOINT,
    0x11,                               // bmAttributes: iso, feedback
    U16_TO_U8S_LE(3),
    0x01,                               // bInterval
    0x02,                               // bRefresh (2^2 = 4 ms)
    0x00,
};

_Static_assert(sizeof(usb_config_descriptor) == CONFIG_TOTAL_LEN,
               "usb_config_descriptor byte count must equal CONFIG_TOTAL_LEN");

const uint16_t usb_config_descriptor_len = CONFIG_TOTAL_LEN;

const uint8_t *const usb_audio_data_ep_desc = &usb_config_descriptor[OFFSET_ALT1_DATA_EP];
const uint8_t *const usb_audio_fb_ep_desc   = &usb_config_descriptor[OFFSET_ALT1_FB_EP];

// ----------------------------------------------------------------------------
// TinyUSB descriptor callbacks
// ----------------------------------------------------------------------------

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&device_descriptor;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return usb_config_descriptor;
}

static uint16_t string_response[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    if (index == STRID_LANGID) {
        string_response[0] = (TUSB_DESC_STRING << 8) | 4;
        string_response[1] = 0x0409;  // English (US)
        return string_response;
    }

    if (index >= TU_ARRAY_SIZE(string_table)) return NULL;
    const char *str = string_table[index];
    if (!str) return NULL;

    size_t len = strlen(str);
    if (len > TU_ARRAY_SIZE(string_response) - 1) len = TU_ARRAY_SIZE(string_response) - 1;
    for (size_t i = 0; i < len; i++) {
        string_response[1 + i] = str[i];
    }
    string_response[0] = (TUSB_DESC_STRING << 8) | (uint8_t)(2 * len + 2);
    return string_response;
}
