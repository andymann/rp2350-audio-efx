/*
 * USB Audio SPSC Ring Buffer
 *
 * Lock-free single-producer single-consumer ring buffer for decoupling
 * the USB isochronous audio packet ISR from main-loop DSP processing.
 *
 * Producer: USB audio packet ISR (_as_audio_packet) on Core 0
 * Consumer: main loop (usb_audio_drain_ring) on Core 0
 *
 * Design follows the PDM ring pattern in pdm_generator.c.  Fixed-slot
 * layout avoids wrap-boundary splits and variable-length allocation.
 *
 * Memory barriers:
 *   RP2040 (Cortex-M0+): volatile alone is sufficient (in-order single-bus).
 *   RP2350 (Cortex-M33): __dmb() required before publishing index updates
 *   and after observing them, due to the write buffer.
 *   Both platforms use __dmb() for portability and documentation of intent.
 */

#ifndef USB_AUDIO_RING_H
#define USB_AUDIO_RING_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "hardware/sync.h"   // __dmb()

// Ring geometry. Sized for THIS build's specific consumption pattern:
// audio_pipeline_fill_block() (the consumer) runs on core 1, gated by a
// hardcoded-blocking call inside pico_audio_i2s_multi (see main.c's top
// comment) to roughly once per audio block (~4.3ms at 44.1kHz/192
// samples) -- not once per USB packet (~1ms) like a tighter-polling
// consumer would. A head/tail ring using "next_head == tail" as its
// full check (this one) can only ever hold SLOTS-1 items before
// appearing full, not SLOTS -- the original 4-slot sizing (from the
// upstream DSPi project this file was carried over from unchanged)
// therefore only actually absorbed ~3ms of jitter, not the 4ms its own
// comment claimed. That was fine for the original's tighter-cadence
// consumer, but for this build's ~4.3ms cadence it was measurably
// insufficient: with ~4.3 packets arriving between drains but only 3
// usable slots to hold them, roughly 68% of frames came through non-
// silent instead of the expected ~100% -- confirmed directly by
// counting real-vs-silent frames. 8 slots (7 usable) gives comfortable
// headroom above the ~4.3 actually needed, at a cost of ~3.1KB more
// SRAM (negligible against this build's large free margin).
#define USB_RING_SLOTS      8
#define USB_RING_SLOT_MASK  (USB_RING_SLOTS - 1)

// Maximum payload per slot.  Must accommodate the largest possible USB audio
// packet, i.e. be >= AUDIO_EP_MAX_PKT (usb_descriptors.h) since the iso OUT EP
// is armed for that many bytes.
//   RP2040 (stereo only): (96kHz/1000 + 1) * 2ch * 3B = 582.
//   RP2350 (+ 8-channel alt): (48kHz/1000 + 1) * 8ch * 2B = 784, rounded to 788.
// The +1 frame accounts for feedback jitter.  Kept in lockstep with
// AUDIO_EP_MAX_PKT (which is platform-conditional for the same reason).
#if PICO_RP2350
#define USB_RING_MAX_PKT    788
#else
#define USB_RING_MAX_PKT    582
#endif

// ---------------------------------------------------------------------------
// Slot and ring structures
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t data_len;                  // Actual byte count this packet
    // 4-byte aligned: the 24-bit USB deinterleave casts `data` to uint32_t* and
    // the compiler may emit `ldrd`, which faults on a non-word-aligned address.
    // Without this, `data` lands at offset 2 (after data_len) and is only
    // 2-byte aligned, hard-faulting on the first 24-bit stereo packet.
    uint8_t  data[USB_RING_MAX_PKT] __attribute__((aligned(4)));  // Raw USB audio payload
} usb_audio_slot_t;

typedef struct {
    usb_audio_slot_t slots[USB_RING_SLOTS];
    volatile uint8_t head;              // Written by USB ISR (producer) only
    volatile uint8_t tail;              // Written by main loop (consumer) only
    volatile uint32_t overrun_count;    // Ring-full drops + oversize drops
} usb_audio_ring_t;

// ---------------------------------------------------------------------------
// Producer — called from USB ISR (must be in RAM for flash safety)
// ---------------------------------------------------------------------------

// Push a USB audio packet into the ring.
// Returns true on success, false if the ring is full or the packet is
// oversize.  On failure, overrun_count is incremented and the packet
// is silently dropped (no partial frames — clamping would produce
// malformed data).
static inline bool __not_in_flash_func(usb_audio_ring_push)(
        usb_audio_ring_t *ring, const uint8_t *data, uint16_t len) {

    // Defensive: reject oversize packets rather than clamping.
    if (len > USB_RING_MAX_PKT) {
        ring->overrun_count++;
        return false;
    }

    uint8_t h = ring->head;
    uint8_t next_h = (h + 1) & USB_RING_SLOT_MASK;

    if (next_h == ring->tail) {
        // Ring full — drop packet.
        ring->overrun_count++;
        return false;
    }

    usb_audio_slot_t *slot = &ring->slots[h];
    slot->data_len = len;
    memcpy(slot->data, data, len);

    // Release barrier: ensure slot data is visible before head advances.
    __dmb();
    ring->head = next_h;

    return true;
}

// ---------------------------------------------------------------------------
// Consumer — called from main loop (thread context)
// ---------------------------------------------------------------------------

// Peek at the next available slot without consuming it.
// Returns a pointer to the slot, or NULL if the ring is empty.
// The returned pointer is valid until usb_audio_ring_consume() is called.
static inline usb_audio_slot_t * __not_in_flash_func(usb_audio_ring_peek)(
        usb_audio_ring_t *ring) {

    if (ring->tail == ring->head)
        return NULL;

    // Acquire barrier: ensure we read slot data written before head advanced.
    __dmb();
    return &ring->slots[ring->tail];
}

// Advance the tail after processing a peeked slot.
// Must be called exactly once per successful peek.
static inline void __not_in_flash_func(usb_audio_ring_consume)(
        usb_audio_ring_t *ring) {

    // Release barrier: ensure all reads of the slot are complete before
    // we advance tail (which frees the slot for the producer).
    __dmb();
    ring->tail = (ring->tail + 1) & USB_RING_SLOT_MASK;
}

// ---------------------------------------------------------------------------
// Lifecycle — called from main loop during stream transitions
// ---------------------------------------------------------------------------

// Discard all pending data.  Used on stream stop/start to flush stale
// packets from a previous stream.
static inline void usb_audio_ring_flush(usb_audio_ring_t *ring) {
    __dmb();  // Consistent with barrier discipline in push/peek/consume
    ring->tail = ring->head;
}

#endif // USB_AUDIO_RING_H
