/*
 * i2s_input.c - see i2s_input.h
 *
 * Single-pair capture, running the self-verifying "checked" PIO program
 * (audio_i2s_rx_slave_checked in i2s_input.pio) against BCK/LRCLK
 * genuinely externally driven by the connected PCM1808 breakout board
 * (see i2s_output.h's top comment for why: that board carries its own
 * onboard crystal and drives these pins itself, regardless of its
 * MD0/MD1 header pins' state). IRQ-less self-retriggering DMA ring, same
 * technique as the original DSPi's i2s_input.c: a data channel moves PIO
 * RX FIFO words into a power-of-2 ring (write-address wrap) and chains
 * to a reload channel that rewrites the data channel's write address and
 * retriggers it -- capture survives IRQ-disabled windows with zero IRQs.
 *
 * The "checked" variant (re-derives LRCLK framing every single frame,
 * autocorrects on any detected slip) is the right choice for a
 * genuinely external clock source like this one -- unlike the plainer
 * "slave" variant (syncs once at boot, then free-runs forever with no
 * further verification), which produced constant, un-self-healing
 * garbage in practice under the mistaken assumption that this device
 * itself generated BCK/LRCLK.
 */

#include "i2s_input.h"
#include "config.h"

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "i2s_input.pio.h"

#include <string.h>

// This device's own PIO/DMA resources for capture -- separate PIO block
// and DMA channels from i2s_output.c's (PIO0 SM0 / DMA channel 0), no
// sharing needed since there's no S/PDIF/ADAT input to contend with.
#define I2S_IN_PIO      pio1
#define I2S_IN_SM       0u
#define I2S_IN_DMA_DATA   1u
#define I2S_IN_DMA_RELOAD 2u

// Ring sizing: power of 2 (DMA address wrap) and even (L/R parity).
// 1024 words = 512 stereo frames = ~5.3ms of headroom @ 96kHz, matching
// the original's RP2350 sizing philosophy (a few ms to survive main-loop
// stalls without the ring wrapping).
#define I2S_RING_WORDS 1024u
#define I2S_RING_BITS  12u    // log2(ring bytes) = log2(4096)
#define I2S_RING_BYTES (I2S_RING_WORDS * 4u)

static uint32_t __attribute__((aligned(I2S_RING_BYTES))) i2s_rx_ring[I2S_RING_WORDS];
static uintptr_t ring_base;
static uint32_t  rd_word = 0;   // software read index into the ring

// Patch the 5-bit GPIO index field of a `wait gpio` instruction (see
// i2s_input.pio's comment: the slave program is authored with placeholder
// indices 0=BCK, 1=LRCLK, patched here to the board's real pins).
static inline uint16_t patch_wait_gpio(uint16_t instr, uint8_t pin) {
    return (uint16_t)((instr & ~0x1Fu) | (pin & 0x1Fu));
}

static uint16_t slave_prog_ram[
    sizeof(audio_i2s_rx_slave_checked_program_instructions) / sizeof(uint16_t)];
static struct pio_program slave_prog = {
    .instructions = slave_prog_ram,
    .length = 0,
    .origin = -1,
};

static uint32_t pair_write_word(void) {
    return (uint32_t)((dma_hw->ch[I2S_IN_DMA_DATA].write_addr - ring_base) / 4u) %
           I2S_RING_WORDS;
}

void i2s_input_init(void)
{
    ring_base = (uintptr_t)i2s_rx_ring;
    rd_word = 0;
    memset(i2s_rx_ring, 0, sizeof(i2s_rx_ring));

    // Load + patch the checked slave program.
    uint8_t len = audio_i2s_rx_slave_checked_program.length;
    memcpy(slave_prog_ram, audio_i2s_rx_slave_checked_program_instructions,
           (size_t)len * sizeof(uint16_t));
    for (uint8_t i = 0; i < len; i++) {
        uint16_t instr = slave_prog_ram[i];
        if ((instr >> 13) != 0x1u) continue;          // not a WAIT
        if (((instr >> 5) & 0x3u) != 0u) continue;    // WAIT source not GPIO
        uint8_t pin = (instr & 0x1Fu) ? (uint8_t)(I2S_BCK_PIN + 1u)
                                      : (uint8_t)I2S_BCK_PIN;
        slave_prog_ram[i] = patch_wait_gpio(instr, pin);
    }
    slave_prog.length = len;
    int offset = pio_add_program(I2S_IN_PIO, &slave_prog);

    gpio_set_function(I2S_DATA_IN_PIN, GPIO_FUNC_PIO1);
    // BCK/LRCLK are inputs here -- externally driven by the PCM1808
    // board itself (see i2s_output.h's top comment). Explicitly force
    // their input path on, matching the original DSPi project's own
    // i2s_input.c ("Make sure the BCK/LRCLK input buffers are on (belt
    // and braces in the on-chip slave role)") -- this device configuring
    // a pin as a PIO output elsewhere is not a guarantee that a
    // DIFFERENT PIO block's input path for that same pin stays enabled.
    gpio_set_input_enabled(I2S_BCK_PIN, true);
    gpio_set_input_enabled(I2S_BCK_PIN + 1u, true);

    audio_i2s_rx_slave_checked_program_init(I2S_IN_PIO, I2S_IN_SM, offset,
                                             I2S_DATA_IN_PIN, I2S_BCK_PIN + 1u);
    // Explicit divider=1.0, matching the original DSPi project's own
    // defensive setting rather than assuming the SM's default config
    // already has it.
    pio_sm_set_clkdiv_int_frac(I2S_IN_PIO, I2S_IN_SM, 1, 0);

    // Start procedure for the DMA ring (same technique as i2s_output's
    // proven pattern): reload channel rewrites the data channel's write
    // address and re-triggers it every time the data channel completes a
    // full lap, forever, with zero IRQs.
    dma_channel_config cb = dma_channel_get_default_config(I2S_IN_DMA_RELOAD);
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_32);
    channel_config_set_read_increment(&cb, false);
    channel_config_set_write_increment(&cb, false);
    channel_config_set_chain_to(&cb, I2S_IN_DMA_RELOAD);
    dma_channel_configure(I2S_IN_DMA_RELOAD, &cb,
                          &dma_hw->ch[I2S_IN_DMA_DATA].al2_write_addr_trig,
                          &ring_base, 1, false);

    dma_channel_config ca = dma_channel_get_default_config(I2S_IN_DMA_DATA);
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_32);
    channel_config_set_read_increment(&ca, false);
    channel_config_set_write_increment(&ca, true);
    channel_config_set_ring(&ca, true, I2S_RING_BITS);
    channel_config_set_dreq(&ca, pio_get_dreq(I2S_IN_PIO, I2S_IN_SM, false));
    channel_config_set_chain_to(&ca, I2S_IN_DMA_RELOAD);
    dma_channel_configure(I2S_IN_DMA_DATA, &ca,
                          i2s_rx_ring, &I2S_IN_PIO->rxf[I2S_IN_SM],
                          I2S_RING_WORDS, true);

    pio_sm_set_enabled(I2S_IN_PIO, I2S_IN_SM, true);
}

uint32_t i2s_input_poll(float *out_l, float *out_r, uint32_t max_frames)
{
    uint32_t wr_word = pair_write_word();
    // Available words between rd_word and wr_word (mod ring size), rounded
    // down to a whole number of stereo frames (2 words).
    uint32_t avail_words = (wr_word + I2S_RING_WORDS - rd_word) % I2S_RING_WORDS;
    uint32_t avail_frames = avail_words / 2u;
    if (avail_frames > max_frames) avail_frames = max_frames;

    for (uint32_t i = 0; i < avail_frames; i++) {
        uint32_t l_word = i2s_rx_ring[rd_word];
        rd_word = (rd_word + 1u) % I2S_RING_WORDS;
        uint32_t r_word = i2s_rx_ring[rd_word];
        rd_word = (rd_word + 1u) % I2S_RING_WORDS;

        // Audio is in bits [31:8] of each pushed word (see i2s_input.pio's
        // top comment); bits [7:0] are whatever the source drove after
        // the 24th bit, masked off here. Treat as 24-bit signed, scale to
        // float [-1, 1].
        int32_t l24 = (int32_t)l_word >> 8;
        int32_t r24 = (int32_t)r_word >> 8;
        out_l[i] = (float)l24 * (1.0f / 8388608.0f);   // 2^23
        out_r[i] = (float)r24 * (1.0f / 8388608.0f);
    }

    return avail_frames;
}
