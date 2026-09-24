/*
 * i2s_output.c - see i2s_output.h
 *
 * Manual PIO+DMA TX ring, structured like i2s_input.c's own RX ring but
 * mirrored for output: a self-retriggering DMA ring feeds the PIO TX
 * FIFO from a buffer this file owns (read-address wrap, reload channel
 * rewrites the data channel's read address and retriggers it every lap,
 * same IRQ-less technique as i2s_input.c), and i2s_output_write_block()
 * (called from core 1's audio processing loop) writes new samples into
 * that same ring behind the DMA's read position.
 */

#include "i2s_output.h"
#include "config.h"

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "audio_i2s_dataout_extclk.pio.h"

#include <string.h>

// This device's own PIO/DMA resources -- PIO0 SM0 (same slot an earlier,
// self-clocking implementation used), DMA channels 0 and 3
// (i2s_input.c's RX ring already owns 1 and 2).
#define I2S_OUT_PIO       pio0
#define I2S_OUT_SM        0u
#define I2S_OUT_DMA_DATA    0u
#define I2S_OUT_DMA_RELOAD  3u

// Ring sizing: power of 2 (DMA address wrap), interleaved L/R stereo
// frames. 1024 words = 512 stereo frames = ~5.3ms of headroom at 96kHz,
// comfortably more than one core-1 audio block period needs.
#define TX_RING_WORDS 1024u
#define TX_RING_BITS  12u    // log2(ring bytes) = log2(4096)
#define TX_RING_BYTES (TX_RING_WORDS * 4u)

static int32_t __attribute__((aligned(TX_RING_BYTES))) i2s_tx_ring[TX_RING_WORDS];
static uintptr_t ring_base;
static uint32_t  wr_word = 0;   // software write index (core 1 only)

// Feedback-servo bookkeeping (core 0 only -- see i2s_output_words_consumed()).
static uint32_t last_read_word_pos = 0;
static uint32_t completed_laps = 0;

// Patch the 5-bit GPIO index field of a `wait gpio` instruction (see
// audio_i2s_dataout_extclk.pio's comment: authored with placeholder
// indices 0=BCK, 1=LRCLK, patched here to the board's real pins) --
// same technique as i2s_input.c's patch_wait_gpio().
static inline uint16_t patch_wait_gpio(uint16_t instr, uint8_t pin) {
    return (uint16_t)((instr & ~0x1Fu) | (pin & 0x1Fu));
}

static uint16_t out_prog_ram[
    sizeof(audio_i2s_dataout_extclk_program_instructions) / sizeof(uint16_t)];
static struct pio_program out_prog = {
    .instructions = out_prog_ram,
    .length = 0,
    .origin = -1,
};

static uint32_t pair_read_word(void) {
    return (uint32_t)((dma_hw->ch[I2S_OUT_DMA_DATA].read_addr - ring_base) / 4u) %
           TX_RING_WORDS;
}

void i2s_output_init(void)
{
    ring_base = (uintptr_t)i2s_tx_ring;
    wr_word = 0;
    last_read_word_pos = 0;
    completed_laps = 0;
    memset(i2s_tx_ring, 0, sizeof(i2s_tx_ring));

    // Load + patch the external-clock TX program.
    uint8_t len = audio_i2s_dataout_extclk_program.length;
    memcpy(out_prog_ram, audio_i2s_dataout_extclk_program_instructions,
           (size_t)len * sizeof(uint16_t));
    for (uint8_t i = 0; i < len; i++) {
        uint16_t instr = out_prog_ram[i];
        if ((instr >> 13) != 0x1u) continue;          // not a WAIT
        if (((instr >> 5) & 0x3u) != 0u) continue;    // WAIT source not GPIO
        uint8_t pin = (instr & 0x1Fu) ? (uint8_t)(I2S_BCK_PIN + 1u)
                                      : (uint8_t)I2S_BCK_PIN;
        out_prog_ram[i] = patch_wait_gpio(instr, pin);
    }
    out_prog.length = len;
    int offset = pio_add_program(I2S_OUT_PIO, &out_prog);

    gpio_set_function(I2S_DATA_OUT_PIN, GPIO_FUNC_PIO0);
    // BCK/LRCLK are inputs here -- externally driven by the PCM1808
    // board itself (see this file's top comment). Explicitly force
    // their input path on, matching i2s_input.c's own defensive
    // gpio_set_input_enabled() calls for the same pins.
    gpio_set_input_enabled(I2S_BCK_PIN, true);
    gpio_set_input_enabled(I2S_BCK_PIN + 1u, true);

    audio_i2s_dataout_extclk_program_init(I2S_OUT_PIO, I2S_OUT_SM, offset,
                                          I2S_DATA_OUT_PIN, I2S_BCK_PIN);

    // Start procedure for the DMA ring (same technique as i2s_input.c's
    // proven RX ring, mirrored for TX): reload channel rewrites the data
    // channel's READ address and re-triggers it every time the data
    // channel completes a full lap, forever, with zero IRQs.
    dma_channel_config cb = dma_channel_get_default_config(I2S_OUT_DMA_RELOAD);
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_32);
    channel_config_set_read_increment(&cb, false);
    channel_config_set_write_increment(&cb, false);
    channel_config_set_chain_to(&cb, I2S_OUT_DMA_RELOAD);
    dma_channel_configure(I2S_OUT_DMA_RELOAD, &cb,
                          &dma_hw->ch[I2S_OUT_DMA_DATA].al3_read_addr_trig,
                          &ring_base, 1, false);

    dma_channel_config ca = dma_channel_get_default_config(I2S_OUT_DMA_DATA);
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_32);
    channel_config_set_read_increment(&ca, true);
    channel_config_set_write_increment(&ca, false);
    channel_config_set_ring(&ca, false, TX_RING_BITS);   // false = read ring
    channel_config_set_dreq(&ca, pio_get_dreq(I2S_OUT_PIO, I2S_OUT_SM, true));
    channel_config_set_chain_to(&ca, I2S_OUT_DMA_RELOAD);
    dma_channel_configure(I2S_OUT_DMA_DATA, &ca,
                          &I2S_OUT_PIO->txf[I2S_OUT_SM], i2s_tx_ring,
                          TX_RING_WORDS, true);

    pio_sm_set_enabled(I2S_OUT_PIO, I2S_OUT_SM, true);
}

void i2s_output_write_block(const int32_t *stereo_samples, uint32_t sample_count)
{
    uint32_t words_needed = sample_count * 2u;
    for (uint32_t i = 0; i < words_needed; i++) {
        // Spin until there's a free slot behind the DMA's current read
        // position. Acceptable here: core 1 (the only caller) has no
        // other work while waiting for the externally-paced consumer.
        uint32_t next_wr = (wr_word + 1u) % TX_RING_WORDS;
        while (next_wr == pair_read_word()) {
            tight_loop_contents();
        }
        i2s_tx_ring[wr_word] = stereo_samples[i];
        wr_word = next_wr;
    }
}

uint32_t i2s_output_words_consumed(void)
{
    uint32_t cur = pair_read_word();
    if (cur < last_read_word_pos) completed_laps++;   // wrapped a full lap
    last_read_word_pos = cur;
    return completed_laps * TX_RING_WORDS;
}

uint32_t i2s_output_current_transfer_words(void)
{
    return TX_RING_WORDS;
}

uint8_t i2s_output_dma_channel(void)
{
    return I2S_OUT_DMA_DATA;
}
