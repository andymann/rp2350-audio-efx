/*
 * Copyright (c) 2026 Andy
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

// Custom board header for the rp2350-audio-efx project: a QFN-80 RP2350B
// with 8MB of QSPI PSRAM, built as a multi-effects USB audio DSP based on
// WeebLabs/DSPi. Modeled on pico-sdk's weact_studio_rp2350b_core.h (also an
// RP2350B board) — adjust the pin numbers below to match your actual wiring.

#ifndef _BOARDS_RP2350B_AUDIO_EFX_H
#define _BOARDS_RP2350B_AUDIO_EFX_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)

// For board detection
#define RP2350B_AUDIO_EFX

// --- UART ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 12
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 13
#endif

// --- LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 8
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 9
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 0
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 18
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 19
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 16
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 17
#endif

// --- FLASH ---
// TODO: confirm your flash chip. W25Q080-style boot2 assumed, matching
// DSPi's existing firmware/CMakeLists.txt PICO_FLASH_SPI_CLKDIV=6 override
// (needed because DSPi clocks flash SPI faster than the W25Q080's rated
// 104-133MHz at the default divider — see that file's comment).
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

// TODO: set this to your board's actual flash size.
pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (16 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

// --- PSRAM ---
// 8MB QSPI PSRAM, using pico-sdk 2.3.0's hardware_psram driver (see
// firmware/DSPi/CMakeLists.txt for the hardware_psram link, and
// firmware/CMakeLists.txt for the pico-sdk version pin). CS pin follows
// the same convention as WeAct's RP2350B core board and Pimoroni's Pico
// Plus 2 (both also RP2350B + 8MB PSRAM) -- TODO: confirm against your
// actual schematic.
#define RP2350B_AUDIO_EFX_PSRAM_CS_PIN 0

#ifndef PICO_PSRAM_CS_PIN
#define PICO_PSRAM_CS_PIN RP2350B_AUDIO_EFX_PSRAM_CS_PIN
#endif

pico_board_cmake_set_default(PICO_PSRAM_SIZE_BYTES, (8 * 1024 * 1024))
#ifndef PICO_PSRAM_SIZE_BYTES
#define PICO_PSRAM_SIZE_BYTES (8 * 1024 * 1024)
#endif

// --- RP2350 VARIANT ---
// This means RP2350B (QFN-80, more GPIOs/peripherals than the A variant
// used on Pico 2). DSPi's own code only checks PICO_RP2350 (true for both
// A and B), so no DSPi source changes are needed for the B variant.
#define PICO_RP2350A 0

pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
