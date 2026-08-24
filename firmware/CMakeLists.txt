cmake_minimum_required(VERSION 3.12)

# Default SDK path to submodule location if not set via environment
if (NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
endif()

# Always use local pico-extras (contains modified libraries)
set(PICO_EXTRAS_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-extras CACHE PATH "" FORCE)

# Override the "pico"/"pico2" boards' PICO_FLASH_SPI_CLKDIV=2 default.  DSPi
# runs both platforms at 307.2 MHz, making div=2 yield a 153.6 MHz flash clock
# — above the W25Q080's 104-133 MHz rated max.  Reads survive (single-shot at
# boot) but erase/program can't be clocked reliably at that rate, and preset
# saves silently fail verification.  Divider 6 → 51.2 MHz, safely within spec.
# Must be set BEFORE pico_sdk_init() so it propagates to the boot2 build too.
add_compile_definitions(PICO_FLASH_SPI_CLKDIV=6)
# Note: PICO_FLASH_SPI_CLKDIV only reaches hardware on RP2040 (via boot2's SSI
# setup).  On RP2350, boot2 isn't executed at all by default, and the ROM's own
# XIP setup (CLKDIV≈3) stays in force.  The RP2350 override is done at runtime
# in DSPi/flash_clkdiv.c by writing QMI registers directly.

# PSRAM clock: cap below the SDK's 133MHz default. This exact board/chip
# combination (RP2350B + AP6404L-class PSRAM) has hit audible corruption
# noise on real hardware before -- an earlier session's PSRAM-backed effect
# was noisy at the 133MHz default, then AGAIN at 100MHz (that time delayed
# until the delay buffer's first wraparound -- a periodic, tonal-sounding
# glitch, matching what "high-pitched noise instead of a delay" sounds
# like), and only became reliable at 66MHz there. Started at 66MHz here on
# that basis, but noise persisted even after confirming (via
# fx_delay_psram_ok()/onboard LED) that PSRAM itself is correctly detected
# and mapped -- ruling out wiring and narrowing this specifically to clock/
# timing margin. Lowered further to 40MHz. DSPi runs its system clock at
# 307.2MHz (see PICO_FLASH_SPI_CLKDIV's comment above), so this trades a
# divisor of ~5 (61.44MHz actual) for one of 8 (38.4MHz actual) -- a
# meaningfully different timing point, not just a small nudge.
#
# If still not clean at 40MHz, next steps in order:
#   1. Lower further still (lower cost, try first) -- e.g. 20-25MHz.
#   2. hardware/psram.h's psram_set_params() to manually set divisor/
#      rxdelay/max_select/min_deselect instead of trusting
#      psram_configure_params()'s auto-calculation at any frequency --
#      the earlier session's notes flag this board/PSRAM-chip pairing as
#      having public reports of needing hand-tuned timing, and an
#      auto-calculated rxdelay that's merely close (not exact) for this
#      board's actual trace length is a plausible reason lowering the
#      frequency alone hasn't been sufficient.
# (Plain literal, not `40*MHZ`: CMake silently drops command-line defines
# whose value starts with '(' as function-like-macro syntax.)
add_compile_definitions(PICO_DEFAULT_PSRAM_MAX_FREQ=40000000)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

# We also need PICO EXTRAS
include(pico_extras_import.cmake)

project(pico_playground C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Local multi-instance S/PDIF library (replaces system pico_audio_spdif)
add_subdirectory(pico-extras/src/rp2_common/pico_audio_spdif_multi)

# Multi-instance I2S library (same patterns as S/PDIF multi, independent)
add_subdirectory(pico-extras/src/rp2_common/pico_audio_i2s_multi)

# S/PDIF receiver library (forked from elehobica/pico_spdif_rx, patched for DSPi)
add_subdirectory(pico-extras/src/rp2_common/pico_spdif_rx)

add_subdirectory(DSPi)
