PROJECT(DSPi)

add_compile_options(-fcommon)
add_compile_options(-O2)

# Use -O3 for DSP-critical files (the USB/I2S audio path and every FX effect).
set_source_files_properties(
    usb_audio.c audio_pipeline.c i2s_input.c i2s_output.c dac_hw_mute.c fx_delay.c tempo_sync.c fx_stutter.c fx_phaser.c fx_djfilter.c fx_beatrepeat.c fx_reverb.c
    PROPERTIES COMPILE_FLAGS "-O3"
)

# Minimal build: USB + I2S audio input, the FX chain, I2S audio output.
# Removed entirely versus the original DSPi (S/PDIF, ADAT, PDM outputs;
# the crossfeed/crossover/loudness/psybass/leveller/upmix DSP-island
# features; control surfaces, IR remote, bulk parameter transfer,
# flash-persisted presets, the vendor USB control protocol and its UART
# transport, sound-sync, servo control, I2C control, notifications,
# loopback capture, and the test-signal generator): adat_input.c/h,
# adat_output.c/h, spdif_input.c/h, pdm_generator.c/h, crossfeed.c/h,
# crossover.c/h, loudness.c/h, psybass.c/h, leveller.c/h, upmix.c/h,
# lg_sound_sync.c/h, input_servo.c/h, notify.c/h, i2c_control.c/h,
# control_surfaces.c/h, control_surfaces_ir.c/h, control_surfaces_nouns.c,
# bulk_params.c/h, flash_storage.c/h, vendor_commands.c/h,
# uart_control.c/h, loopback.c/h, siggen.c/h.
#
# dsp_pipeline.c/h, dsp_biquad.h, dsp_svf.h, dcp_inline.h are ALSO gone in
# this build (unlike an earlier abandoned attempt at this same trim,
# which kept them as inert infrastructure because they were referenced
# structurally from files that got surgically edited in place) -- this
# build's usb_audio.c/usb_descriptors.c/i2s_input.c were rewritten fresh/
# trimmed from scratch rather than edited in place, so they never pull
# in the PEQ/crossover engine's math to begin with.
add_executable(DSPi
    audio_pipeline.c
    audio_pipeline.h
    config.h
    fx_control.c
    fx_control.h
    fx_delay.c
    fx_delay.h
    fx_reverb.c
    fx_reverb.h
    fx_stutter.c
    fx_stutter.h
    fx_phaser.c
    fx_phaser.h
    fx_djfilter.c
    fx_djfilter.h
    fx_beatrepeat.c
    fx_beatrepeat.h
    flash_clkdiv.c
    flash_clkdiv.h
    i2s_input.c
    i2s_input.h
    i2s_output.c
    i2s_output.h
    main.c
    tempo_sync.c
    tempo_sync.h
    usb_audio.c
    usb_audio.h
    usb_audio_ring.h
    usb_descriptors.c
    usb_descriptors.h
    usb_feedback_controller.c
    usb_feedback_controller.h
    tusb_config.h
)

# XIP execution: cold code runs from flash through the XIP cache; hot
# audio/USB code stays in RAM via .time_critical / __not_in_flash_func
# attributes. scripts/check_ram_placement.py (if present) enforces the
# hot-symbol placement.
pico_set_binary_type(DSPi default)
pico_set_linker_script(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/memmap_dspi_rp2350_xip.ld)

# Board has 8MB QSPI PSRAM (PICO_PSRAM_SIZE_BYTES/PICO_PSRAM_CS_PIN set in
# boards/rp2350b_audio_efx.h); link the driver so runtime_init auto-brings
# it up and __uninitialized_psram() is usable (fx_delay, fx_reverb,
# fx_beatrepeat).
target_link_libraries(DSPi hardware_psram)

pico_generate_pio_header(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/i2s_input.pio)

target_include_directories(DSPi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(DSPi PRIVATE
    # I2S TX on DMA_IRQ_0. No S/PDIF TX/RX in this build to need IRQ
    # separation from, but kept on 0 (rather than removing the define
    # and taking the library's own default) since i2s_output.c and the
    # top-level CMakeLists' PSRAM-clock comments were written assuming
    # this exact value is in force.
    PICO_AUDIO_I2S_DMA_IRQ=0

    # SDK float shims go to .time_critical; every FX effect with a
    # continuously-recomputed filter coefficient (fx_phaser, fx_djfilter,
    # fx_reverb) calls into these every sample.
    PICO_FLOAT_IN_RAM=1
)

target_link_libraries(DSPi
    pico_stdlib
    pico_unique_id
    hardware_flash
    pico_audio_i2s_multi
    tinyusb_device
)

# Debug UART removed; GPIO 16/17 are freed for the FX UART control
# interface (fx_control.c).
pico_enable_stdio_uart(DSPi 0)

pico_add_extra_outputs(DSPi)
