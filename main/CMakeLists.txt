idf_component_register(SRCS "flog.c" "bt_app_av.c"
                            "bt_app_core.c"
                            "main.c"
                            "bridge.c"
                            "sfx.c"
                            "audio_out.c"
                            "ota_ctl.c"
                            "flog.c"
                    PRIV_REQUIRES esp_driver_i2s esp_driver_gpio bt nvs_flash esp_ringbuf esp_driver_spi esp_partition esp_timer app_update esp_app_format
                    INCLUDE_DIRS ".")

# Write the packed sound bank into the 'audio' partition whenever the app is
# flashed, so sounds and firmware can never end up out of step on a device.
#
# This deliberately consumes the packed artifact, not sounds/wav/*.wav: the
# conversion is a manual step (sounds/pack_sounds.py), and sfx_bank.bin is
# committed. That keeps Python off the build's critical path - a fresh clone
# builds and flashes with no ffmpeg, no venv, no host toolchain beyond IDF -
# at the cost of having to remember to re-run the packer. sounds/README.md
# documents that trade and the workflow around it.
esptool_py_flash_to_partition(flash "audio" "${CMAKE_CURRENT_LIST_DIR}/sounds/sfx_bank.bin")
