idf_component_register(SRCS "bt_app_av.c"
                            "bt_app_core.c"
                            "main.c"
                            "bridge.c"
                    PRIV_REQUIRES esp_driver_i2s bt nvs_flash esp_ringbuf esp_driver_dac esp_driver_spi
                    INCLUDE_DIRS ".")
