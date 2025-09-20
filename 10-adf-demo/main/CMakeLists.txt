idf_component_register(SRCS "main.c"
    PRIV_REQUIRES spi_flash
    INCLUDE_DIRS "."
    REQUIRES
        audio_pipeline audio_stream
        esp_peripherals audio_hal
        # We also use the legacy I2S driver type via ADF’s i2s_stream
        driver
)
