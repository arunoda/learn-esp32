idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES driver esp_adc
                       INCLUDE_DIRS "")
