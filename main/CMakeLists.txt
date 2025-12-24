idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash wifi_provisioning espressif__qrcode esp_driver_rmt esp_driver_gpio esp_adc mqtt
)
