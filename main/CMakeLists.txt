idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash 
             wifi_provisioning 
             qrcode 
             esp_driver_rmt 
             esp_driver_gpio 
             esp_adc 
             mqtt 
             spiffs 
             vfs          # <-- ADD THIS LINE
)

spiffs_create_partition_image(spiffs certs FLASH_IN_PROJECT)