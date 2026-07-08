idf_component_register(
    SRCS "main.c"
         "app/app_mpu6050.c"
         "app/app_ds18b20.c"
         "app/app_max30102.c"
         "app/max30102_algorithm.c"
         "app/app_ssd1306_oled.c"
         "app/app_spp.c"
    INCLUDE_DIRS "."
                 "app"
    REQUIRES mpu6050 espressif__ds18b20 espressif__onewire_bus max30102 ssd1306_oled spp
)
