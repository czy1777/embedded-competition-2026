idf_component_register(
    SRCS "src/soft_i2c.c"
         "src/ssd1306_oled.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
