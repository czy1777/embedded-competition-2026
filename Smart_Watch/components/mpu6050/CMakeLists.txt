set(MPU6050_SRC "src/mpu6050.c")
idf_component_register(
    SRCS ${MPU6050_SRC}
    INCLUDE_DIRS "include"
    REQUIRES driver esp_common
)
