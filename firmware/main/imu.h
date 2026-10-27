#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef struct { float gyro[3]; float accel[3]; } imu_sample_t;   /* rad/s, m/s^2, eixos do corpo */

esp_err_t imu_init(void);                    /* I2C + MPU-6050 + LIS3MDL; ESP_OK se o IMU responde */
esp_err_t imu_read(imu_sample_t *out);
esp_err_t mag_read(float mag_uT[3]);         /* eixos do corpo */
bool      imu_mag_present(void);
