#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "cube_core3.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef BENCH_CAN_TELEMETRY
#include "wheel_telemetry.h"
#endif
#ifdef BENCH_USB_CAMERA
#include "bench_camera.h"
#endif

enum {
    IMU_ADDRESS = 0x69,
    IMU_ID = 0x70,
    SDA_PIN = 5,
    SCL_PIN = 6,
    I2C_FREQ_HZ = 100000,
    TIMEOUT_MS = 20,
    PERIOD_MS = 50,
    REG_SMPLRT_DIV = 0x19,
    REG_PWR_MGMT_1 = 0x6b,
    REG_WHO_AM_I = 0x75,
    REG_ACCEL_XOUT_H = 0x3b,
};

static const char *TAG = "bench_sensors";
static const double ACCEL_SCALE = 9.80665 / 8192.0;
static const double GYRO_SCALE = 3.141592653589793 / (180.0 * 65.5);
static cc3_t attitude;

static void publish_attitude(int64_t sample_us, cc_imu_status_t status)
{
    float q[4] = {0};
    float rates[3] = {0};
    unsigned validity = 0;
    if (status != CC_IMU_OK) {
        validity = 2;
    } else if (attitude.calibrated) {
        validity = 1;
        cc3_get_attitude(&attitude, q);
        for (unsigned axis = 0; axis < 3; ++axis) {
            rates[axis] = attitude.rate[axis];
        }
    }
    printf("BENCH_ATTITUDE {\"schema\":1,\"sample_us\":%" PRIi64
           ",\"sent_us\":%" PRIi64 ",\"state\":%u,\"validity\":%u,"
           "\"yaw_reference\":\"relative\",\"q\":[%.9f,%.9f,%.9f,%.9f],"
           "\"rates\":[%.7f,%.7f,%.7f]}\n",
           sample_us, esp_timer_get_time(), (unsigned)attitude.mode, validity,
           q[0], q[1], q[2], q[3], rates[0], rates[1], rates[2]);
}

static esp_err_t read_registers(i2c_master_dev_handle_t device, uint8_t first,
                                uint8_t *data, size_t size)
{
    return i2c_master_transmit_receive(device, &first, 1, data, size, TIMEOUT_MS);
}

static esp_err_t init_mpu6500(i2c_master_dev_handle_t device)
{
    uint8_t identity;
    esp_err_t error = read_registers(device, REG_WHO_AM_I, &identity, 1);
    if (error != ESP_OK) {
        return error;
    }
    if (identity != IMU_ID) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02x, expected 0x%02x", (unsigned)identity, IMU_ID);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint8_t power_mode[] = {REG_PWR_MGMT_1, 0x01, 0x00};
    error = i2c_master_transmit(device, power_mode, sizeof power_mode, TIMEOUT_MS);
    if (error != ESP_OK) {
        return error;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    const uint8_t setup[] = {
        REG_SMPLRT_DIV,
        9,    // 100 Hz internal sample rate from the filtered 1 kHz clock.
        5,    // Gyroscope low-pass filter, DLPF_CFG.
        0x08, // Gyroscope full scale ±500 degrees per second, FCHOICE_B=0.
        0x08, // Accelerometer full scale ±4 g.
        5,    // Accelerometer low-pass filter, A_DLPFCFG.
    };
    return i2c_master_transmit(device, setup, sizeof setup, TIMEOUT_MS);
}

static int16_t read_i16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

void app_main(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    i2c_master_bus_handle_t bus;
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_new_master_bus(&bus_config, &bus)) != ESP_OK) {
        return;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDRESS,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t sensor;
    esp_err_t error = i2c_master_bus_add_device(bus, &device_config, &sensor);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(error) != ESP_OK) {
        i2c_del_master_bus(bus);
        return;
    }
    error = init_mpu6500(sensor);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(error) != ESP_OK) {
        i2c_master_bus_rm_device(sensor);
        i2c_del_master_bus(bus);
        return;
    }

    cc3_config_t config;
    cc3_config_default(&config);
    config.stationary_limit = 0.20f; // Covers the measured uncorrected bias.
    config.axis[2].calib_s = 2.0f;
    config.axis[2].imu_timeout_us = 150000; // Three nominal sample periods.
    for (unsigned axis = 0; axis < 3; ++axis) {
        config.axis[axis].use_mag = 0;
    }
    if (!cc3_init(&attitude, &config)) {
        ESP_LOGE(TAG, "Could not configure bench attitude estimator");
        return;
    }
    ESP_LOGI(TAG, "MPU-6500 0x69, USB IMU stream %d Hz; keep still for relative yaw reference",
             1000 / PERIOD_MS);
#ifdef BENCH_CAN_TELEMETRY
    wheel_telemetry_start();
#endif
#ifdef BENCH_USB_CAMERA
    bench_camera_start();
#endif
    unsigned samples = 0;
    unsigned errors = 0;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        uint8_t data[14];
        error = read_registers(sensor, REG_ACCEL_XOUT_H, data, sizeof data);
        const int64_t sample_us = esp_timer_get_time();
        cc3_meas_t input = {0};
        if (error == ESP_OK) {
            const double ax = read_i16(data) * ACCEL_SCALE;
            const double ay = read_i16(data + 2) * ACCEL_SCALE;
            const double az = read_i16(data + 4) * ACCEL_SCALE;
            const double wx = read_i16(data + 8) * GYRO_SCALE;
            const double wy = read_i16(data + 10) * GYRO_SCALE;
            const double wz = read_i16(data + 12) * GYRO_SCALE;
            ESP_LOGI(TAG,
                     "RAW t=%" PRIi64 " imu=%u mag=0 errors=%u a=%.5f,%.5f,%.5f w=%.6f,%.6f,%.6f",
                     sample_us, ++samples, errors, ax, ay, az, wx, wy, wz);
            // Sensor to body rotation matches the installed cube_adcs build.
            input.accel[0] = -ax;
            input.accel[1] = -az;
            input.accel[2] = -ay;
            input.gyro[0] = -wx;
            input.gyro[1] = -wz;
            input.gyro[2] = -wy;
        } else {
            ++errors;
            if (errors == 1 || errors % 20 == 0) {
                ESP_LOGW(TAG, "IMU read: %s, total errors=%u", esp_err_to_name(error), errors);
            }
        }
        cc3_out_t unused_motor_outputs;
        const cc_imu_status_t status = cc3_step_with_imu(&attitude, sample_us, &input,
                                                         error == ESP_OK, &unused_motor_outputs);
        publish_attitude(sample_us, status);
        if (status == CC_IMU_RECAL_REQUIRED) {
            ESP_LOGW(TAG, "Sample gap; keep still while the relative yaw reference is reset");
            cc3_init(&attitude, &config);
        }
        if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_MS)) == pdFALSE) {
            last_wake = xTaskGetTickCount();
        }
    }
}
