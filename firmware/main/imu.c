#include "imu.h"
#include <math.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "board.h"

static const char *TAG = "imu";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_mpu, s_mag;
static bool s_mag_ok;
static const int8_t R_IMU[3][3] = IMU_MAP;
static const int8_t R_MAG[3][3] = MAG_MAP;

/* MPU-6050: giroscópio +-500 dps (65.5 LSB/dps), acelerómetro +-4 g (8192 LSB/g) */
#define GYRO_LSB_PER_DPS  65.5f
#define ACC_LSB_PER_G     8192.0f
/* LIS3MDL: +-4 gauss = 6842 LSB/gauss; 1 gauss = 100 uT */
#define MAG_UT_PER_LSB    (100.0f / 6842.0f)

static esp_err_t wr(i2c_master_dev_handle_t d, uint8_t reg, uint8_t v)
{
    uint8_t b[2] = { reg, v };
    return i2c_master_transmit(d, b, 2, 20);
}

static esp_err_t rd(i2c_master_dev_handle_t d, uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(d, &reg, 1, buf, n, 20);
}

static void map3(const int8_t R[3][3], const float in[3], float out[3])
{
    for (int i = 0; i < 3; i++) out[i] = R[i][0] * in[0] + R[i][1] * in[1] + R[i][2] * in[2];
}

esp_err_t imu_init(void)
{
     s_mag_ok = false;
    i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0, .sda_io_num = PIN_I2C_SDA, .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &s_bus));

    i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .scl_speed_hz = I2C_FREQ_HZ };
    dc.device_address = MPU6050_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dc, &s_mpu));
    dc.device_address = LIS3MDL_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dc, &s_mag));

    uint8_t id = 0;
    esp_err_t e = rd(s_mpu, 0x75, &id, 1);                   /* WHO_AM_I */
    if (e != ESP_OK || (id & 0x7E) != 0x68) {
        ESP_LOGE(TAG, "MPU-6050 não responde (WHO_AM_I=0x%02x, err=%s)", id, esp_err_to_name(e));
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(wr(s_mpu, 0x6B, 0x01));                  /* PWR_MGMT_1: acordar, relógio = PLL gyro X */
    ESP_ERROR_CHECK(wr(s_mpu, 0x19, 4));                     /* SMPLRT_DIV: 1 kHz/(1+4) = 200 Hz */
    ESP_ERROR_CHECK(wr(s_mpu, 0x1A, 0x03));                  /* CONFIG: DLPF ~42 Hz */
    ESP_ERROR_CHECK(wr(s_mpu, 0x1B, 0x08));                  /* GYRO_CONFIG: +-500 dps */
    ESP_ERROR_CHECK(wr(s_mpu, 0x1C, 0x08));                  /* ACCEL_CONFIG: +-4 g */

        id = 0;
        e = rd(s_mag, 0x0F, &id, 1);  /* WHO_AM_I = 0x3D */

        if (e == ESP_OK && id == 0x3D) {
            e = wr(s_mag, 0x20, 0x7C);  /* CTRL_REG1 */
            if (e == ESP_OK) e = wr(s_mag, 0x21, 0x00);  /* CTRL_REG2 */
            if (e == ESP_OK) e = wr(s_mag, 0x22, 0x00);  /* CTRL_REG3 */
            if (e == ESP_OK) e = wr(s_mag, 0x23, 0x0C);  /* CTRL_REG4 */
            if (e == ESP_OK) e = wr(s_mag, 0x24, 0x40);  /* CTRL_REG5 */

            if (e == ESP_OK) {
                s_mag_ok = true;
                ESP_LOGI(TAG, "LIS3MDL configurado");
            } else {
                ESP_LOGW(TAG,
                        "Falha ao configurar LIS3MDL: %s; magnetómetro indisponível",
                        esp_err_to_name(e));
            }
        } else {
            ESP_LOGW(TAG,
                    "LIS3MDL indisponível (id=0x%02x, err=%s)",
                    id, esp_err_to_name(e));
        }
    return ESP_OK;
}

esp_err_t imu_read(imu_sample_t *o)
{
    uint8_t b[14];
    esp_err_t e = rd(s_mpu, 0x3B, b, sizeof(b));
    if (e != ESP_OK) return e;
    int16_t v[7];
    for (int i = 0; i < 7; i++) v[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    float acc[3], gyr[3];
    for (int i = 0; i < 3; i++) {
        acc[i] = v[i] / ACC_LSB_PER_G * 9.80665f;
        gyr[i] = v[4 + i] / GYRO_LSB_PER_DPS * (float)M_PI / 180.0f;
    }
    map3(R_IMU, acc, o->accel);
    map3(R_IMU, gyr, o->gyro);
    return ESP_OK;
}

esp_err_t mag_read(float m[3])
{
    if (!s_mag_ok) return ESP_ERR_NOT_FOUND;
    uint8_t b[6];
    esp_err_t e = rd(s_mag, 0x28 | 0x80, b, 6);              /* auto-incremento: bit 7 */
    if (e != ESP_OK) return e;
    float raw[3];
    for (int i = 0; i < 3; i++) raw[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) * MAG_UT_PER_LSB;
    map3(R_MAG, raw, m);
    return ESP_OK;
}

bool imu_mag_present(void) { return s_mag_ok; }
