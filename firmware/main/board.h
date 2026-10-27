    #pragma once
/* Pinout do slide "How to start - Hardware" (Seeed XIAO ESP32-S3 Sense) */
#define PIN_BAT_ADC      1   /* D0  GPIO1  ADC1_CH0, divisor 100k/13k */
#define PIN_BUZZER       2   /* D1  GPIO2  saída digital, activo alto */
#define PIN_CAN_CS       3   /* D2  GPIO3  MCP2518FD CS (pull-up externo 10k a 3.3 V) */
#define PIN_CAN_INT      4   /* D3  GPIO4  MCP2518FD INT */
#define PIN_I2C_SDA      5   /* D4  GPIO5  MPU-6050 + LIS3MDL */
#define PIN_I2C_SCL      6   /* D5  GPIO6 */
#define PIN_SPI_SCK      7   /* D8  GPIO7 */
#define PIN_SPI_MISO     8   /* D9  GPIO8 */
#define PIN_SPI_MOSI     9   /* D10 GPIO9 */

#define BAT_DIV_R_TOP    100000.0f
#define BAT_DIV_R_BOT    13000.0f

#define I2C_FREQ_HZ      400000
#define MPU6050_ADDR     0x68   /* AD0 = GND */
#define LIS3MDL_ADDR     0x1E   /* SA1 = VDD (placa Pololu); 0x1C se SA1 = GND */

/* Matriz sensor -> corpo (x frente, y direita, z baixo). Por omissão: chip a plano, z para cima,
 * x para a frente  =>  diag(+1, -1, -1).  VERIFICAR: rodar o cubo no sentido horário visto de cima
 * tem de dar giroscópio z > 0.  O magnetómetro usa a mesma matriz se estiver co-orientado. */
#define IMU_MAP  { {1, 0, 0}, {0, -1, 0}, {0, 0, -1} }
#define MAG_MAP  { {1, 0, 0}, {0, -1, 0}, {0, 0, -1} }

#define CONTROL_HZ       200
#define MAG_DIV          4      /* magnetómetro a 50 Hz */
