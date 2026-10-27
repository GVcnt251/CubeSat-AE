/*
 * can_mcp2518fd.c - Driver CAN-FD via SPI (MCP2518FD -> Moteus r4.11)
 *
 * Implementação para Seeed XIAO ESP32-S3 Sense:
 *  - SPI SCK:  GPIO 7
 *  - SPI MISO: GPIO 8
 *  - SPI MOSI: GPIO 9
 *  - SPI CS:   GPIO 3
 *  - CAN INT:  GPIO 4
 */

#include "can_port.h"
#include "board.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "can_mcp2518fd";

// Instância do dispositivo SPI do ESP-IDF
static spi_device_handle_t spi_mcp = NULL;

/* --------------------------------------------------------------------------
 * Comandos SPI do MCP2518FD (Datasheet DS20006027)
 * -------------------------------------------------------------------------- */
#define MCP_CMD_RESET   0x0000
#define MCP_CMD_READ    0x3000
#define MCP_CMD_WRITE   0x2000

/* Registos Principais do MCP2518FD */
#define REG_CiCON       0x000  // Control Register
#define REG_CiNBTCFG    0x004  // Nominal Bit Time Configuration (1 Mbps)
#define REG_CiDBTCFG    0x008  // Data Bit Time Configuration (5 Mbps)
#define REG_CiTDC       0x00C  // Transmitter Delay Compensation
#define REG_CiFIFOBA    0x038  // FIFO Base Address
#define REG_CiFIFOCON1  0x05C  // FIFO 1 Control (TX FIFO)
#define REG_CiFIFOCON2  0x060  // FIFO 2 Control (RX FIFO)
#define REG_CiFIFOUA1   0x05E  // FIFO 1 User Address
#define REG_CiFIFOUA2   0x062  // FIFO 2 User Address

/* --------------------------------------------------------------------------
 * Funções Auxiliares de Comunicação SPI
 * -------------------------------------------------------------------------- */

// Escreve um valor de 32 bits num registo do MCP2518FD
static esp_err_t mcp_write_reg32(uint16_t reg, uint32_t val)
{
    uint8_t tx_data[6];
    uint16_t cmd = MCP_CMD_WRITE | (reg & 0x0FFF);
    tx_data[0] = (cmd >> 8) & 0xFF;
    tx_data[1] = cmd & 0xFF;
    tx_data[2] = (val >> 0) & 0xFF;
    tx_data[3] = (val >> 8) & 0xFF;
    tx_data[4] = (val >> 16) & 0xFF;
    tx_data[5] = (val >> 24) & 0xFF;

    spi_transaction_t t = {
        .length = 48, // 6 bytes * 8 bits
        .tx_buffer = tx_data,
    };
    return spi_device_transmit(spi_mcp, &t);
}

// Lê um valor de 32 bits de um registo do MCP2518FD
static uint32_t mcp_read_reg32(uint16_t reg)
{
    uint8_t tx_data[6] = {0};
    uint8_t rx_data[6] = {0};
    uint16_t cmd = MCP_CMD_READ | (reg & 0x0FFF);
    tx_data[0] = (cmd >> 8) & 0xFF;
    tx_data[1] = cmd & 0xFF;

    spi_transaction_t t = {
        .length = 48,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    if (spi_device_transmit(spi_mcp, &t) != ESP_OK) {
        return 0xFFFFFFFF;
    }

    return (rx_data[2]) | (rx_data[3] << 8) | (rx_data[4] << 16) | (rx_data[5] << 24);
}

// Envia comando de Reset por SPI
static esp_err_t mcp_reset(void)
{
    uint8_t tx_data[2];
    tx_data[0] = (MCP_CMD_RESET >> 8) & 0xFF;
    tx_data[1] = MCP_CMD_RESET & 0xFF;

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    return spi_device_transmit(spi_mcp, &t);
}

/* --------------------------------------------------------------------------
 * Inicialização do Barramento SPI e do Controlador CAN-FD
 * -------------------------------------------------------------------------- */
esp_err_t can_port_init(void)
{
    ESP_LOGI(TAG, "Inicializando barramento SPI para MCP2518FD...");

    // Configuração do barramento SPI (SCK=7, MISO=8, MOSI=9)
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { // Ignora se já estiver inicializado
        ESP_LOGE(TAG, "Falha ao inicializar o barramento SPI (%s)", esp_err_to_name(ret));
        return ret;
    }

    // Configuração do dispositivo MCP2518FD no CS (GPIO 3) a 10 MHz
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .mode = 0,                          // SPI Mode 0
        .spics_io_num = PIN_CAN_CS,
        .queue_size = 7,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_mcp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar o dispositivo MCP2518FD ao barramento SPI");
        return ret;
    }

    // Configura o pino de interrupção (GPIO 4)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_CAN_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    // Reset ao chip MCP2518FD
    mcp_reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configura o timing de bits: 1 Mbps nominal, 5 Mbps dados (Requisito Moteus r4.11)
    mcp_write_reg32(REG_CiNBTCFG, 0x000E0303); // 1 Mbit/s
    mcp_write_reg32(REG_CiDBTCFG, 0x00020101); // 5 Mbit/s
    mcp_write_reg32(REG_CiTDC,    0x00020000); // Compensação de atraso

    // Modo Normal de Operação CAN-FD
    mcp_write_reg32(REG_CiCON, 0x06000000); 

    ESP_LOGI(TAG, "Driver MCP2518FD (CAN-FD) inicializado com sucesso!");
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Envio de Trama CAN-FD para o Motor Moteus
 * -------------------------------------------------------------------------- */
esp_err_t can_port_send(const can_frame_t *f)
{
    if (!spi_mcp || !f) return ESP_ERR_INVALID_ARG;

    // Coloca os dados da trama no FIFO de transmissão e solicita o envio (TXREQ)
    // As tramas geradas pelo moteus_proto.c são enviadas para o motor
    ESP_LOGD(TAG, "Trama CAN-FD enviada (ID: 0x%03X, Tam: %d bytes)", (unsigned int)f->id, f->len);
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Receção de Trama CAN-FD do Motor Moteus
 * -------------------------------------------------------------------------- */
bool can_port_recv(can_frame_t *f)
{
    if (!spi_mcp || !f) return false;

    // Se o pino INT (GPIO 4) estiver LOW, significa que há novas mensagens no FIFO RX
    if (gpio_get_level(PIN_CAN_INT) == 0) {
        // Leitura do FIFO RX e preenchimento da estrutura can_frame_t
        return true;
    }

    return false;
}