
/*
 * CAN-FD MCP2518FD -> moteus, ESP-IDF / XIAO ESP32-S3.
 * Preparado para integracao. NAO VALIDADO EM HARDWARE.
 * 1 Mbps arbitragem, 5 Mbps dados (BRS); IDs CAN standard de 11 bits.
 * Verificar cristal/oscilador, transceiver, alimentacao e terminacoes antes de usar.
 */
#include "can_port.h"
#include "board.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "can_mcp2518fd";
static spi_device_handle_t spi_mcp = NULL;
static SemaphoreHandle_t can_lock = NULL;
static bool can_ready = false;

/* Ajustar de acordo com o oscilador real da placa (40 ou 20 MHz).
   Um cristal de 4 MHz com PLL necessita de configuracao adicional. */
#ifndef MCP_SYSCLK_HZ
#define MCP_SYSCLK_HZ 40000000UL
#endif
#if MCP_SYSCLK_HZ != 40000000UL && MCP_SYSCLK_HZ != 20000000UL
#error "MCP_SYSCLK_HZ deve ser 40000000 ou 20000000 (sem PLL)"
#endif

#define MCP_RESET     0x0000u
#define MCP_READ      0x3000u
#define MCP_WRITE     0x2000u
#define REG_CiCON     0x000u
#define REG_CiNBTCFG  0x004u
#define REG_CiDBTCFG  0x008u
#define REG_CiTDC     0x00Cu
#define REG_CiFIFOCON1 0x05Cu
#define REG_CiFIFOSTA1 0x060u
#define REG_CiFIFOUA1  0x064u
#define REG_CiFIFOCON2 0x068u
#define REG_CiFIFOSTA2 0x06Cu
#define REG_CiFIFOUA2  0x070u
#define REG_CiFLTCON0  0x1D0u
#define REG_CiFLTOBJ0  0x1F0u
#define REG_CiMASK0    0x1F4u
#define REG_OSC        0xE00u
#define REG_ECCCON     0xE0Cu
#define RAM_START      0x400u
#define RAM_SIZE       2048u

/* Uma transferencia SPI por comando, sem separar cabecalho e dados. */
static esp_err_t mcp_io(bool is_read, uint16_t addr, uint8_t *data, size_t len)
{
    if (!spi_mcp || !data || len > 76 || (uint32_t)addr + len > 0x1000u) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t tx[78] = {0}, rx[78] = {0};
    uint16_t cmd = (is_read ? MCP_READ : MCP_WRITE) | (addr & 0x0FFFu);
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)cmd;
    if (!is_read) memcpy(&tx[2], data, len);
    spi_transaction_t t = {
        .length = (2 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = is_read ? rx : NULL,
    };
    esp_err_t ret = spi_device_polling_transmit(spi_mcp, &t);
    if (ret == ESP_OK && is_read) memcpy(data, &rx[2], len);
    return ret;
}

static esp_err_t reg_write(uint16_t addr, uint32_t value)
{
    uint8_t d[4] = {
        (uint8_t)value, (uint8_t)(value >> 8),
        (uint8_t)(value >> 16), (uint8_t)(value >> 24)
    };
    return mcp_io(false, addr, d, 4);
}

static esp_err_t reg_read(uint16_t addr, uint32_t *value)
{
    uint8_t d[4];
    esp_err_t e = mcp_io(true, addr, d, 4);
    if (e == ESP_OK) {
        *value = (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                 ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    }
    return e;
}

static esp_err_t write_byte(uint16_t addr, uint8_t value)
{
    return mcp_io(false, addr, &value, 1);
}

static esp_err_t mcp_reset(void)
{
    uint8_t cmd[2] = {0x00, 0x00};
    spi_transaction_t t = { .length = 16, .tx_buffer = cmd };
    return spi_device_polling_transmit(spi_mcp, &t);
}

static uint8_t dlc_to_len(uint8_t dlc)
{
    static const uint8_t lens[16] = {
        0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
    };
    return lens[dlc & 0x0F];
}

static uint8_t len_to_dlc(uint8_t len)
{
    if (len <= 8) return len;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;
}

static esp_err_t config_timing(void)
{
    /* Valores dos campos sao (numero de TQ - 1), fCAN = SYSCLK.
       40 MHz: nominal 1+31+8 = 40 TQ, dados 1+5+2 = 8 TQ.
       20 MHz: nominal 1+15+4 = 20 TQ, dados 1+2+1 = 4 TQ.
       Amostragem aproximada: 80% / 75% (40 MHz); 80% / 75% (20 MHz). */
#if MCP_SYSCLK_HZ == 40000000UL
    const uint32_t nbt = (0u << 24) | (30u << 16) | (7u << 8) | 3u;
    const uint32_t dbt = (0u << 24) | (4u << 16) | (1u << 8) | 1u;
    const uint32_t tdco = 5u;
#else
    const uint32_t nbt = (0u << 24) | (14u << 16) | (3u << 8) | 1u;
    const uint32_t dbt = (0u << 24) | (1u << 16) | (0u << 8) | 0u;
    const uint32_t tdco = 2u;
#endif
    esp_err_t e;
    if ((e = reg_write(REG_CiNBTCFG, nbt)) != ESP_OK) return e;
    if ((e = reg_write(REG_CiDBTCFG, dbt)) != ESP_OK) return e;
    /* TDCMOD=2 (automatico), TDCO calculado para o data bit time. */
    return reg_write(REG_CiTDC, (2u << 16) | (tdco << 8));
}

/* So declarar init concluida quando os registos responderem e o modo for confirmado. */
esp_err_t can_port_init(void)
{
    if (can_ready) return ESP_OK;
    if (spi_mcp) return ESP_ERR_INVALID_STATE;

    spi_bus_config_t bus = {
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };
    esp_err_t e = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_CAN_CS,
        .queue_size = 4,
    };
    e = spi_bus_add_device(SPI2_HOST, &dev, &spi_mcp);
    if (e != ESP_OK) return e;

    can_lock = xSemaphoreCreateMutex();
    if (!can_lock) { e = ESP_ERR_NO_MEM; goto fail; }
    gpio_config_t gpio = {
        .pin_bit_mask = (1ULL << PIN_CAN_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    e = gpio_config(&gpio);
    if (e != ESP_OK) goto fail;

    e = mcp_reset();
    if (e != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t con = 0, osc = 0;
    e = reg_read(REG_CiCON, &con);
    if (e != ESP_OK) goto fail;
    if (((con >> 21) & 7u) != 4u) {
        ESP_LOGE(TAG, "Chip nao entrou em modo Configuration: CiCON=0x%08lX",
                 (unsigned long)con);
        e = ESP_FAIL; goto fail;
    }
    e = reg_read(REG_OSC, &osc);
    if (e != ESP_OK) goto fail;
    ESP_LOGI(TAG, "OSC=0x%08lX; frequencia assumida=%lu Hz",
             (unsigned long)osc, (unsigned long)MCP_SYSCLK_HZ);
    /* PLLEN (bit 0) e SCLKDIV (bit 4): esta implementacao requer ambos desligados. */
    if ((osc & ((1u << 0) | (1u << 4))) != 0) {
        ESP_LOGE(TAG, "PLL/divisor ativos: configuracao de clock nao suportada");
        e = ESP_ERR_NOT_SUPPORTED; goto fail;
    }

    /* Ativar ECC antes de inicializar RAM, para gerar paridade correta. */
    e = reg_write(REG_ECCCON, 1u);
    if (e != ESP_OK) goto fail;
    uint8_t zeros[64] = {0};
    for (uint16_t p = RAM_START; p < RAM_START + RAM_SIZE; p += sizeof(zeros)) {
        e = mcp_io(false, p, zeros, sizeof(zeros));
        if (e != ESP_OK) goto fail;
    }

    e = config_timing();
    if (e != ESP_OK) goto fail;

    /* FIFO1 TX, FIFO2 RX, ambas com 4 mensagens de 64 bytes. */
    e = reg_write(REG_CiFIFOCON1, (7u << 29) | (3u << 24) | 0x80u);
    if (e != ESP_OK) goto fail;
    e = reg_write(REG_CiFIFOCON2, (7u << 29) | (3u << 24) | 0x01u);
    if (e != ESP_OK) goto fail;

    /* Filtro0: aceitar todas as tramas e encaminhar para FIFO2. */
    e = reg_write(REG_CiFLTCON0, 0u);
    if (e != ESP_OK) goto fail;
    e = reg_write(REG_CiFLTOBJ0, 0u);
    if (e != ESP_OK) goto fail;
    e = reg_write(REG_CiMASK0, 0u);
    if (e != ESP_OK) goto fail;
    e = write_byte(REG_CiFLTCON0, 0x82u);
    if (e != ESP_OK) goto fail;

    /* REQOP=0: modo Normal CAN-FD. */
    con &= ~(7u << 24);
    e = reg_write(REG_CiCON, con);
    if (e != ESP_OK) goto fail;
    bool normal = false;
    for (int i = 0; i < 20; ++i) {
        e = reg_read(REG_CiCON, &con);
        if (e != ESP_OK) goto fail;
        if (((con >> 21) & 7u) == 0) { normal = true; break; }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!normal) { e = ESP_ERR_TIMEOUT; goto fail; }
    can_ready = true;
    ESP_LOGI(TAG, "MCP2518FD inicializado (teste real de CAN-FD pendente)");
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "Inicializacao MCP2518FD falhou: %s", esp_err_to_name(e));
    can_ready = false;
    if (can_lock) { vSemaphoreDelete(can_lock); can_lock = NULL; }
    if (spi_mcp) { spi_bus_remove_device(spi_mcp); spi_mcp = NULL; }
    return e;
}

esp_err_t can_port_send(const can_frame_t *f)
{
    if (!f || f->id > 0x7FFu || f->len > 64) return ESP_ERR_INVALID_ARG;
    if (!can_ready) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(can_lock, pdMS_TO_TICKS(20)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t e;
    uint32_t sta = 0, ua = 0;
    e = reg_read(REG_CiFIFOSTA1, &sta);
    if (e != ESP_OK) goto done;
    if (!(sta & 1u)) { e = ESP_ERR_TIMEOUT; goto done; } /* FIFO cheia */
    e = reg_read(REG_CiFIFOUA1, &ua);
    if (e != ESP_OK) goto done;
    if ((ua & 0x0FFFu) > RAM_SIZE - 72) { e = ESP_FAIL; goto done; }

    uint8_t dlc = len_to_dlc(f->len);
    uint8_t padded_len = dlc_to_len(dlc);
    uint8_t msg[72] = {0};
    /* Palavra 0: SID nos bits 10:0; palavra 1: FDF, BRS, DLC. */
    msg[0] = (uint8_t)f->id;
    msg[1] = (uint8_t)(f->id >> 8);
    msg[4] = (uint8_t)(0xC0u | dlc);
    memcpy(&msg[8], f->data, f->len);
    e = mcp_io(false, RAM_START + (ua & 0x0FFFu), msg, 8u + padded_len);
    if (e != ESP_OK) goto done;
    /* Byte 1 de CiFIFOCON1: UINC + TXREQ. */
    e = write_byte(REG_CiFIFOCON1 + 1u, 0x03u);

done:
    xSemaphoreGive(can_lock);
    return e;
}

bool can_port_recv(can_frame_t *f)
{
    if (!f || !can_ready) return false;
    if (xSemaphoreTake(can_lock, 0) != pdTRUE) return false;
    bool valid = false;
    uint32_t sta = 0, ua = 0;
    if (reg_read(REG_CiFIFOSTA2, &sta) != ESP_OK || !(sta & 1u)) goto done;
    if (reg_read(REG_CiFIFOUA2, &ua) != ESP_OK ||
        (ua & 0x0FFFu) > RAM_SIZE - 72) goto done;

    uint8_t header[8] = {0};
    uint16_t addr = RAM_START + (ua & 0x0FFFu);
    if (mcp_io(true, addr, header, 8) != ESP_OK) goto done;
    uint8_t dlc = header[4] & 0x0Fu;
    uint8_t n = dlc_to_len(dlc);
    bool ide = (header[4] & (1u << 4)) != 0;
    bool fd = (header[4] & (1u << 7)) != 0;
    bool rtr = (header[4] & (1u << 5)) != 0;
    uint32_t id = ((uint32_t)header[0] | ((uint32_t)header[1] << 8)) & 0x7FFu;

    /* Esta interface representa apenas IDs standard e mensagens de dados. */
    if (!ide && !rtr && (fd || n <= 8u)) {
        can_frame_t result = { .id = id, .len = n };
        if (n == 0 || mcp_io(true, addr + 8u, result.data, n) == ESP_OK) {
            *f = result;
            valid = true;
        }
    }
    /* Retirar a trama da FIFO (inclusive tramas que nao interessam). */
    if (write_byte(REG_CiFIFOCON2 + 1u, 0x01u) != ESP_OK) valid = false;
done:
    xSemaphoreGive(can_lock);
    return valid;
}
