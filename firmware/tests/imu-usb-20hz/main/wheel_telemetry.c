#include "wheel_telemetry.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "moteus_proto.h"
#include "wire_le.h"

enum {
    PIN_SCK = 7,
    PIN_MISO = 8,
    PIN_MOSI = 9,
    WHEEL_ID = 1,
    AXIS_COUNT = 3,
    POLL_MS = 1000,
    REPLY_MS = 25,
    FIFO_DEPTH = 4,
    SPI_READ = 0x3000,
    SPI_WRITE = 0x2000,
    REG_CON = 0x000,
    REG_NBTCFG = 0x004,
    REG_DBTCFG = 0x008,
    REG_TDC = 0x00c,
    REG_ERROR_COUNTERS = 0x034,
    REG_BUS_DIAGNOSTIC = 0x03c,
    REG_TX_CON = 0x05c,
    REG_TX_STA = 0x060,
    REG_TX_UA = 0x064,
    REG_RX_CON = 0x068,
    REG_RX_STA = 0x06c,
    REG_RX_UA = 0x070,
    REG_FILTER_CON = 0x1d0,
    REG_FILTER_OBJ = 0x1f0,
    REG_FILTER_MASK = 0x1f4,
    REG_ECC_CON = 0xe0c,
    RAM_START = 0x400,
    RAM_SIZE = 2048,
    RAM_HEADER = 8,
    RX_CAPACITY = 32,
    CON_MODE_POS = 21,
    CON_MODE_CONFIG = 4,
    CON_REQOP_MASK = 7u << 24,
    CON_RXTX_FLAGS = (1u << 19) | (1u << 20), // TEF and TXQ, unused.
    CON_LIMIT_RETRIES = 1u << 16,
    FIFO_CAN_ACCESS = 1, // TX not full or RX not empty, depending on the FIFO.
    FIFO_TX_EMPTY = 1u << 2,
    FIFO_UINC = 1,
    FIFO_TXREQ = 2,
    FIFO_RESET = 4,
    FRAME_IDE = 1u << 4,
    FRAME_RTR = 1u << 5,
    FRAME_BRS = 1u << 6,
    FRAME_FD = 1u << 7,
};

typedef struct {
    unsigned count;
    unsigned expected_id_count;
    uint32_t last_id;
    uint8_t last_flags;
    uint8_t last_length;
} reply_observation_t;

static const char *TAG = "wheel_usb";
static const int chip_select[AXIS_COUNT] = {2, 3, 4};
static const char axis_name[AXIS_COUNT] = {'X', 'Y', 'Z'};

static esp_err_t mcp_io(spi_device_handle_t device, uint16_t command,
                        uint8_t *data, size_t bytes)
{
    uint8_t tx[66] = {(uint8_t)(command >> 8), (uint8_t)command};
    uint8_t rx[sizeof tx];
    const bool read = (command & 0xf000u) == SPI_READ;
    if (bytes > sizeof tx - 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (!read && bytes) {
        memcpy(tx + 2, data, bytes);
    }
    spi_transaction_t transfer = {
        .length = (2 + bytes) * 8,
        .tx_buffer = tx,
        .rx_buffer = read ? rx : NULL,
    };
    const esp_err_t error = spi_device_polling_transmit(device, &transfer);
    if (error == ESP_OK && read) {
        memcpy(data, rx + 2, bytes);
    }
    return error;
}

static esp_err_t read_register(spi_device_handle_t device, uint16_t reg, uint32_t *value)
{
    uint8_t bytes[4];
    const esp_err_t error = mcp_io(device, SPI_READ | reg, bytes, sizeof bytes);
    if (error == ESP_OK) {
        *value = cc_read_le32(bytes);
    }
    return error;
}

static esp_err_t write_register(spi_device_handle_t device, uint16_t reg, uint32_t value)
{
    uint8_t bytes[4];
    cc_write_le32(bytes, value);
    return mcp_io(device, SPI_WRITE | reg, bytes, sizeof bytes);
}

static esp_err_t init_controller(spi_device_handle_t device)
{
    ESP_RETURN_ON_ERROR(mcp_io(device, 0, NULL, 0), TAG, "SPI reset");
    vTaskDelay(pdMS_TO_TICKS(5));

    uint32_t control;
    ESP_RETURN_ON_ERROR(read_register(device, REG_CON, &control), TAG, "read CiCON");
    if (((control >> CON_MODE_POS) & 7u) != CON_MODE_CONFIG || control == UINT32_MAX) {
        ESP_LOGW(TAG, "Unexpected CiCON after reset: 0x%08" PRIx32, control);
        return ESP_ERR_INVALID_RESPONSE;
    }
    control &= ~CON_RXTX_FLAGS;
    control |= CON_LIMIT_RETRIES;
    ESP_RETURN_ON_ERROR(write_register(device, REG_CON, control), TAG, "configure CiCON");

    // Use the timing and transceiver-delay configuration found in factory firmware.
    static const struct { uint16_t reg; uint32_t value; } config[] = {
        {REG_NBTCFG, 0x001e0707u},
        {REG_DBTCFG, 0x00050000u},
        {REG_TDC, 0x02020600u},
        {REG_TX_CON, 0x80u}, // One 8-byte query at a time, no retransmissions.
        {REG_RX_CON, 5u << 29 | (FIFO_DEPTH - 1u) << 24}, // 32 bytes, no timestamps.
        {REG_FILTER_OBJ, 0},
        {REG_FILTER_MASK, 0}, // Accept standard and extended replies.
        {REG_FILTER_CON, 0x82u}, // Route filter 0 to RX FIFO 2.
        {REG_ECC_CON, 1u},
    };
    for (size_t i = 0; i < sizeof config / sizeof config[0]; ++i) {
        ESP_RETURN_ON_ERROR(write_register(device, config[i].reg, config[i].value),
                            TAG, "configure CAN reg 0x%03x", (unsigned)config[i].reg);
    }
    uint8_t zeros[64] = {0};
    for (unsigned address = RAM_START; address < RAM_START + RAM_SIZE; address += sizeof zeros) {
        ESP_RETURN_ON_ERROR(mcp_io(device, SPI_WRITE | address, zeros, sizeof zeros),
                            TAG, "initialize CAN RAM");
    }
    ESP_RETURN_ON_ERROR(write_register(device, REG_CON, control & ~CON_REQOP_MASK),
                        TAG, "request classic/FD normal mode");
    for (unsigned tries = 0; tries < 50; ++tries) {
        vTaskDelay(pdMS_TO_TICKS(1));
        ESP_RETURN_ON_ERROR(read_register(device, REG_CON, &control), TAG, "check CAN mode");
        if ((control & CON_REQOP_MASK) == 0 && ((control >> CON_MODE_POS) & 7u) == 0 &&
            (control & CON_LIMIT_RETRIES) != 0) {
            return ESP_OK;
        }
    }
    return ESP_ERR_TIMEOUT;
}

static bool fifo_address(spi_device_handle_t device, uint16_t status_reg, uint16_t addr_reg,
                         size_t capacity, uint16_t *message)
{
    uint32_t status, offset;
    if (read_register(device, status_reg, &status) != ESP_OK ||
        !(status & FIFO_CAN_ACCESS) ||
        read_register(device, addr_reg, &offset) != ESP_OK ||
        offset > RAM_SIZE - RAM_HEADER - capacity || (offset & 3u)) {
        return false;
    }
    *message = RAM_START + offset;
    return true;
}

static bool send_status_query(spi_device_handle_t device)
{
    uint8_t message[RAM_HEADER + 8] = {0};
    const uint32_t id = moteus_can_id(0, WHEEL_ID, 1);
    cc_write_le32(message, (id >> 18) | ((id & 0x3ffffu) << 11));
    const size_t length = moteus_pack_query(message + RAM_HEADER);
    message[4] = FRAME_IDE | FRAME_FD | FRAME_BRS | length; // For 0–8 bytes DLC equals length.
    uint16_t address;
    // The MCP2518FD only commits complete RAM words. DLC still carries the query length.
    if (!fifo_address(device, REG_TX_STA, REG_TX_UA, 8, &address) ||
        mcp_io(device, SPI_WRITE | address, message, sizeof message) != ESP_OK) {
        return false;
    }
    uint8_t request = FIFO_UINC | FIFO_TXREQ;
    return mcp_io(device, SPI_WRITE | (REG_TX_CON + 1), &request, 1) == ESP_OK;
}

static bool wait_for_tx_idle(spi_device_handle_t device)
{
    for (unsigned attempt = 0; attempt < 5; ++attempt) {
        uint32_t control;
        if (read_register(device, REG_TX_CON, &control) != ESP_OK) {
            return false;
        }
        if (((control >> 8) & (FIFO_TXREQ | FIFO_RESET)) == 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

static bool discard_expired_query(spi_device_handle_t device)
{
    uint32_t status;
    if (read_register(device, REG_TX_STA, &status) != ESP_OK) {
        return false;
    }
    if (status & FIFO_TX_EMPTY) {
        return true;
    }

    // A query without an ACK may remain queued. Abort and clear it before the next poll.
    uint8_t command = 0;
    if (mcp_io(device, SPI_WRITE | (REG_TX_CON + 1), &command, 1) != ESP_OK ||
        !wait_for_tx_idle(device)) {
        return false;
    }
    command = FIFO_RESET;
    return mcp_io(device, SPI_WRITE | (REG_TX_CON + 1), &command, 1) == ESP_OK &&
           wait_for_tx_idle(device);
}

static bool read_status_reply(spi_device_handle_t device, moteus_status_t *status,
                              reply_observation_t *observation)
{
    static const uint8_t dlc_bytes[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    bool received = false;
    for (unsigned frame = 0; frame < FIFO_DEPTH; ++frame) {
        uint16_t address;
        uint8_t header[RAM_HEADER];
        if (!fifo_address(device, REG_RX_STA, REG_RX_UA, RX_CAPACITY, &address) ||
            mcp_io(device, SPI_READ | address, header, sizeof header) != ESP_OK) {
            break;
        }

        const uint32_t header_id = cc_read_le32(header);
        const uint32_t sid = header_id & 0x7ffu;
        const uint32_t id = (header[4] & FRAME_IDE) ?
                            (sid << 18) | ((header_id >> 11) & 0x3ffffu) : sid;
        const size_t length = dlc_bytes[header[4] & 0x0fu];
        const size_t ram_length = (length + 3u) & ~(size_t)3u;
        uint8_t data[RX_CAPACITY];
        moteus_status_t candidate;
        const bool expected_id = id == moteus_can_id(WHEEL_ID, 0, 0);
        if (observation) {
            ++observation->count;
            observation->expected_id_count += expected_id;
            observation->last_id = id;
            observation->last_flags = header[4];
            observation->last_length = length;
        }
        const bool valid = expected_id &&
            (header[4] & (FRAME_FD | FRAME_RTR)) == FRAME_FD &&
            length > 0 && ram_length <= sizeof data &&
            mcp_io(device, SPI_READ | (address + RAM_HEADER), data, ram_length) == ESP_OK &&
            moteus_parse_reply(data, length, &candidate) == 0 &&
            candidate.have_mode && candidate.have_motion && candidate.have_aux;
        uint8_t consume = FIFO_UINC;
        if (mcp_io(device, SPI_WRITE | (REG_RX_CON + 1), &consume, 1) != ESP_OK) {
            break;
        }
        if (valid) {
            *status = candidate;
            received = true;
        }
    }
    return received;
}

static void publish_status(unsigned axis, const moteus_status_t *status, const char *state)
{
    const int64_t now = esp_timer_get_time();
    if (!status) {
        printf("BENCH_WHEEL {\"schema\":1,\"axis\":\"%c\",\"sample_us\":%" PRIi64
               ",\"status\":\"%s\"}\n", axis_name[axis], now, state);
        return;
    }
    printf("BENCH_WHEEL {\"schema\":1,\"axis\":\"%c\",\"sample_us\":%" PRIi64
           ",\"status\":\"ok\",\"rpm\":%.3f,\"torque_nm\":%.6f,\"voltage_v\":%.2f,"
           "\"temperature_c\":%.2f,\"position_rev\":%.6f,\"mode\":%u,\"fault\":%u}\n",
           axis_name[axis], now, (double)status->velocity_rev_s * 60.0, status->torque_nm,
           status->voltage_v, status->temp_c, status->position_rev, (unsigned)status->mode,
           (unsigned)status->fault);
}

#ifdef BENCH_CAN_DIAGNOSTICS
static void publish_diagnostics(unsigned axis, spi_device_handle_t device, bool queried, bool valid,
                                const reply_observation_t *early, const reply_observation_t *reply)
{
    uint32_t trec, bdiag1, txsta;
    if (read_register(device, REG_ERROR_COUNTERS, &trec) != ESP_OK ||
        read_register(device, REG_BUS_DIAGNOSTIC, &bdiag1) != ESP_OK ||
        read_register(device, REG_TX_STA, &txsta) != ESP_OK) {
        ESP_LOGW(TAG, "CAN %c diagnostics unavailable", axis_name[axis]);
        return;
    }
    printf("BENCH_CAN {\"axis\":\"%c\",\"queued\":%u,\"valid_reply\":%u,"
           "\"tec\":%u,\"rec\":%u,\"trec\":%" PRIu32 ",\"bdiag1\":%" PRIu32 ",\"txsta\":%" PRIu32 ","
           "\"rx_before\":%u,\"rx_after\":%u,\"expected_id\":%u,"
           "\"last_id\":%" PRIu32 ",\"last_flags\":%u,\"last_len\":%u}\n",
           axis_name[axis], (unsigned)queried, (unsigned)valid, (unsigned)((trec >> 8) & 0xff),
           (unsigned)(trec & 0xff), trec, bdiag1, txsta, early->count, reply->count,
           early->expected_id_count + reply->expected_id_count,
           reply->count ? reply->last_id : early->last_id,
           (unsigned)(reply->count ? reply->last_flags : early->last_flags),
           (unsigned)(reply->count ? reply->last_length : early->last_length));
}
#endif

static void wheel_task(void *unused)
{
    (void)unused;
    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 128,
    };
    spi_device_handle_t devices[AXIS_COUNT] = {0};
    bool ready[AXIS_COUNT] = {0};
    esp_err_t error = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (error == ESP_OK) {
        // Configure all chip-select outputs before starting SPI communication.
        for (unsigned axis = 0; axis < AXIS_COUNT; ++axis) {
            const spi_device_interface_config_t config = {
                .clock_speed_hz = 10000000,
                .spics_io_num = chip_select[axis],
                .mode = 0,
                .queue_size = 1,
            };
            error = spi_bus_add_device(SPI2_HOST, &config, &devices[axis]);
            if (error != ESP_OK) {
                ESP_LOGE(TAG, "Could not configure chip select for %c: %s",
                         axis_name[axis], esp_err_to_name(error));
                break;
            }
        }
    }
    if (error == ESP_OK) {
        for (unsigned axis = 0; axis < AXIS_COUNT; ++axis) {
            const esp_err_t init = init_controller(devices[axis]);
            ready[axis] = init == ESP_OK;
            ESP_LOGI(TAG, "CAN %c (GPIO %d): %s", axis_name[axis],
                     chip_select[axis], esp_err_to_name(init));
        }
    } else {
        ESP_LOGE(TAG, "SPI setup failed: %s", esp_err_to_name(error));
    }

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        bool sent[AXIS_COUNT] = {0};
#ifdef BENCH_CAN_DIAGNOSTICS
        reply_observation_t early[AXIS_COUNT] = {0};
#endif
        for (unsigned axis = 0; axis < AXIS_COUNT; ++axis) {
            if (ready[axis]) {
                moteus_status_t discard;
                reply_observation_t *before = NULL;
#ifdef BENCH_CAN_DIAGNOSTICS
                before = &early[axis];
#endif
                read_status_reply(devices[axis], &discard, before);
                sent[axis] = send_status_query(devices[axis]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(REPLY_MS));
        for (unsigned axis = 0; axis < AXIS_COUNT; ++axis) {
            moteus_status_t status;
            reply_observation_t *after = NULL;
#ifdef BENCH_CAN_DIAGNOSTICS
            reply_observation_t reply = {0};
            after = &reply;
#endif
            const bool valid = sent[axis] && read_status_reply(devices[axis], &status, after);
#ifdef BENCH_CAN_DIAGNOSTICS
            if (ready[axis]) {
                publish_diagnostics(axis, devices[axis], sent[axis], valid, &early[axis], &reply);
            }
#endif
            if (valid) {
                publish_status(axis, &status, "ok");
            } else if (ready[axis] && !discard_expired_query(devices[axis])) {
                publish_status(axis, NULL, "queue_recovery_failed");
            } else {
                publish_status(axis, NULL, !ready[axis] ? "controller_unavailable" :
                               !sent[axis] ? "query_not_sent" : "no_reply");
            }
        }
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_MS));
    }
}

void wheel_telemetry_start(void)
{
    if (xTaskCreate(wheel_task, TAG, 4096, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Unable to start CAN status polling");
    }
}
