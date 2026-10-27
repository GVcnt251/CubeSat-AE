/*
 * can_mcp2518fd.c - ESQUELETO. O MCP2518FD (SPI -> CAN-FD) NÃO está implementado.
 *
 * O que falta (ver DS20006027 da Microchip):
 *   1. spi_bus_initialize + spi_bus_add_device (SCK=GPIO7, MISO=GPIO8, MOSI=GPIO9, CS=GPIO3, até 10 MHz)
 *   2. reset (instrução 0x0000) e verificação de OSC/CiCON
 *   3. bit timing nominal 1 Mbit/s e de dados 5 Mbit/s (igual ao do moteus; confirmar com `moteus_tool`)
 *   4. FIFO de TX (1 x 64 B) e FIFO de RX (filtro: id de origem = CONFIG_CUBE_MOTEUS_ID), modo CAN-FD normal
 *   5. can_port_send: escrever no FIFO TX + pedir envio (TXREQ); can_port_recv: ler FIFO RX (via INT GPIO4 ou polling)
 * Alternativa: portar uma biblioteca existente (p.ex. a do próprio fabricante da placa de CAN-FD).
 *
 * Enquanto isto não existir, usar CONFIG_CUBE_WHEEL_FAKE=y (roda simulada em software).
 * As tramas moteus em si (core/src/moteus_proto.c) estão validadas em PC contra a documentação oficial.
 */
#include "can_port.h"
#include "esp_log.h"

static const char *TAG = "can";

esp_err_t can_port_init(void)
{
    ESP_LOGE(TAG, "driver MCP2518FD não implementado (ver can_mcp2518fd.c)");
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t can_port_send(const can_frame_t *f) { return ESP_ERR_NOT_SUPPORTED; }
bool can_port_recv(can_frame_t *f) { return false; }
