#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct { uint32_t id; uint8_t len; uint8_t data[64]; } can_frame_t;

esp_err_t can_port_init(void);
esp_err_t can_port_send(const can_frame_t *f);   /* CAN-FD, com BRS */
bool      can_port_recv(can_frame_t *f);         /* não bloqueante: false se não houver trama */
