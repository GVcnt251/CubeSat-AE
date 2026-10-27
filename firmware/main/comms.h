#pragma once
#include <stdint.h>
#include "cube_core.h"

void comms_init(void);                 /* Wi-Fi (STA) */
void comms_task(void *arg);            /* UDP: recebe TC, envia TM_STATE; aprende o endereço da GS no 1.º TC */
void comms_send_vision(const cc_vision_t *v, uint32_t proc_us);
void comms_send_image(const uint8_t *gray320x240);   /* 24 pacotes de 80x60 sub-amostrado */
