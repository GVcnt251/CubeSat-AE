#pragma once
#include <stdbool.h>
#include "cube_core.h"

typedef struct {
    float vel_rps;       /* velocidade da roda no sentido do núcleo (rad/s) */
    float torque_nm;
    float vbat_v;        /* tensão do bus lida pelo moteus (0 se desconhecida) */
    bool  valid;         /* resposta recente do driver */
} wheel_state_t;

int  wheel_init(void);                                   /* 0 = ok */
wheel_state_t wheel_update(const cc_out_t *cmd);         /* envia o comando, devolve o último estado */
