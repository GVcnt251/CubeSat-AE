/*
 * cube_core.h - núcleo de controlo de atitude (1 eixo: guinada) com roda de reacção.
 *
 * REGRA DO PROJECTO: este código é o MESMO no simulador (SIL) e no ESP32.
 * Não inclui nada de hardware: recebe medidas, devolve comandos.
 *
 * Convenções
 *   - SI e radianos em todo o lado (a conversão para graus/rpm é feita no protocolo/GS).
 *   - Guinada positiva = rodar para a direita (sentido horário visto de cima, NED).
 *   - Imagem: x cresce para a direita. Alvo à direita do centro => erro > 0 => o cubo
 *     tem de rodar com guinada positiva.
 *   - Eixos do corpo: x frente, y direita, z baixo. Giroscópio z = velocidade de guinada.
 *   - Roda: velocidade RELATIVA ao corpo, positiva => o corpo reage com guinada negativa.
 *
 * Arquitectura (slide "shopping list"): câmara -> detecção -> erro
 *                      IMU -> estimador -> controlador em cascata
 *                      atitude -> velocidade angular -> velocidade da roda (moteus)
 */
#ifndef CUBE_CORE_H
#define CUBE_CORE_H

#include <stdint.h>
#include "target_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CC_HIST 64

typedef enum {
    CC_MODE_CALIB   = 0,  /* arranque: calibra bias do giroscópio e rumo inicial (cubo parado) */
    CC_MODE_IDLE    = 1,  /* roda em roda livre */
    CC_MODE_WHEEL   = 2,  /* nível 1: velocidade da roda directa */
    CC_MODE_RATE    = 3,  /* malha interior: velocidade angular do corpo */
    CC_MODE_HEADING = 4,  /* níveis 2-3: rumo (IMU + magnetómetro) */
    CC_MODE_TRACK   = 5   /* níveis 5-6: apontar ao ponto com a câmara */
} cc_mode_t;

typedef enum {
    CC_TRK_NONE   = 0,
    CC_TRK_LOCKED = 1,    /* medida de câmara recente */
    CC_TRK_COAST  = 2,    /* alvo perdido há pouco: IMU cobre o intervalo */
    CC_TRK_SEARCH = 3     /* alvo perdido há muito (ou nunca visto): varre */
} cc_trk_t;

enum {
    CC_FLAG_TARGET_VALID = 1 << 0,
    CC_FLAG_WHEEL_SAT    = 1 << 1,
    CC_FLAG_MAG_OK       = 1 << 2,
    CC_FLAG_CALIBRATED   = 1 << 3,
    CC_FLAG_TILT_HIGH    = 1 << 4,
    CC_FLAG_UNLOADING    = 1 << 5
};

typedef enum {
    CC_CMD_NONE = 0,
    CC_CMD_MODE,       /* a = cc_mode_t */
    CC_CMD_WHEEL,      /* a = rad/s */
    CC_CMD_RATE,       /* a = rad/s */
    CC_CMD_HEADING,    /* a = rad (relativo ao rumo no fim da calibração) */
    CC_CMD_AIM,        /* a = rad (desvio pretendido do alvo em relação ao centro) */
    CC_CMD_GAIN,       /* id = cc_gain_t, a = valor (SI) */
    CC_CMD_RECAL
} cc_cmd_type_t;

typedef enum {
    CC_GAIN_K_THETA = 0,   /* 1/s */
    CC_GAIN_RATE_KP,       /* (rad/s da roda)/(rad/s do corpo) */
    CC_GAIN_RATE_KI,       /* (rad/s da roda)/rad */
    CC_GAIN_W_REF_MAX,     /* rad/s */
    CC_GAIN_A_REF_MAX,     /* rad/s^2 */
    CC_GAIN_WHEEL_LIM,     /* rad/s */
    CC_GAIN_TORQUE_MAX,    /* N*m */
    CC_GAIN_R_CAM,         /* rad (desvio padrão) */
    CC_GAIN_R_MAG,         /* rad (desvio padrão) */
    CC_GAIN_USE_MAG,       /* 0/1 */
    CC_GAIN_T_SEARCH,      /* s */
    CC_GAIN_COUNT
} cc_gain_t;

typedef struct {
    uint32_t type;   /* cc_cmd_type_t */
    uint32_t id;
    float    a;
    float    b;
} cc_cmd_t;

typedef struct {
    float gyro[3];       /* rad/s, eixos do corpo */
    float accel[3];      /* m/s^2 */
    float wheel_rps;     /* velocidade da roda (relativa), rad/s */
    float wheel_torque;  /* N*m medido pelo driver (informativo) */
    float vbat_v;
} cc_meas_t;

typedef struct {
    uint32_t enable;       /* 0 => driver em "stopped" (roda livre) */
    float    wheel_cmd_rps;
    float    torque_max_nm;
    float    accel_limit_rps2;
} cc_out_t;

typedef struct {
    uint32_t t_ms;
    uint8_t  mode, trk;
    uint16_t flags;
    float heading;       /* rad, relativo ao rumo inicial */
    float rate;          /* rad/s, corrigida do bias */
    float bias;          /* rad/s */
    float err;           /* rad, erro que o controlador está a usar */
    float rate_ref;      /* rad/s */
    float wheel_rps;
    float wheel_cmd_rps;
    float torque_nm;
    float vbat_v;
    float target_err;    /* rad, estimativa psi_t - psi (sem desvio de pontaria) */
    float cam_age_s;     /* s desde a última medida de câmara aceite */
} cc_tlm_t;

typedef struct {
    uint32_t t_capture_ms;
    uint8_t  found, accepted, n_cand, pad;
    float    x, y;           /* px */
    float    err_x, err_y;   /* rad */
    float    contrast;
    uint32_t area;
} cc_vision_t;

typedef struct {
    float cam_fx, cam_fy, cam_cx, cam_cy;
    cc_det_cfg_t det;
    float calib_s;
    /* estimador */
    float q_psi, q_bias, q_target;     /* ruído de processo (rad^2/s, (rad/s)^2/s, rad^2/s) */
    float r_mag, r_cam;                /* variâncias (rad^2) */
    float cam_gate;                    /* rad */
    float mag_nom_uT, mag_tol;
    int   use_mag;
    /* controlador */
    float k_theta, rate_kp, rate_ki;
    float w_ref_max, a_ref_max;
    float wheel_lim, torque_max, wheel_accel;
    /* máquina de estados de seguimento */
    float t_lock, t_search, search_rate;
    float search_span_known, search_span_unknown;
    int gimbal_unload; /* Só quando existe atrito externo no apoio; desativado por omissão. */
    uint32_t imu_timeout_us; /* Default 50000; internal, not TMTC. */
} cc_config_t;

typedef struct cc_s {
    cc_config_t cfg;
    cc_det_ws_t ws;

    /* estimador: x = [psi (rumo), bias do gyro, psi_alvo]; todos "desenrolados" */
    float x[3], P[3][3];
    float e_hist[CC_HIST], psi_hist[CC_HIST];
    uint64_t t_hist[CC_HIST];
    int h_head, h_count;
    uint64_t last_us;
    int started;

    float roll, pitch; int tilt_valid;
    float gyro_z;

    /* calibração */
    uint64_t calib_start_us;
    double gsum, gsq; int gcount;
    double msin, mcos; int mcount;
    float psi_zero; int calibrated;
    int mag_ok;

    /* visão */
    int target_valid, rejects;
    uint64_t last_cam_us, track_entry_us;

    /* comandos / estado */
    cc_mode_t mode; cc_trk_t trk;
    float heading_sp, aim, wheel_sp, rate_sp;
    float rate_I, w_ref_prev, w_ref_last, search_dir;
    int entering_track;
    float wheel_meas;

    int cam_covered;
    float search_rot;
    float search_center, search_span;
    float unload_offset, unload_dir;
    int unloading, unload_returning;

    cc_tlm_t tlm;
    uint64_t imu_last_valid_us, imu_last_call_us;
    int imu_seen, imu_call_seen, imu_missing, imu_recal_required, imu_recal_requested;
    int imu_calibrated_once;
    int search_target_known; /* Accepted target since the last calibration. */
    int cam_seen;
    uint64_t last_cam_capture_us; /* Last accepted capture: reject duplicates. */

} cc_t;

void cc_config_default(cc_config_t *cfg);
void cc_init(cc_t *c, const cc_config_t *cfg);   /* cfg == NULL => valores por omissão */
/* Invalid commands are ignored. Gains use SI units; zero Ki disables integration. */
int cc_gain_valid(uint32_t id, float value);
void cc_command(cc_t *c, const cc_cmd_t *cmd);

/* Chamar a frequência fixa (ex.: 200 Hz). now_us = instante da amostra do IMU. */
typedef enum {
    CC_IMU_OK = 0,
    CC_IMU_UNAVAILABLE,
    CC_IMU_RECAL_REQUIRED
} cc_imu_status_t;

/* Valid means a fresh IMU sample for this cycle. Long gaps latch RECAL_REQUIRED.
 * The latch clears only after a requested, continuous calibration completes.
 * First recovered sample rebases time without prediction or PI integration.
 * WHEEL may run with unavailable IMU after initial calibration; motor health
 * must be enforced by the caller. All structures must be recompiled together. */
cc_imu_status_t cc_step_with_imu(cc_t *c, uint64_t now_us,
                               const cc_meas_t *m, int imu_valid, cc_out_t *out);
void cc_step(cc_t *c, uint64_t now_us, const cc_meas_t *m, cc_out_t *out);

/* Assíncronas */
void cc_on_mag(cc_t *c, uint64_t t_us, const float mag_uT[3]);
int  cc_on_image(cc_t *c, uint64_t t_capture_us, uint64_t now_us, const uint8_t *gray, cc_vision_t *out);
/* No firmware: a tarefa da câmara faz cc_detect() (pesado) com o seu próprio workspace, FORA do mutex,
 * e só depois chama cc_on_detection() (leve) dentro do mutex. cc_on_image() = os dois, para o SIL. */
int  cc_on_detection(cc_t *c, uint64_t t_capture_us, uint64_t now_us, const cc_det_t *det, cc_vision_t *out);

void cc_get_tlm(const cc_t *c, cc_tlm_t *out);
/* Adaptadores ópticos de SIL: não alteram estado/calibração dos sensores. */
int cc_set_camera_model(cc_t *c, float fx, float fy);
void cc_clear_target(cc_t *c);
int cc_point_absolute(cc_t *c, float heading);
int cc_set_search_limits(cc_t *c, float known_span, float unknown_span);
void cc_set_gimbal_unload(cc_t *c, int enabled);

/* Auxiliares para ligações (ctypes) */
cc_t *cc_create(void);
void  cc_destroy(cc_t *c);
uint32_t cc_sizeof_cmd(void);
uint32_t cc_sizeof_meas(void);
uint32_t cc_sizeof_out(void);
uint32_t cc_sizeof_tlm(void);
uint32_t cc_sizeof_vision(void);

#ifdef __cplusplus
}
#endif
#endif
