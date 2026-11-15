/* Temporary firmware compatibility: remove only after all scalar callers migrate.
 *
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

#include "cc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gyro[3];       /* rad/s, eixos do corpo */
    float accel[3];      /* m/s^2 */
    float wheel_rps;     /* velocidade da roda (relativa), rad/s */
    float wheel_torque;  /* N*m medido pelo driver (informativo) */
    float vbat_v;
} cc_meas_t;

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
