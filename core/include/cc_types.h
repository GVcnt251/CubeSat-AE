/* Common API types for the temporary scalar and standalone 3D builds.
 * Field order, enum values and wire meanings are unchanged.
 * Neither controller implementation is required by this header.
 */
#ifndef CC_TYPES_H
#define CC_TYPES_H
#include <stdint.h>
#include "target_detect.h"

#define CC_HIST 64

typedef enum {
    CC_MODE_CALIB   = 0,  /* arranque: calibra bias do giroscópio e rumo inicial (cubo parado) */
    CC_MODE_IDLE    = 1,  /* roda em roda livre */
    CC_MODE_WHEEL   = 2,  /* velocidade direta das rodas XYZ */
    CC_MODE_RATE    = 3,  /* malha interior: velocidade angular do corpo */
    CC_MODE_HEADING = 4,  /* orientacao relativa 3D */
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
    /* Process noise in the scalar estimator; reserved/unused by core3. */
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
    int gimbal_unload; /* Scalar support-friction unloading only; core3 requires zero. */
    uint32_t imu_timeout_us; /* Default 50000; internal, not TMTC. */
} cc_config_t;


typedef enum {
    CC_IMU_OK = 0,
    CC_IMU_UNAVAILABLE,
    CC_IMU_RECAL_REQUIRED
} cc_imu_status_t;
#endif
