/*
 * target_detect.h - detecção do ponto escuro no poster (nível 4).
 *
 * Imagem de entrada: 8 bits, escala de cinzentos, 320x240 (QVGA), linha a linha.
 * Sem alocação dinâmica: o chamador fornece o workspace (cc_det_ws_t, ~33 kB).
 * Sem dependências de hardware -> corre igual no PC (SIL) e no ESP32-S3.
 */
#ifndef TARGET_DETECT_H
#define TARGET_DETECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_IMG_W 320
#define CC_IMG_H 240
#define CC_IMG_N (CC_IMG_W * CC_IMG_H)
#define CC_DET_STACK 6144

typedef struct {
    float dark_ratio;   /* limiar = fundo * dark_ratio (fundo = percentil 90 do histograma) */
    int   min_area;     /* px */
    int   max_area;     /* px */
    float min_fill;     /* area / area(bbox): um círculo dá ~0.785 */
    float max_fill;
    float max_aspect;   /* lado maior / lado menor da bbox */
    float min_contrast; /* níveis de cinzento: anel - interior (mínimo absoluto) */
    float min_rel_contrast; /* (anel - interior)/anel */
    int   min_bg;       /* fundo mais escuro que isto => cena inválida (câmara tapada) */
} cc_det_cfg_t;

typedef struct {
    uint8_t found;
    uint8_t n_cand;     /* nº de blobs avaliados */
    float   x, y;       /* centroide sub-pixel; (0,0) = centro do 1.º pixel */
    int     area;
    float   fill;
    float   contrast;
    float   bg;
    float   threshold;
} cc_det_t;

typedef struct {
    uint32_t mask[(CC_IMG_N + 31) / 32];
    uint32_t stack[CC_DET_STACK];
} cc_det_ws_t;

void cc_det_cfg_default(cc_det_cfg_t *cfg);

/* Devolve out->found = 1 se houver um blob com a forma e contraste esperados. */
void cc_detect(const cc_det_cfg_t *cfg, cc_det_ws_t *ws, const uint8_t *img, cc_det_t *out);

#ifdef __cplusplus
}
#endif
#endif
