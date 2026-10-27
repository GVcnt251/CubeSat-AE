#include "target_detect.h"
#include <string.h>
#include <math.h>

#define W CC_IMG_W
#define H CC_IMG_H

typedef struct {
    int x0, y0, x1, y1;
    int area;
    int overflow;
    uint32_t sum_i;     /* soma das intensidades do blob */
} blob_t;

static inline int  mget(const cc_det_ws_t *ws, int i) { return (ws->mask[i >> 5] >> (i & 31)) & 1u; }
static inline void mclr(cc_det_ws_t *ws, int i)       { ws->mask[i >> 5] &= ~(1u << (i & 31)); }

void cc_det_cfg_default(cc_det_cfg_t *c)
{
    c->dark_ratio   = 0.30f;
    c->min_area     = 40;
    c->max_area     = 6000;
    c->min_fill     = 0.60f;
    c->max_fill     = 0.95f;
    c->max_aspect   = 1.6f;
    c->min_contrast = 20.0f;
    c->min_rel_contrast = 0.50f;
    c->min_bg       = 60;
}

/* Preenche o blob a partir de uma semente (4-conectividade, pilha explícita). */
static void flood(cc_det_ws_t *ws, const uint8_t *img, int seed, int max_area, blob_t *b)
{
    int sp = 0;
    b->x0 = W; b->y0 = H; b->x1 = -1; b->y1 = -1;
    b->area = 0; b->overflow = 0; b->sum_i = 0;

    mclr(ws, seed);
    ws->stack[sp++] = (uint32_t)seed;

    while (sp > 0) {
        int idx = (int)ws->stack[--sp];
        int y = idx / W, x = idx - y * W;

        b->area++;
        b->sum_i += img[idx];
        if (x < b->x0) b->x0 = x;
        if (x > b->x1) b->x1 = x;
        if (y < b->y0) b->y0 = y;
        if (y > b->y1) b->y1 = y;
        if (b->area > max_area) b->overflow = 1;   /* continua a limpar, mas já é lixo */

        int nb[4] = { x > 0 ? idx - 1 : -1, x < W - 1 ? idx + 1 : -1,
                      y > 0 ? idx - W : -1, y < H - 1 ? idx + W : -1 };
        for (int k = 0; k < 4; k++) {
            int n = nb[k];
            if (n < 0 || !mget(ws, n)) continue;
            if (sp >= CC_DET_STACK) { b->overflow = 1; continue; }  /* fica para outra semente */
            mclr(ws, n);
            ws->stack[sp++] = (uint32_t)n;
        }
    }
}

/* Média de um anel a 'off' px da bbox. Devolve nº de amostras; n_dark = amostras < thr. */
static int ring_stats(const uint8_t *img, const blob_t *b, int off, float thr, float *mean, int *n_dark)
{
    int x0 = b->x0 - off, x1 = b->x1 + off, y0 = b->y0 - off, y1 = b->y1 + off;
    long sum = 0; int n = 0, nd = 0;
#define SAMPLE(xx, yy) do { if ((xx) >= 0 && (xx) < W && (yy) >= 0 && (yy) < H) { \
        int v = img[(yy) * W + (xx)]; sum += v; n++; if (v < thr) nd++; } } while (0)
    for (int x = x0; x <= x1; x++) { SAMPLE(x, y0); SAMPLE(x, y1); }
    for (int y = y0 + 1; y < y1; y++) { SAMPLE(x0, y); SAMPLE(x1, y); }
#undef SAMPLE
    *mean = n ? (float)sum / (float)n : 0.0f;
    *n_dark = nd;
    return n;
}

void cc_detect(const cc_det_cfg_t *cfg, cc_det_ws_t *ws, const uint8_t *img, cc_det_t *out)
{
    memset(out, 0, sizeof(*out));

    /* 1) histograma (subamostrado 2x2); fundo = percentil 90 (robusto a sombras e ao ponto) */
    uint32_t hist[256];
    memset(hist, 0, sizeof(hist));
    uint32_t total = 0;
    for (int y = 0; y < H; y += 2) {
        const uint8_t *row = img + y * W;
        for (int x = 0; x < W; x += 2) { hist[row[x]]++; total++; }
    }
    uint32_t acc = 0, target = total - total / 10;
    int bg = 255;
    for (int v = 0; v < 256; v++) { acc += hist[v]; if (acc >= target) { bg = v; break; } }
    if (bg < cfg->min_bg) return;             /* cena demasiado escura / câmara tapada */
    out->bg = (float)bg;
    float thr = (float)bg * cfg->dark_ratio;
    out->threshold = thr;

    /* 2) máscara de pixels escuros */
    memset(ws->mask, 0, sizeof(ws->mask));
    int ithr = (int)thr;
    for (int i = 0; i < CC_IMG_N; i++)
        if (img[i] < ithr) ws->mask[i >> 5] |= 1u << (i & 31);

    /* 3) componentes ligados -> melhor candidato */
    blob_t aborted[4]; int n_ab = 0;
    float best_score = -1.0f; blob_t bb = {0}; float bb_ring = 0, bb_in = 0;
    int cand = 0;

    for (int wi = 0; wi < (CC_IMG_N + 31) / 32; wi++) {
        uint32_t m;
        while ((m = ws->mask[wi]) != 0) {
            int seed = wi * 32 + __builtin_ctz(m);
            int sx = seed % W, sy = seed / W, skip = 0;
            for (int k = 0; k < n_ab; k++)
                if (sx >= aborted[k].x0 && sx <= aborted[k].x1 && sy >= aborted[k].y0 && sy <= aborted[k].y1) skip = 1;
            if (skip) { mclr(ws, seed); continue; }

            blob_t b;
            flood(ws, img, seed, cfg->max_area, &b);

            if (b.overflow || b.area > cfg->max_area) {
                if (n_ab < 4) aborted[n_ab++] = b;
                continue;
            }
            if (b.area < cfg->min_area) continue;
            cand++;

            int bw = b.x1 - b.x0 + 1, bh = b.y1 - b.y0 + 1;
            if (b.x0 == 0 || b.y0 == 0 || b.x1 == W - 1 || b.y1 == H - 1) continue;  /* cortado pela borda */
            float aspect = bw > bh ? (float)bw / bh : (float)bh / bw;
            float fill = (float)b.area / (float)(bw * bh);
            if (aspect > cfg->max_aspect || fill < cfg->min_fill || fill > cfg->max_fill) continue;

            float ring; int nd;
            int ns = ring_stats(img, &b, 4, thr, &ring, &nd);
            if (ns < 8 || nd * 10 > ns) continue;           /* vizinhança tem de ser clara e limpa */
            float inner = (float)b.sum_i / (float)b.area;
            float contrast = ring - inner;
            if (contrast < cfg->min_contrast || contrast < cfg->min_rel_contrast * ring) continue;

            float circ = 1.0f - 2.0f * fabsf(fill - 0.785f);
            float score = contrast * (circ > 0.1f ? circ : 0.1f);
            if (score > best_score) { best_score = score; bb = b; bb_ring = ring; bb_in = inner; }
        }
    }
    out->n_cand = (uint8_t)cand;
    if (best_score < 0.0f) return;

    /* 4) centroide sub-pixel: peso = quanto abaixo do meio-termo entre fundo e interior */
    float mid = 0.5f * (bb_ring + bb_in);
    float sw = 0, sx = 0, sy = 0;                  /* float: o ESP32-S3 só tem FPU de precisão simples */
    int ex = 2;
    for (int y = bb.y0 - ex; y <= bb.y1 + ex; y++) {
        if (y < 0 || y >= H) continue;
        for (int x = bb.x0 - ex; x <= bb.x1 + ex; x++) {
            if (x < 0 || x >= W) continue;
            float w = mid - (float)img[y * W + x];
            if (w > 0) { sw += w; sx += w * (float)(x - bb.x0); sy += w * (float)(y - bb.y0); }
        }
    }
    if (sw <= 0) return;
    out->found    = 1;
    out->x        = (float)bb.x0 + sx / sw;
    out->y        = (float)bb.y0 + sy / sw;
    out->area     = bb.area;
    out->fill     = (float)bb.area / (float)((bb.x1 - bb.x0 + 1) * (bb.y1 - bb.y0 + 1));
    out->contrast = bb_ring - bb_in;
}
