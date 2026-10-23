/*
 * r3d.c - band based z-buffer matcap rasterizer (see r3d.h)
 *
 * Hot loops are written for the ESP32-S3 (Xtensa LX7, single precision FPU, 240 MHz):
 *   - float only in per-vertex / per-triangle / per-span setup
 *   - 32-bit fixed point in the per-pixel loop (z 24.8, u/v 16.16, light 8.16)
 *   - everything touched per pixel (band colour, band depth, matcaps) lives in SRAM
 */
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "r3d.h"

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define R3D_HOT IRAM_ATTR
#else
#define R3D_HOT
#endif

#define FIX_CLAMP 15000          /* |panel coord| limit in 1/16 px -> no int32 overflow */

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
/* float <-> bits without a trip through memory (rfr / wfr on Xtensa) */
#if defined(__XTENSA__)
static inline uint32_t f2u(float f) { uint32_t r; __asm__("rfr %0, %1" : "=r"(r) : "f"(f)); return r; }
static inline float u2f(uint32_t u) { float r; __asm__("wfr %0, %1" : "=f"(r) : "r"(u)); return r; }
#else
static inline uint32_t f2u(float f) { union { float f; uint32_t u; } x = {f}; return x.u; }
static inline float u2f(uint32_t u) { union { uint32_t u; float f; } x = {u}; return x.f; }
#endif

/* int <-> float with a free power-of-two scale (Xtensa FLOAT.S / TRUNC.S immediates) */
#if defined(__XTENSA__)
#define I2F_Q4(v)  ({ float r_; __asm__("float.s %0, %1, 4" : "=f"(r_) : "r"((int)(v))); r_; })
#define F2I_Q8(f)  ({ int r_; __asm__("trunc.s %0, %1, 8" : "=r"(r_) : "f"((float)(f))); r_; })
#define F2I_Q15(f) ({ int r_; __asm__("trunc.s %0, %1, 15" : "=r"(r_) : "f"((float)(f))); r_; })
#else
#define I2F_Q4(v)  ((float)(v) * 0.0625f)
#define F2I_Q8(f)  ((int)((f) * 256.0f))
#define F2I_Q15(f) ((int)((f) * 32768.0f))
#endif

/* reciprocal without a divide instruction (the ESP32-S3 FPU has none -> __divsf3 call):
 * bit-trick seed + Newton steps. frcp2: 2 steps (rel. err ~2e-4), frcp: 3 steps (< 1e-7) */
static inline float frcp2(float x)
{
    float r = u2f(0x7EF311C7u - f2u(x));
    r = r * (2.0f - x * r);
    r = r * (2.0f - x * r);
    return r;
}
static inline float frcp(float x)
{
    float r = frcp2(x);
    return r * (2.0f - x * r);
}

/* ceil / floor to int: one FPU instruction on Xtensa (GCC would call ceilf()) */
#if defined(__XTENSA__)
static inline int iceil(float x) { int r; __asm__("ceil.s %0, %1, 0" : "=r"(r) : "f"(x)); return r; }
static inline int ifloor(float x) { int r; __asm__("floor.s %0, %1, 0" : "=r"(r) : "f"(x)); return r; }
#else
static inline int iceil(float x) { int i = (int)(x + 16384.0f); return i - 16384 + ((float)(i - 16384) < x); }
static inline int ifloor(float x) { return (int)(x + 16384.0f) - 16384; }
#endif

static void *xalloc(const r3d_config_t *c, size_t n, r3d_mem_t k)
{
    void *p = c->alloc ? c->alloc(n, k) : malloc(n);
    if (!p && k == R3D_MEM_FAST && c->alloc) p = c->alloc(n, R3D_MEM_BIG);
    return p;
}

static void setup_rotation(r3d_t *r)
{
    int rot = r->cfg.rotation & 3;
    r->logical_w = (rot & 1) ? r->cfg.height : r->cfg.width;
    r->logical_h = (rot & 1) ? r->cfg.width : r->cfg.height;
}

bool r3d_init(r3d_t *r, const r3d_config_t *cfg, int max_vertices, int max_triangles)
{
    memset(r, 0, sizeof(*r));
    r->cfg = *cfg;
    if (r->cfg.band_h & 1) r->cfg.band_h++;
    if (r->cfg.workers < 1) r->cfg.workers = 1;
    if (r->cfg.workers > R3D_MAX_WORKERS) r->cfg.workers = R3D_MAX_WORKERS;
    r->nbands = (cfg->height + r->cfg.band_h - 1) / r->cfg.band_h;
    if (r->nbands > 255) return false;
    r->vtx_cap = max_vertices;
    r->tri_cap = max_triangles;
    setup_rotation(r);

    const int np = cfg->reflection ? R3D_PASSES : 1;
    for (int p = 0; p < np; p++) {
        r->tv[p] = xalloc(cfg, sizeof(r3d_tv_t) * (size_t)max_vertices, R3D_MEM_BIG);
        if (!r->tv[p]) return false;
    }
    r->tri_band = xalloc(cfg, sizeof(uint16_t) * (size_t)max_triangles * np, R3D_MEM_BIG);
    if (!r->tri_band) return false;
    for (int w = 0; w < r->cfg.workers; w++) {
        r3d_worker_bins_t *b = &r->bins[w];
        b->count = xalloc(cfg, sizeof(uint16_t) * r->nbands, R3D_MEM_FAST);
        b->start = xalloc(cfg, sizeof(uint32_t) * (r->nbands + 1), R3D_MEM_FAST);
        b->xmin = xalloc(cfg, sizeof(int16_t) * r->nbands, R3D_MEM_FAST);
        b->xmax = xalloc(cfg, sizeof(int16_t) * r->nbands, R3D_MEM_FAST);
        b->list_cap = (uint32_t)max_triangles * np * 2 + 1024;
        b->list = xalloc(cfg, sizeof(uint32_t) * b->list_cap, R3D_MEM_BIG);
        if (!b->count || !b->start || !b->xmin || !b->xmax || !b->list) return false;
        memset(b->count, 0, sizeof(uint16_t) * r->nbands);
        memset(b->start, 0, sizeof(uint32_t) * (r->nbands + 1));
    }
    r->prev_x0 = xalloc(cfg, sizeof(int16_t) * r->nbands, R3D_MEM_FAST);
    r->prev_x1 = xalloc(cfg, sizeof(int16_t) * r->nbands, R3D_MEM_FAST);
    if (!r->prev_x0 || !r->prev_x1) return false;
    for (int i = 0; i < r->nbands; i++) { r->prev_x0[i] = 1; r->prev_x1[i] = 0; }
    r->force_full = true;
    return true;
}

bool r3d_set_model(r3d_t *r, const r3d_model_t *m)
{
    if (m) {
        for (int l = 0; l < 2; l++)
            if (m->lod[l].num_vertices > r->vtx_cap || m->lod[l].num_triangles > r->tri_cap) {
                r->model = NULL;
                return false;
            }
    }
    r->model = m;
    r->force_full = true;
    return true;
}

void r3d_set_rotation(r3d_t *r, int rotation)
{
    r->cfg.rotation = rotation & 3;
    setup_rotation(r);
    r->force_full = true;
}

void r3d_invalidate(r3d_t *r) { r->force_full = true; }

void r3d_set_overlay(r3d_t *r, bool on, int x0, int y0, int x1, int y1)
{
    if (r->overlay_on && !on) r->force_full = true;    /* erase the old HUD */
    r->overlay_on = on;
    r->ov_x0 = x0; r->ov_y0 = y0; r->ov_x1 = x1; r->ov_y1 = y1;
}

void r3d_logical_to_panel(const r3d_t *r, int lx, int ly, int *px, int *py)
{
    int W = r->cfg.width, H = r->cfg.height;
    switch (r->cfg.rotation & 3) {
    default: *px = lx; *py = ly; break;
    case 1: *px = W - 1 - ly; *py = lx; break;
    case 2: *px = W - 1 - lx; *py = H - 1 - ly; break;
    case 3: *px = ly; *py = H - 1 - lx; break;
    }
}

/* -------------------------------------------------------------------------------------- */
void r3d_begin_frame(r3d_t *r, const r3d_view_t *v)
{
    r->edge_aa = v->edge_aa;
    const r3d_model_t *m = r->model;
    memset(&r->stats, 0, sizeof(r->stats));
    r->full_frame = r->force_full;
    r->force_full = false;
    uint16_t bg = v->bg_color;
    r->bg_swapped = (uint16_t)((bg >> 8) | (bg << 8));
    if (!m) { r->passes = 0; return; }

    const float R = m->radius;
    const float fov = (v->fov_deg > 5.0f ? v->fov_deg : 28.0f) * 0.0174533f;
    const float t = tanf(fov * 0.5f);
    const float dist = R / (0.92f * t);
    const float zoom = clampf(v->zoom, 0.3f, 2.5f);
    const float base = 0.46f * (float)r->logical_w * dist / R;    /* bounding sphere fills the width */
    float fit = 1.0f;
    const float pivot_y = m->ymax * 0.45f;
    r3d_mat3_t Rm = r3d_quat_to_mat3(v->orient);
    if (v->autofit) {
        /* project the 8 corners of the bounding box (rotation only) and make the larger of
         * the horizontal / vertical extents fit 92 % of the screen */
        float mx = 1e-3f, my = 1e-3f;
        for (int c = 0; c < 8; c++) {
            const float px = (c & 1) ? m->bmax[0] : m->bmin[0];
            const float py = ((c & 2) ? m->bmax[1] : m->bmin[1]) - pivot_y;
            const float pz = (c & 4) ? m->bmax[2] : m->bmin[2];
            const float X = Rm.m[0][0] * px + Rm.m[0][1] * py + Rm.m[0][2] * pz;
            const float Y = Rm.m[1][0] * px + Rm.m[1][1] * py + Rm.m[1][2] * pz;
            const float Z = Rm.m[2][0] * px + Rm.m[2][1] * py + Rm.m[2][2] * pz;
            const float s = dist / (dist - Z);                     /* perspective of that corner */
            if (fabsf(X) * s > mx) mx = fabsf(X) * s;
            if (fabsf(Y) * s > my) my = fabsf(Y) * s;
        }
        const float fx = 0.46f * (float)r->logical_w * dist / mx;
        const float fy = 0.46f * (float)r->logical_h * dist / my;
        float target = (fx < fy ? fx : fy) / base;
        if (target > 1.25f) target = 1.25f;
        r->fit = (r->fit <= 0.0f) ? target : r->fit + (target - r->fit) * 0.18f;
        fit = r->fit;
    }
    r->focal = base * fit * zoom;
    r->lcx = r->logical_w * 0.5f + v->shift_x;
    r->lcy = r->logical_h * 0.5f + v->shift_y;

    const float rext = R + m->ymax;
    float nearz = dist - rext, farz = dist + rext;
    if (nearz < 0.05f * dist) nearz = 0.05f * dist;
    r->zk1 = 65535.0f / (1.0f / nearz - 1.0f / farz);
    r->zk0 = r->zk1 / nearz;

    const float *sc = m->pos_scale;
    for (int p = 0; p < R3D_PASSES; p++) {
        float ys = p ? -1.0f : 1.0f;
        for (int i = 0; i < 3; i++) {
            r->M[p][i][0] = Rm.m[i][0] * sc[0];
            r->M[p][i][1] = Rm.m[i][1] * sc[1] * ys;
            r->M[p][i][2] = Rm.m[i][2] * sc[2];
            r->M[p][i][3] = -Rm.m[i][1] * pivot_y;
        }
        r->M[p][2][3] -= dist;
    }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r->N[i][j] = Rm.m[i][j] * (1.0f / 127.0f);

    /* reflection only while the camera is above the floor; fade it out near the horizon */
    r->passes = 1;
    r->refl_k = 0;
    if (v->reflection && r->tv[1] && v->reflection_strength > 0.01f) {
        /* camera position in model space: R^T * (0,0,dist) + pivot */
        float cam_y = Rm.m[2][1] * dist + pivot_y;
        float k = clampf(cam_y / (0.35f * R), 0.0f, 1.0f);
        if (k > 0.01f) {
            r->passes = 2;
            r->refl_k = v->reflection_strength * k;
            r->refl_inv_h = 1.0f / (m->ymax * 0.7f);
        }
    }
}

/* -------------------------------------------------------------------------------------- */
void R3D_HOT r3d_job_transform(r3d_t *r, int w, int nw)
{
    const r3d_model_t *m = r->model;
    if (!m) return;
    const int W = r->cfg.width, H = r->cfg.height;
    const int rot = r->cfg.rotation & 3;
    /* logical (lx, ly) = (lcx + X*s, lcy - Y*s), s = focal / w; panel = rotation of logical.
     * Fold rotation, focal length and the 1/16 sub-pixel scale into two matrix rows so the
     * per-vertex work is 3 dot products, one reciprocal and a few conversions. */
    float ax, ay, bx, by, cx, cy;      /* fx = cx + (ax*X + ay*Y)*s ; fy = cy + (bx*X + by*Y)*s */
    switch (rot) {
    default: ax = 1;  ay = 0;  cx = r->lcx;            bx = 0;  by = -1; cy = r->lcy;            break;
    case 1:  ax = 0;  ay = 1;  cx = (float)W - r->lcy; bx = 1;  by = 0;  cy = r->lcx;            break;
    case 2:  ax = -1; ay = 0;  cx = (float)W - r->lcx; bx = 0;  by = 1;  cy = (float)H - r->lcy; break;
    case 3:  ax = 0;  ay = -1; cx = r->lcy;            bx = -1; by = 0;  cy = (float)H - r->lcx; break;
    }
    const float F16 = r->focal * 16.0f;
    const float cx16 = cx * 16.0f + 0.5f, cy16 = cy * 16.0f + 0.5f;   /* +0.5: round on trunc */
    const float zk0 = r->zk0 + 0.5f, zk1 = r->zk1;
    /* matcap uv in 0.16 fixed: u = (0.5 + 0.5 * nx_view) * 65535 */
    const float US = 0.5f * 65535.0f;
    const int UV_LO = 256, UV_HI = 65279;          /* half a texel margin at 128x128 */
    const float ysc = m->pos_scale[1];

    for (int p = 0; p < r->passes; p++) {
        const r3d_lod_t *L = &m->lod[p];
        const int n = L->num_vertices;
        const int v0 = n * w / nw, v1 = n * (w + 1) / nw;
        const float (*M)[4] = r->M[p];
        const float PX0 = F16 * (ax * M[0][0] + ay * M[1][0]), PX1 = F16 * (ax * M[0][1] + ay * M[1][1]);
        const float PX2 = F16 * (ax * M[0][2] + ay * M[1][2]), PX3 = F16 * (ax * M[0][3] + ay * M[1][3]);
        const float PY0 = F16 * (bx * M[0][0] + by * M[1][0]), PY1 = F16 * (bx * M[0][1] + by * M[1][1]);
        const float PY2 = F16 * (bx * M[0][2] + by * M[1][2]), PY3 = F16 * (bx * M[0][3] + by * M[1][3]);
        const float W0 = -M[2][0], W1 = -M[2][1], W2 = -M[2][2], W3 = -M[2][3];
        const float nys = p ? -1.0f : 1.0f;
        const float U0 = US * r->N[0][0], U1 = US * r->N[0][1] * nys, U2 = US * r->N[0][2];
        const float V0 = -US * r->N[1][0], V1 = -US * r->N[1][1] * nys, V2 = -US * r->N[1][2];
        const float UC = US + 0.5f;
        const float hk = ysc * r->refl_inv_h;
        r3d_tv_t *out = r->tv[p] + v0;
        const r3d_vertex_t *in = L->vertices + v0;
        for (int i = v0; i < v1; i++, in++, out++) {
            const float px = in->p[0], py = in->p[1], pz = in->p[2];
            const float iw = frcp2(W0 * px + W1 * py + W2 * pz + W3);
            int xi = (int)(cx16 + (PX0 * px + PX1 * py + PX2 * pz + PX3) * iw);
            int yi = (int)(cy16 + (PY0 * px + PY1 * py + PY2 * pz + PY3) * iw);
            int zi = (int)(zk0 - zk1 * iw);
            out->x = (int16_t)clampi(xi, -FIX_CLAMP, FIX_CLAMP);
            out->y = (int16_t)clampi(yi, -FIX_CLAMP, FIX_CLAMP);
            out->z = (uint16_t)clampi(zi, 0, 65535);
            const float nx = in->n[0], ny = in->n[1], nz = in->n[2];
            out->u = (uint16_t)clampi((int)(UC + U0 * nx + U1 * ny + U2 * nz), UV_LO, UV_HI);
            out->v = (uint16_t)clampi((int)(UC + V0 * nx + V1 * ny + V2 * nz), UV_LO, UV_HI);
            if (p == 0) {
                out->a = in->ao;
            } else {
                float h = 1.0f - py * hk;
                h = h < 0.0f ? 0.0f : h;
                out->a = (uint8_t)clampi((int)(in->ao * r->refl_k * h * h), 0, 255);
            }
            out->flags = 0;
        }
    }
}

/* -------------------------------------------------------------------------------------- */
void R3D_HOT r3d_job_bin(r3d_t *r, int w, int nw)
{
    const r3d_model_t *m = r->model;
    r3d_worker_bins_t *B = &r->bins[w];
    const int nb = r->nbands, bh = r->cfg.band_h;
    const int W = r->cfg.width, H = r->cfg.height;
    for (int b = 0; b < nb; b++) { B->count[b] = 0; B->xmin[b] = 32767; B->xmax[b] = -1; }
    B->start[0] = 0;
    if (!m || r->passes == 0) { for (int b = 0; b < nb; b++) B->start[b + 1] = 0; return; }

    uint8_t dsided[R3D_MAX_MATERIALS];
    for (int i = 0; i < m->num_materials; i++) dsided[i] = (m->mat[i].flags & R3D_MAT_DOUBLE_SIDED) ? 1 : 0;

    uint32_t binned = 0;
    for (int p = 0; p < r->passes; p++) {
        const r3d_lod_t *L = &m->lod[p];
        const int nt = L->num_triangles;
        const int t0 = nt * w / nw, t1 = nt * (w + 1) / nw;
        const r3d_tv_t *tv = r->tv[p];
        uint16_t *tb = r->tri_band + (size_t)p * r->tri_cap;
        const int front = p ? 1 : -1;       /* sign of the screen area of front faces */
        const uint8_t *tmat = L->tri_material;
        const uint16_t *ix = L->indices + 3 * t0;
        for (int t = t0; t < t1; t++, ix += 3) {
            const r3d_tv_t *a = &tv[ix[0]], *b = &tv[ix[1]], *c = &tv[ix[2]];
            int32_t area = (int32_t)(b->x - a->x) * (c->y - a->y) - (int32_t)(c->x - a->x) * (b->y - a->y);
            if (area == 0 || ((area > 0 ? 1 : -1) != front && !dsided[tmat[t]])) { tb[t] = 0xFFFF; continue; }
            int ymin = a->y, ymax = a->y, xmin = a->x, xmax = a->x;
            if (b->y < ymin) ymin = b->y;
            if (b->y > ymax) ymax = b->y;
            if (c->y < ymin) ymin = c->y;
            if (c->y > ymax) ymax = c->y;
            if (b->x < xmin) xmin = b->x;
            if (b->x > xmax) xmax = b->x;
            if (c->x < xmin) xmin = c->x;
            if (c->x > xmax) xmax = c->x;
            int rs = (ymin + 7) >> 4, re = (ymax + 7) >> 4;
            int cs = (xmin + 7) >> 4, ce = (xmax + 7) >> 4;
            if (rs < 0) rs = 0;
            if (re > H) re = H;
            if (cs < 0) cs = 0;
            if (ce > W) ce = W;
            if (rs >= re || cs >= ce) { tb[t] = 0xFFFF; continue; }
            int b0 = rs / bh, b1 = (re - 1) / bh;
            tb[t] = (uint16_t)(b0 | (b1 << 8));
            for (int k = b0; k <= b1; k++) {
                B->count[k]++;
                if (cs < B->xmin[k]) B->xmin[k] = (int16_t)cs;
                if (ce - 1 > B->xmax[k]) B->xmax[k] = (int16_t)(ce - 1);
            }
            binned++;
        }
    }
    /* prefix sums (clamped to capacity) */
    uint32_t acc = 0;
    for (int b = 0; b < nb; b++) {
        B->start[b] = acc;
        acc += B->count[b];
        if (acc > B->list_cap) acc = B->list_cap;
    }
    B->start[nb] = acc;
    for (int b = 0; b < nb; b++) B->count[b] = 0;   /* reuse as fill cursor */
    for (int p = 0; p < r->passes; p++) {
        const int nt = m->lod[p].num_triangles;
        const int t0 = nt * w / nw, t1 = nt * (w + 1) / nw;
        const uint16_t *tb = r->tri_band + (size_t)p * r->tri_cap;
        const uint32_t tag = (uint32_t)p << 31;
        for (int t = t0; t < t1; t++) {
            uint16_t e = tb[t];
            if (e == 0xFFFF) continue;
            int b0 = e & 0xFF, b1 = e >> 8;
            for (int k = b0; k <= b1; k++) {
                uint32_t pos = B->start[k] + B->count[k];
                if (pos < B->start[k + 1]) { B->list[pos] = (uint32_t)t | tag; B->count[k]++; }
            }
        }
    }
    __atomic_fetch_add(&r->stats.tris_binned, binned, __ATOMIC_RELAXED);
}

/* -------------------------------------------------------------------------------------- */
/*
 * One span. SH is a compile-time constant for the common matcap sizes (64/128) so the
 * texel address is two shifts + or, instead of variable shifts through the SAR register.
 * light a is Q15 (0..255 << 15); factor f = a >> 18 is 0..32, applied to R, G, B at once.
 */
#define SPAN_LOOP(SH)                                                                       \
    do {                                                                                    \
        const uint32_t zz = z >> 8;                                                         \
        if (zz < *zp) {                                                                     \
            *zp = (uint16_t)zz;                                                             \
            CNT(r3d_cnt_drawn++);                                                           \
            uint32_t col = mc[(((uv & 0xFFFFu) >> (16 - (SH))) << (SH)) | (uv >> (32 - (SH)))]; \
            uint32_t s_ = (col | (col << 16)) & 0x07E0F81Fu;                                \
            s_ = ((s_ * ((a + (1u << 17)) >> 18 & 63u)) >> 5) & 0x07E0F81Fu;                \
            col = s_ | (s_ >> 16);                                                          \
            *cp = (uint16_t)((col >> 8) | (col << 8));                                      \
        }                                                                                   \
        zp++; cp++;                                                                         \
        z += (uint32_t)dzx_i; uv += duvx; a += (uint32_t)dax_i;                             \
    } while (--n)

#ifdef R3D_COUNT
uint64_t r3d_cnt_setup, r3d_cnt_rows, r3d_cnt_spans, r3d_cnt_px, r3d_cnt_drawn;
#define CNT(x) (x)
#else
#define CNT(x)
#endif
/* rows [R0, R1) with left edge CL + yc*KL and right edge CR + yc*KR (both pre-shifted by
 * -0.5 so ceil() gives the first covered pixel centre) */
#define ROWS(R0, R1, CL, KL, CR, KR)                                                        \
    for (int row = (R0); row < (R1); row++) {                                               \
        const float yc = (float)row + 0.5f;                                                 \
        CNT(r3d_cnt_rows++);                                                                \
        int xs = iceil((CL) + yc * (KL));                                                   \
        int xe = iceil((CR) + yc * (KR));                                                   \
        xs = xs < rx0 ? rx0 : xs;                                                           \
        xe = xe > rx1 + 1 ? rx1 + 1 : xe;                                                   \
        int n = xe - xs;                                                                    \
        if (n > 0) {                                                                        \
            const uint32_t rr = (uint32_t)(row - rs);                                       \
            const uint32_t ox = (uint32_t)(xs - xb);                                        \
            CNT(r3d_cnt_spans++);                                                           \
            CNT(r3d_cnt_px += n);                                                           \
            uint32_t z = zb0 + (uint32_t)dzy_i * rr + (uint32_t)dzx_i * ox;                 \
            uint32_t a = ab0 + (uint32_t)day_i * rr + (uint32_t)dax_i * ox;                 \
            uint32_t uv = uvb0 + duvy * rr + duvx * ox;                                     \
            const size_t off = (size_t)(row - by0) * stride + (size_t)(xs - rx0);           \
            uint16_t *zp = zbuf + off;                                                      \
            uint16_t *cp = cbuf + off;                                                      \
            if (sh == 7) SPAN_LOOP(7);                                                      \
            else if (sh == 6) SPAN_LOOP(6);                                                 \
            else SPAN_LOOP(sh);                                                             \
        }                                                                                   \
    }

static void R3D_HOT raster_tri(const r3d_tv_t *va, const r3d_tv_t *vb, const r3d_tv_t *vc,
                                const r3d_material_t *mat, int by0, int by1, int rx0, int rx1,
                                uint16_t *cbuf, uint16_t *zbuf, int stride)
{
    /* sort by y (integer 12.4 coordinates) */
    const r3d_tv_t *t;
    if (vb->y < va->y) { t = va; va = vb; vb = t; }
    if (vc->y < va->y) { t = va; va = vc; vc = t; }
    if (vc->y < vb->y) { t = vb; vb = vc; vc = t; }

    /* covered rows, exact in integers: row r is inside if y_top <= r + 0.5 < y_bottom */
    int rs = (va->y + 7) >> 4;
    const int rm = (vb->y + 7) >> 4;
    int re = (vc->y + 7) >> 4;
    if (rs < by0) rs = by0;
    if (re > by1 + 1) re = by1 + 1;
    if (rs >= re) return;

    const float x0 = I2F_Q4(va->x), y0 = I2F_Q4(va->y);
    const float x1 = I2F_Q4(vb->x), y1 = I2F_Q4(vb->y);
    const float x2 = I2F_Q4(vc->x), y2 = I2F_Q4(vc->y);
    const float e1x = x1 - x0, e1y = y1 - y0, e2x = x2 - x0, e2y = y2 - y0;
    const float area = e1x * e2y - e2x * e1y;
    if (area > -1e-4f && area < 1e-4f) return;
    CNT(r3d_cnt_setup++);
    const float inv = frcp2(area);
    /* gradient factors shared by all attributes:
     * dA/dx = dA1*g1 - dA2*g2 ; dA/dy = dA2*g3 - dA1*g4 */
    const float g1 = e2y * inv, g2 = e1y * inv, g3 = e1x * inv, g4 = e2x * inv;

    /* base point near the triangle (left of bbox, first row) keeps fixed point in range */
    float xmin = x0 < x1 ? x0 : x1;
    xmin = x2 < xmin ? x2 : xmin;
    int xb = ifloor(xmin);
    if (xb < rx0) xb = rx0;
    const float fx = ((float)xb + 0.5f) - x0, fy = ((float)rs + 0.5f) - y0;

    /* attributes: depth z (x256), packed matcap uv (u high / v low 16 bits), light a (x32768) */
    int32_t dzx_i, dzy_i, dax_i, day_i;
    uint32_t zb0, ab0, uvb0, duvx, duvy;
    {
        const float A0 = va->z, d1 = (float)vb->z - A0, d2 = (float)vc->z - A0;
        const float gx = d1 * g1 - d2 * g2, gy = d2 * g3 - d1 * g4;
        dzx_i = F2I_Q8(gx);
        dzy_i = F2I_Q8(gy);
        zb0 = (uint32_t)F2I_Q8(A0 + gx * fx + gy * fy);
    }
    {
        const float A0 = va->a, d1 = (float)vb->a - A0, d2 = (float)vc->a - A0;
        const float gx = d1 * g1 - d2 * g2, gy = d2 * g3 - d1 * g4;
        dax_i = F2I_Q15(gx);
        day_i = F2I_Q15(gy);
        ab0 = (uint32_t)F2I_Q15(A0 + gx * fx + gy * fy);
    }
    {
        const float U0 = va->u, du1 = (float)vb->u - U0, du2 = (float)vc->u - U0;
        const float ugx = du1 * g1 - du2 * g2, ugy = du2 * g3 - du1 * g4;
        const uint32_t ub = (uint32_t)(int32_t)(U0 + ugx * fx + ugy * fy);
        const uint32_t ux = (uint32_t)(int32_t)ugx, uy = (uint32_t)(int32_t)ugy;
        const float V0 = va->v, dv1 = (float)vb->v - V0, dv2 = (float)vc->v - V0;
        const float vgx = dv1 * g1 - dv2 * g2, vgy = dv2 * g3 - dv1 * g4;
        duvx = (ux << 16) + (uint32_t)(int32_t)vgx;
        duvy = (uy << 16) + (uint32_t)(int32_t)vgy;
        uvb0 = (ub << 16) + (uint32_t)(int32_t)(V0 + vgx * fx + vgy * fy);
    }

    /* edges: x(yc) = c + yc * k, c = xa - ya*k - 0.5 (bit-identical for neighbours) */
    const float k02 = e2x * frcp2(e2y);
    const float c02 = x0 - y0 * k02 - 0.5f;
    const uint16_t *mc = mat->matcap;
    const int sh = mat->log2;
    const bool long_left = area > 0;      /* y-down: area > 0 -> v1 right of long edge */
    const int rmid = rm < rs ? rs : (rm > re ? re : rm);

    if (rs < rmid) {                      /* upper part: edge v0 -> v1 */
        const float k01 = e1x * frcp2(e1y);
        const float c01 = x0 - y0 * k01 - 0.5f;
        if (long_left) { ROWS(rs, rmid, c02, k02, c01, k01) }
        else           { ROWS(rs, rmid, c01, k01, c02, k02) }
    }
    if (rmid < re) {                      /* lower part: edge v1 -> v2 */
        const float k12 = (x2 - x1) * frcp2(y2 - y1);
        const float c12 = x1 - y1 * k12 - 0.5f;
        if (long_left) { ROWS(rmid, re, c02, k02, c12, k12) }
        else           { ROWS(rmid, re, c12, k12, c02, k02) }
    }
}
#undef ROWS

/* -------------------------------------------------------------------------------------- */
/*
 * Edge anti-aliasing (post pass on one band). A pixel is an edge when one of its 4
 * neighbours is background while it is not (or the other way round), when the depth
 * jumps (one part in front of another) or when the green channel (≈ luma) jumps.
 * Edge pixels become 1/2 own colour + 1/8 of each neighbour: the stair-steps of
 * silhouettes and part boundaries turn into soft gradients, flat areas stay sharp.
 * Rows at the band border only use the neighbours inside the band.
 * Colours in cbuf are byte-swapped RGB565.
 */
#define AA_DZ     1200u      /* depth jump (z16 units) */
#define AA_DG     14         /* green jump (0..63) */

static inline uint32_t spread565(uint16_t sw)
{
    const uint32_t c = (uint16_t)((sw >> 8) | (sw << 8));
    return (c | (c << 16)) & 0x07E0F81Fu;
}

static inline int green6(uint16_t sw) { return ((sw << 3) & 0x38) | (sw >> 13); }  /* G of swapped 565 */

static inline bool aa_edge(uint16_t za, uint16_t zb, uint16_t ca, uint16_t cb)
{
    if ((za == 0xFFFF) != (zb == 0xFFFF)) return true;
    if (za == 0xFFFF) return false;
    const uint32_t dz = za > zb ? (uint32_t)(za - zb) : (uint32_t)(zb - za);
    if (dz > AA_DZ) return true;
    const int dg = green6(ca) - green6(cb);
    return dg > AA_DG || dg < -AA_DG;
}

static void R3D_HOT edge_aa(uint16_t *cbuf, const uint16_t *zbuf, int stride, int rows)
{
    if (stride < 3 || rows < 1 || stride > 400) return;
    uint16_t prev[400], cur[400];            /* original colours of row y-1 and y */
    memcpy(cur, cbuf, (size_t)stride * 2);
    for (int y = 0; y < rows; y++) {
        uint16_t *crow = cbuf + (size_t)y * stride;
        const uint16_t *zr = zbuf + (size_t)y * stride;
        const uint16_t *zu = y > 0 ? zr - stride : NULL;
        const uint16_t *zd = y + 1 < rows ? zr + stride : NULL;
        const uint16_t *cd = zd ? crow + stride : NULL;      /* row below: still original */
        if (y > 0) {
            memcpy(prev, cur, (size_t)stride * 2);
            memcpy(cur, crow, (size_t)stride * 2);
        }
        for (int x = 1; x < stride - 1; x++) {
            const uint16_t z = zr[x];
            const uint16_t c = cur[x];
            /* fast reject: all 4 neighbours background like us */
            if (z == 0xFFFF && zr[x - 1] == 0xFFFF && zr[x + 1] == 0xFFFF &&
                (!zu || zu[x] == 0xFFFF) && (!zd || zd[x] == 0xFFFF))
                continue;
            const uint16_t cl = cur[x - 1], cr = cur[x + 1];
            const uint16_t cu = zu ? prev[x] : c, cdn = zd ? cd[x] : c;
            bool e = aa_edge(z, zr[x - 1], c, cl) || aa_edge(z, zr[x + 1], c, cr);
            if (!e && zu) e = aa_edge(z, zu[x], c, cu);
            if (!e && zd) e = aa_edge(z, zd[x], c, cdn);
            if (!e) continue;
            uint32_t s = spread565(c) * 4 + spread565(cl) + spread565(cr) + spread565(cu) + spread565(cdn);
            s = (s >> 3) & 0x07E0F81Fu;
            const uint16_t n = (uint16_t)(s | (s >> 16));
            crow[x] = (uint16_t)((n >> 8) | (n << 8));
        }
    }
}
#undef AA_DZ
#undef AA_DG

bool R3D_HOT r3d_render_band(r3d_t *r, int band, uint16_t *cbuf, uint16_t *zbuf, r3d_rect_t *out)
{
    const int W = r->cfg.width, H = r->cfg.height, bh = r->cfg.band_h;
    const int y0 = band * bh;
    int y1 = y0 + bh - 1;
    if (y1 > H - 1) y1 = H - 1;

    int cx0 = 32767, cx1 = -1;
    for (int w = 0; w < r->cfg.workers; w++) {
        if (r->bins[w].xmin[band] < cx0) cx0 = r->bins[w].xmin[band];
        if (r->bins[w].xmax[band] > cx1) cx1 = r->bins[w].xmax[band];
    }
    int x0, x1;
    int dx0 = cx0, dx1 = cx1;                       /* geometry + HUD */
    if (r->overlay_on && r->ov_y0 <= y1 && r->ov_y1 >= y0) {
        if (r->ov_x0 < dx0) dx0 = r->ov_x0;
        if (r->ov_x1 > dx1) dx1 = r->ov_x1;
    }
    if (r->full_frame) {
        x0 = 0; x1 = W - 1;
    } else {
        x0 = dx0; x1 = dx1;
        if (r->prev_x0[band] <= r->prev_x1[band]) {
            if (r->prev_x0[band] < x0) x0 = r->prev_x0[band];
            if (r->prev_x1[band] > x1) x1 = r->prev_x1[band];
        }
        if (x0 > x1) {               /* nothing now, nothing before */
            r->prev_x0[band] = 1; r->prev_x1[band] = 0;
            return false;
        }
        x0 &= ~1;                    /* panel wants even start / odd end */
        x1 |= 1;
        if (x1 > W - 1) x1 = W - 1;
    }
    r->prev_x0[band] = (int16_t)dx0;
    r->prev_x1[band] = (int16_t)dx1;

    const int stride = x1 - x0 + 1;
    const int rows = y1 - y0 + 1;
    const size_t npx = (size_t)stride * rows;
    memset(zbuf, 0xFF, npx * 2);
    if (r->bg_swapped == 0) memset(cbuf, 0, npx * 2);
    else for (size_t i = 0; i < npx; i++) cbuf[i] = r->bg_swapped;

    const r3d_model_t *m = r->model;
    if (m && cx0 <= cx1) {
        uint32_t ntri = 0;
        for (int w = 0; w < r->cfg.workers; w++) {
            const r3d_worker_bins_t *B = &r->bins[w];
            for (uint32_t k = B->start[band]; k < B->start[band + 1]; k++) {
                const uint32_t e = B->list[k];
                const int t = (int)(e & 0x7FFFFFFF);
                const int p = (int)(e >> 31);
                const r3d_lod_t *L = &m->lod[p];
                const uint16_t *ix = L->indices + 3 * t;
                const r3d_tv_t *tv = r->tv[p];
                raster_tri(&tv[ix[0]], &tv[ix[1]], &tv[ix[2]], &m->mat[L->tri_material[t]],
                           y0, y1, x0, x1, cbuf, zbuf, stride);
            }
            ntri += B->start[band + 1] - B->start[band];
        }
        __atomic_fetch_add(&r->stats.band_tris, ntri, __ATOMIC_RELAXED);
        if (r->edge_aa) edge_aa(cbuf, zbuf, stride, rows);
    }
    __atomic_fetch_add(&r->stats.bands_drawn, 1, __ATOMIC_RELAXED);
    __atomic_fetch_add(&r->stats.pixels_sent, (uint32_t)npx, __ATOMIC_RELAXED);
    out->x0 = x0; out->x1 = x1; out->y0 = y0; out->y1 = y1;
    return true;
}

#ifdef R3D_BENCH
/* micro benchmark hook: raster one triangle into caller buffers */
void r3d_bench_tri(const r3d_tv_t *a, const r3d_tv_t *b, const r3d_tv_t *c, const r3d_material_t *m,
                   int by0, int by1, int rx0, int rx1, uint16_t *cbuf, uint16_t *zbuf, int stride)
{
    raster_tri(a, b, c, m, by0, by1, rx0, rx1, cbuf, zbuf, stride);
}
#endif
