/*
 * r3d.h - real-time software 3D renderer for small MCU displays (ESP32-S3 class)
 *
 * Pipeline per frame (each job can be split across N workers / CPU cores):
 *   r3d_begin_frame()            matrices, culling setup                (one worker)
 *   r3d_job_transform(w, N)      vertices -> screen space, matcap UV, AO (parallel)
 *   --- barrier ---
 *   r3d_job_bin(w, N)            back-face cull, bin triangles to bands  (parallel)
 *   --- barrier ---
 *   r3d_render_band(b, ...)      z-buffered matcap raster of one band    (parallel, any order)
 *
 * Bands are horizontal stripes of the *panel* (native scan order) so each finished band
 * can be pushed to the display with DMA while the next one renders. Only the dirty
 * x-range of a band (this frame + previous frame) is rendered and transmitted.
 *
 * Shading: per-material matcap (studio lighting baked by the converter) indexed by the
 * view-space normal, multiplied by baked per-vertex ambient occlusion. A mirrored
 * second pass with a height fade gives the floor reflection.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "r3d_math.h"
#include "r3d_model.h"

#ifdef __cplusplus
extern "C" {
#endif

#define R3D_MAX_WORKERS 2
#define R3D_PASSES      2      /* 0 = model, 1 = floor reflection */

typedef struct {
    int16_t  x, y;      /* panel coords, 12.4 fixed point */
    uint16_t z;         /* depth, 0 = near */
    uint16_t u, v;      /* matcap coords, 0.16 fixed (0..1) */
    uint8_t  a;         /* light factor 0..255 (AO * fade) */
    uint8_t  flags;
} r3d_tv_t;

typedef struct {
    int          width, height;  /* panel resolution */
    int          band_h;         /* rows per band, even */
    int          rotation;       /* 0..3 quarter turns: logical screen -> panel */
    int          workers;        /* 1..R3D_MAX_WORKERS */
    bool         reflection;     /* allocate the second (floor reflection) pass */
    r3d_alloc_fn alloc;
} r3d_config_t;

typedef struct {
    r3d_quat_t orient;           /* model -> view rotation */
    float      zoom;             /* 1.0 = fit width */
    float      shift_x, shift_y; /* logical pixels */
    float      fov_deg;          /* horizontal field of view (logical), e.g. 28 */
    bool       autofit;          /* keep the whole model on screen for any orientation */
    bool       reflection;
    float      reflection_strength; /* 0..1 */
    uint16_t   bg_color;         /* RGB565 */
    bool       edge_aa;          /* soften stair-steps on depth / contrast edges (~1-2 ms) */
} r3d_view_t;

typedef struct {
    int x0, y0, x1, y1;          /* inclusive panel rectangle */
} r3d_rect_t;

typedef struct {
    uint32_t tris_in, tris_binned, band_tris, bands_drawn, pixels_sent;
} r3d_stats_t;

typedef struct r3d_worker_bins {
    uint16_t *count;             /* [nbands] */
    uint32_t *start;             /* [nbands+1] */
    uint32_t *list;              /* entries: tri | pass << 31 */
    uint32_t  list_cap;
    int16_t  *xmin, *xmax;       /* [nbands] dirty x extents */
} r3d_worker_bins_t;

typedef struct {
    r3d_config_t       cfg;
    const r3d_model_t *model;
    int                logical_w, logical_h;
    int                nbands;
    int                tri_cap, vtx_cap;
    r3d_tv_t          *tv[R3D_PASSES];
    uint16_t          *tri_band;               /* [R3D_PASSES * tri_cap] b0 | b1 << 8 */
    r3d_worker_bins_t  bins[R3D_MAX_WORKERS];
    int16_t           *prev_x0, *prev_x1;      /* [nbands] last frame dirty range */
    bool               force_full;
    bool               full_frame;             /* this frame redraws everything */
    bool               overlay_on;             /* panel rect always redrawn (HUD text) */
    int                ov_x0, ov_y0, ov_x1, ov_y1;
    int                passes;                 /* 1 or 2 this frame */
    /* per-frame transform */
    float              M[R3D_PASSES][3][4];    /* quantised model -> view */
    float              N[3][3];                /* normal rotation (view) */
    float              focal, lcx, lcy;
    float              zk0, zk1;               /* z16 = zk0 - zk1 / w */
    float              refl_k, refl_inv_h;
    float              fit;                    /* smoothed auto-fit scale (0 = not yet set) */
    uint16_t           bg_swapped;
    bool               edge_aa;
    r3d_stats_t        stats;
} r3d_t;

bool r3d_init(r3d_t *r, const r3d_config_t *cfg, int max_vertices, int max_triangles);
bool r3d_set_model(r3d_t *r, const r3d_model_t *m);   /* false if larger than init caps */
void r3d_set_rotation(r3d_t *r, int rotation);     /* also forces a full redraw */
void r3d_invalidate(r3d_t *r);                     /* next frame redraws every band */

void r3d_begin_frame(r3d_t *r, const r3d_view_t *view);
void r3d_job_transform(r3d_t *r, int worker, int nworkers);
void r3d_job_bin(r3d_t *r, int worker, int nworkers);

static inline int r3d_band_count(const r3d_t *r) { return r->nbands; }

/*
 * Render one band. Buffers must hold width*band_h pixels. Returns false if the band is
 * empty this frame and last frame (nothing to send). On success `out` gets the panel
 * rectangle; color_buf holds (x1-x0+1)*(y1-y0+1) big-endian RGB565 pixels (ready for SPI).
 */
bool r3d_render_band(r3d_t *r, int band, uint16_t *color_buf, uint16_t *zbuf, r3d_rect_t *out);

/* panel rectangle that is always part of the dirty area (e.g. an FPS counter drawn by
 * the application into the band buffer after r3d_render_band) */
void r3d_set_overlay(r3d_t *r, bool on, int x0, int y0, int x1, int y1);

/* map logical screen pixel -> panel pixel for the current rotation */
void r3d_logical_to_panel(const r3d_t *r, int lx, int ly, int *px, int *py);

#ifdef __cplusplus
}
#endif
