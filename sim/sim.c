/*
 * sim.c - host simulator for the r3d renderer.
 * Renders frames exactly like the ESP32 does (same bands, same fixed point) into a
 * framebuffer and writes binary PPM files.
 *
 *   sim model.bin out_prefix W H rotation frames [yaw0 yawstep pitch zoom refl]
 *
 * Also prints per-frame host timing and per-frame workload stats.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "r3d.h"

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

#ifdef R3D_COUNT
extern uint64_t r3d_cnt_setup, r3d_cnt_rows, r3d_cnt_spans, r3d_cnt_px, r3d_cnt_drawn;
#endif
static void *sim_alloc(size_t n, r3d_mem_t k) { (void)k; return malloc(n); }

int main(int argc, char **argv)
{
    if (argc < 7) {
        fprintf(stderr, "usage: %s model.bin out_prefix W H rotation frames [yaw0 yawstep pitch zoom refl]\n", argv[0]);
        return 1;
    }
    const char *path = argv[1], *prefix = argv[2];
    int W = atoi(argv[3]), H = atoi(argv[4]), rot = atoi(argv[5]), frames = atoi(argv[6]);
    float yaw0 = argc > 7 ? atof(argv[7]) : 35.0f;
    float ystep = argc > 8 ? atof(argv[8]) : 10.0f;
    float pitch = argc > 9 ? atof(argv[9]) : 18.0f;
    float zoom = argc > 10 ? atof(argv[10]) : 1.0f;
    int refl = argc > 11 ? atoi(argv[11]) : 1;

    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 1; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    void *blob = malloc(sz);
    if (fread(blob, 1, sz, f) != (size_t)sz) return 1;
    fclose(f);

    r3d_model_t model;
    r3d_err_t e = r3d_model_load(&model, blob, sz, sim_alloc, 96 * 1024, true);
    if (e != R3D_OK) { fprintf(stderr, "load: %s\n", r3d_err_str(e)); return 1; }
    printf("model '%s': lod0 %d verts %d tris, lod1 %d verts %d tris, %d mats, R=%.3f ymax=%.3f\n", model.name,
           model.lod[0].num_vertices, model.lod[0].num_triangles, model.lod[1].num_vertices,
           model.lod[1].num_triangles, model.num_materials, model.radius, model.ymax);

    r3d_t r;
    r3d_config_t cfg = {.width = W, .height = H, .band_h = 32, .rotation = rot, .workers = 2, .reflection = refl != 0, .alloc = sim_alloc};
    if (!r3d_init(&r, &cfg, model.lod[0].num_vertices, model.lod[0].num_triangles)) { fprintf(stderr, "init failed\n"); return 1; }
    r3d_set_model(&r, &model);

    uint16_t *fb = calloc((size_t)W * H, 2);
    uint16_t *cbuf = malloc((size_t)W * cfg.band_h * 2);
    uint16_t *zbuf = malloc((size_t)W * cfg.band_h * 2);
    uint8_t *rgb = malloc((size_t)W * H * 3);

    double total = 0;
    for (int fr = 0; fr < frames; fr++) {
        float yaw = (yaw0 + ystep * fr) * 0.0174533f;
        float pit = pitch * 0.0174533f;
        r3d_quat_t qy = r3d_quat_axis_angle(0, 1, 0, yaw);
        r3d_quat_t qx = r3d_quat_axis_angle(1, 0, 0, pit);
        r3d_view_t v = {.orient = r3d_quat_mul(qx, qy), .zoom = zoom, .fov_deg = 28, .autofit = true,
                        .reflection = refl != 0, .reflection_strength = 0.38f, .bg_color = 0x0000,
                        .edge_aa = getenv("R3D_AA") && atoi(getenv("R3D_AA"))};
        double t0 = now_ms();
        r3d_begin_frame(&r, &v);
        r3d_job_transform(&r, 0, 2);
        r3d_job_transform(&r, 1, 2);
        double t1 = now_ms();
        r3d_job_bin(&r, 0, 2);
        r3d_job_bin(&r, 1, 2);
        double t2 = now_ms();
        int sent = 0;
        for (int b = 0; b < r3d_band_count(&r); b++) {
            r3d_rect_t rc;
            if (!r3d_render_band(&r, b, cbuf, zbuf, &rc)) continue;
            int stride = rc.x1 - rc.x0 + 1;
            for (int y = rc.y0; y <= rc.y1; y++)
                memcpy(fb + (size_t)y * W + rc.x0, cbuf + (size_t)(y - rc.y0) * stride, stride * 2);
            sent++;
        }
        double t3 = now_ms();
        total += t3 - t0;
#ifdef R3D_COUNT
        printf("  setups %llu rows %llu spans %llu px-tests %llu drawn %llu\n", (unsigned long long)r3d_cnt_setup,
               (unsigned long long)r3d_cnt_rows, (unsigned long long)r3d_cnt_spans, (unsigned long long)r3d_cnt_px,
               (unsigned long long)r3d_cnt_drawn);
        r3d_cnt_setup = r3d_cnt_rows = r3d_cnt_spans = r3d_cnt_px = r3d_cnt_drawn = 0;
#endif
        printf("frame %3d: xform %.2f ms, bin %.2f ms, raster %.2f ms | binned %u, band-tris %u, bands %d, px %u (%.0f%%)\n",
               fr, t1 - t0, t2 - t1, t3 - t2, r.stats.tris_binned, r.stats.band_tris, sent, r.stats.pixels_sent,
               100.0 * r.stats.pixels_sent / (W * H));
        if (prefix[0] != '-') {
            for (size_t i = 0; i < (size_t)W * H; i++) {
                uint16_t c = fb[i];
                c = (uint16_t)((c >> 8) | (c << 8));   /* stored big-endian for SPI */
                rgb[i * 3 + 0] = (uint8_t)(((c >> 11) & 31) * 255 / 31);
                rgb[i * 3 + 1] = (uint8_t)(((c >> 5) & 63) * 255 / 63);
                rgb[i * 3 + 2] = (uint8_t)((c & 31) * 255 / 31);
            }
            char name[512];
            snprintf(name, sizeof(name), "%s%03d.ppm", prefix, fr);
            FILE *o = fopen(name, "wb");
            fprintf(o, "P6\n%d %d\n255\n", W, H);
            fwrite(rgb, 1, (size_t)W * H * 3, o);
            fclose(o);
        }
    }
    printf("avg host frame %.2f ms\n", total / frames);
    return 0;
}
