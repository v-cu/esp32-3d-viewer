/*
 * render.c - dual-core frame pipeline + display ownership
 *
 *   core 1: render task  (worker 0, frame orchestration, LVGL when settings are open)
 *   core 0: helper task  (worker 1)
 *
 * Per frame: transform (split) -> bin (split) -> bands (work stealing). Each finished
 * band is sent with DMA while the next one is rasterised.
 */
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "render.h"
#include "app.h"
#include "board.h"
#include "display.h"
#include "view_ctrl.h"
#include "lvgl_port.h"
#include "ui_settings.h"
#include "font5x7.h"
#include "model_store.h"
#include "diag.h"
#include "touch.h"
#include "imu.h"
#include "board.h"

/* at least one tick, whatever CONFIG_FREERTOS_HZ is */
#define MS_TICKS(ms) (pdMS_TO_TICKS(ms) > 0 ? pdMS_TO_TICKS(ms) : 1)
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "render";

r3d_t g_r3d;

#define WORKERS 2
static uint16_t *s_cbuf[WORKERS][2];
static uint16_t *s_zbuf[WORKERS];
static int s_toggle[WORKERS];
static atomic_int s_next_band;
static TaskHandle_t s_render_task, s_helper_task;

enum { JOB_TRANSFORM = 1, JOB_BIN, JOB_BANDS, JOB_QUIT };
static volatile int s_job;

static volatile app_screen_t s_req_screen = SCREEN_VIEW3D, s_cur_screen = (app_screen_t)-1;

/* HUD */
static char s_hud[16];
static int s_hud_lx = 8, s_hud_ly = 6, s_hud_scale = 2;

static volatile bool s_req_next_model;
static int64_t s_info_until;              /* show "2/3" in the HUD until this time */

void app_request_screen(app_screen_t s) { s_req_screen = s; }

static volatile bool s_req_rebuild;
void app_request_ui_rebuild(void) { s_req_rebuild = true; }
void app_request_next_model(void) { s_req_next_model = true; }

static volatile bool s_req_sleep;
void app_request_sleep(void) { s_req_sleep = true; }

static void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "going to sleep");
    disp_wait_idle();
    settings_save();
    /* short fade-out so it does not look like a crash */
    const int b0 = disp_get_brightness();
    for (int i = 7; i >= 0; i--) {
        disp_set_brightness((uint8_t)(b0 * i / 8));
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    disp_sleep();
    touch_sleep();
    imu_power_down();
    board_deep_sleep();                    /* does not return; BOOT wakes = normal boot */
}

/* free the current model and load the next one of the pack (render task only) */
static void switch_to_next_model(void)
{
    const int n = model_store_count();
    if (n < 2) return;
    disp_wait_idle();
    r3d_set_model(&g_r3d, NULL);
    if (g_model_ok) model_store_unload(&g_model);
    g_model_ok = false;
    int idx = (g_settings.model_index + 1) % n;
    for (int tries = 0; tries < n && !g_model_ok; tries++, idx = (idx + 1) % n) {
        r3d_err_t e = model_store_load(idx, &g_model);
        if (e == R3D_OK && r3d_set_model(&g_r3d, &g_model)) {
            g_model_ok = true;
            g_settings.model_index = (uint8_t)idx;
        } else {
            ESP_LOGW(TAG, "model #%d not usable (err %d, free internal %u, psram %u)", idx + 1, (int)e,
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            if (e == R3D_OK) model_store_unload(&g_model);
        }
    }
    if (!g_model_ok) snprintf(g_model_error, sizeof(g_model_error), "Model load failed");
    g_r3d.fit = 0;                         /* re-frame instantly for the new model */
    r3d_invalidate(&g_r3d);
    view_ctrl_reset();
    settings_save();
    s_info_until = esp_timer_get_time() + 1500000;
}
app_screen_t app_current_screen(void) { return s_cur_screen; }

uint16_t *render_band_buffer(int worker, int idx) { return s_cbuf[worker][idx]; }
size_t render_band_buffer_bytes(void) { return (size_t)g_board->width * (CONFIG_CAR3D_BAND_HEIGHT & ~1) * 2; }

/* ------------------------------------------------------------------------------------ */
static void hud_rect_panel(int *x0, int *y0, int *x1, int *y1)
{
    int w = (int)strlen("000 fps") * 6 * s_hud_scale, h = 8 * s_hud_scale;
    int ax, ay, bx, by;
    r3d_logical_to_panel(&g_r3d, s_hud_lx, s_hud_ly, &ax, &ay);
    r3d_logical_to_panel(&g_r3d, s_hud_lx + w - 1, s_hud_ly + h - 1, &bx, &by);
    *x0 = (ax < bx ? ax : bx) & ~1; *x1 = (ax > bx ? ax : bx) | 1;
    *y0 = ay < by ? ay : by; *y1 = ay > by ? ay : by;
}

static void hud_draw(const r3d_rect_t *rc, uint16_t *buf)
{
    if (!g_r3d.overlay_on) return;
    const int stride = rc->x1 - rc->x0 + 1;
    const uint16_t fg = 0xFFFF, dim = (uint16_t)((0x8410 >> 8) | (0x8410 << 8));
    for (int ci = 0; s_hud[ci]; ci++) {
        const uint8_t *g = font5x7_glyph(s_hud[ci]);
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                if (!(g[col] & (1 << row))) continue;
                for (int sy = 0; sy < s_hud_scale; sy++)
                    for (int sx = 0; sx < s_hud_scale; sx++) {
                        int lx = s_hud_lx + (ci * 6 + col) * s_hud_scale + sx;
                        int ly = s_hud_ly + row * s_hud_scale + sy;
                        int px, py;
                        r3d_logical_to_panel(&g_r3d, lx, ly, &px, &py);
                        if (px < rc->x0 || px > rc->x1 || py < rc->y0 || py > rc->y1) continue;
                        buf[(py - rc->y0) * stride + (px - rc->x0)] = ci >= 4 ? dim : fg;
                    }
            }
        }
    }
}

static void do_bands(int w)
{
    const int nb = r3d_band_count(&g_r3d);
    for (;;) {
        int b = atomic_fetch_add(&s_next_band, 1);
        if (b >= nb) break;
        uint16_t *buf = s_cbuf[w][s_toggle[w]];
        r3d_rect_t rc;
        if (!r3d_render_band(&g_r3d, b, buf, s_zbuf[w], &rc)) continue;
        hud_draw(&rc, buf);
        disp_draw(rc.x0, rc.y0, rc.x1, rc.y1, buf);   /* waits for the previous DMA, queues this one */
        s_toggle[w] ^= 1;
    }
}

static void helper_task(void *arg)
{
    for (;;) {
        DIAG_STAGE(DIAG_HELPER, DH_WAIT);
        g_diag[DIAG_HELPER].waiting = 1;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        g_diag[DIAG_HELPER].waiting = 0;
        DIAG_BEAT(DIAG_HELPER);
        switch (s_job) {
        case JOB_TRANSFORM: DIAG_STAGE(DIAG_HELPER, DH_TRANSFORM); r3d_job_transform(&g_r3d, 1, WORKERS); break;
        case JOB_BIN:       DIAG_STAGE(DIAG_HELPER, DH_BIN); r3d_job_bin(&g_r3d, 1, WORKERS); break;
        case JOB_BANDS:     DIAG_STAGE(DIAG_HELPER, DH_BANDS); do_bands(1); break;
        default: break;
        }
        DIAG_STAGE(DIAG_HELPER, DH_NOTIFY);
        xTaskNotifyGive(s_render_task);
    }
}

static inline void run_parallel(int job)
{
    s_job = job;
    xTaskNotifyGive(s_helper_task);
    switch (job) {
    case JOB_TRANSFORM: r3d_job_transform(&g_r3d, 0, WORKERS); break;
    case JOB_BIN:       r3d_job_bin(&g_r3d, 0, WORKERS); break;
    case JOB_BANDS:     do_bands(0); break;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

/* ------------------------------------------------------------------------------------ */
static void enter_screen(app_screen_t s)
{
    if (s == s_cur_screen) return;
    disp_wait_idle();
    if (s == SCREEN_VIEW3D) {
        view_ctrl_set_enabled(true);
        r3d_set_rotation(&g_r3d, g_settings.rotation);
        r3d_invalidate(&g_r3d);
    } else {
        view_ctrl_set_enabled(false);
        lvgl_port_set_rotation(g_settings.rotation);
        if (s == SCREEN_SETTINGS) ui_settings_show();
        else ui_error_show(g_model_error);
        lvgl_port_invalidate();
    }
    s_cur_screen = s;
}

static void render_task(void *arg)
{
    uint32_t last_seq = 0xFFFFFFFF;
    int64_t t_fps = esp_timer_get_time();
    int frames = 0, fps = 0;
    bool last_refl = !g_settings.reflection, last_hud = !g_settings.show_fps;
    const int64_t min_frame_us = 1000000 / CONFIG_CAR3D_MAX_FPS;
    bool first = true;
    int64_t acc_x = 0, acc_b = 0, acc_r = 0;
    int frames_log = 0, log_div = 0;

    bool last_info = false;
    bool last_aa = !g_settings.edge_aa;

    for (;;) {
        DIAG_BEAT(DIAG_RENDER);
        DIAG_STAGE(DIAG_RENDER, DR_TOP);
        if (s_req_sleep) enter_deep_sleep();
        if (s_req_next_model) {
            s_req_next_model = false;
            DIAG_STAGE(DIAG_RENDER, DR_MODEL_SWITCH);
            switch_to_next_model();
            if (s_cur_screen == SCREEN_SETTINGS) ui_settings_show();   /* refresh model name */
            last_seq = 0xFFFFFFFF;
        }
        if (s_req_rebuild) {                   /* orientation changed in the settings */
            s_req_rebuild = false;
            disp_wait_idle();
            if (s_cur_screen != SCREEN_VIEW3D) {
                lvgl_port_set_rotation(g_settings.rotation);
                if (s_cur_screen == SCREEN_SETTINGS) ui_settings_show();
                lvgl_port_invalidate();
            }
            last_hud = !last_hud;                  /* HUD position depends on the rotation */
        }
        app_screen_t want = s_req_screen;
        if (!g_model_ok && want == SCREEN_VIEW3D) want = SCREEN_ERROR;
        DIAG_STAGE(DIAG_RENDER, DR_ENTER_SCREEN);
        enter_screen(want);

        if (s_cur_screen != SCREEN_VIEW3D) {
            DIAG_STAGE(DIAG_RENDER, DR_LVGL);
            uint32_t wait = lvgl_port_run();
            if (first) {                 /* e.g. no model: light up for the error screen */
                disp_wait_idle();
                settings_apply_brightness();
                first = false;
            }
            vTaskDelay(MS_TICKS(wait < 5 ? 5 : (wait > 10 ? 10 : wait)));   /* touch read every 10 ms */
            last_seq = 0xFFFFFFFF;
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        view_state_t vs;
        view_ctrl_get(&vs);
        const bool info = esp_timer_get_time() < s_info_until;     /* "2/3" after a model switch */
        bool hud = g_settings.show_fps || info;
        if (info != last_info) { last_info = info; last_seq = 0xFFFFFFFF; }
        if (hud != last_hud) {
            int x0, y0, x1, y1;
            hud_rect_panel(&x0, &y0, &x1, &y1);
            r3d_set_overlay(&g_r3d, hud, x0, y0, x1, y1);
            last_hud = hud;
            last_seq = 0xFFFFFFFF;
        }
        if (g_settings.reflection != last_refl) { last_refl = g_settings.reflection; last_seq = 0xFFFFFFFF; }
        if (g_settings.edge_aa != last_aa) { last_aa = g_settings.edge_aa; last_seq = 0xFFFFFFFF; }
        if (vs.seq == last_seq && !g_r3d.force_full) {
            /* nothing moved: idle (keeps the FPS counter honest) */
            DIAG_STAGE(DIAG_RENDER, DR_IDLE);
            vTaskDelay(MS_TICKS(8));
            if (esp_timer_get_time() - t_fps > 1000000) { t_fps = esp_timer_get_time(); fps = frames = 0; }
            continue;
        }
        last_seq = vs.seq;

        r3d_view_t v = {
            .orient = vs.orient, .zoom = vs.zoom, .fov_deg = 28.0f, .autofit = true,
            .reflection = g_settings.reflection, .reflection_strength = 0.38f, .bg_color = 0x0000,
            .edge_aa = g_settings.edge_aa,
        };
        if (info) snprintf(s_hud, sizeof(s_hud), "%d/%d", g_settings.model_index + 1, model_store_count());
        else snprintf(s_hud, sizeof(s_hud), "%3d fps", fps);
        if (first) ESP_LOGI(TAG, "first frame: start");
        DIAG_STAGE(DIAG_RENDER, DR_TRANSFORM);
        r3d_begin_frame(&g_r3d, &v);
        run_parallel(JOB_TRANSFORM);
        int64_t t1 = esp_timer_get_time();
        if (first) ESP_LOGI(TAG, "first frame: transform done (%d us)", (int)(t1 - t0));
        DIAG_STAGE(DIAG_RENDER, DR_BIN);
        run_parallel(JOB_BIN);
        int64_t t2 = esp_timer_get_time();
        if (first) ESP_LOGI(TAG, "first frame: bin done (%d us), %u tris", (int)(t2 - t1), (unsigned)g_r3d.stats.tris_binned);
        atomic_store(&s_next_band, 0);
        DIAG_STAGE(DIAG_RENDER, DR_BANDS);
        run_parallel(JOB_BANDS);
        int64_t t3 = esp_timer_get_time();
        if (first) ESP_LOGI(TAG, "first frame: bands done (%d us), %u bands", (int)(t3 - t2), (unsigned)g_r3d.stats.bands_drawn);
        acc_x += t1 - t0; acc_b += t2 - t1; acc_r += t3 - t2;

        if (first) {                     /* light up only after the first complete frame */
            DIAG_STAGE(DIAG_RENDER, DR_FIRST_FRAME);
            disp_wait_idle();
            settings_apply_brightness();
            ESP_LOGI(TAG, "first frame on screen, brightness %d %%", g_settings.brightness);
            first = false;
        }
        frames++;
        frames_log++;
        int64_t now = esp_timer_get_time();
        if (now - t_fps >= 1000000) {
            fps = (int)((frames * 1000000LL + (now - t_fps) / 2) / (now - t_fps));
            frames = 0;
            t_fps = now;
            if (++log_div >= 2) {
                int n = frames_log ? frames_log : 1;
                ESP_LOGI(TAG, "fps %d | per frame: transform %d us, bin %d us, raster+send %d us | tris %u, px %u",
                         fps, (int)(acc_x / n), (int)(acc_b / n), (int)(acc_r / n),
                         (unsigned)g_r3d.stats.band_tris, (unsigned)g_r3d.stats.pixels_sent);
                acc_x = acc_b = acc_r = 0;
                frames_log = 0;
                log_div = 0;
            }
        }
        DIAG_STAGE(DIAG_RENDER, DR_FRAME_DELAY);
        int64_t spent = now - t0;
        if (spent < min_frame_us) vTaskDelay(MS_TICKS((min_frame_us - spent) / 1000 + 1));
        else vTaskDelay(1);              /* let the idle task run (task watchdog) */
    }
}

bool render_alloc_buffers(void)
{
    const board_desc_t *b = g_board;
    const int bh = CONFIG_CAR3D_BAND_HEIGHT & ~1;
    const size_t bytes = (size_t)b->width * bh * 2;
    for (int w = 0; w < WORKERS; w++) {
        for (int i = 0; i < 2; i++) {
            s_cbuf[w][i] = heap_caps_malloc(bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
            if (!s_cbuf[w][i]) return false;
        }
        s_zbuf[w] = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_zbuf[w]) return false;
    }
    return true;
}

bool render_init(void)
{
    const board_desc_t *b = g_board;
    const int bh = CONFIG_CAR3D_BAND_HEIGHT & ~1;
    int maxv = 4, maxt = 4;
    model_store_caps(&maxv, &maxt);             /* biggest model of the pack */
    if (maxv < 4) maxv = 4;
    if (maxt < 4) maxt = 4;
    r3d_config_t cfg = {
        .width = b->width, .height = b->height, .band_h = bh,
        .rotation = g_settings.rotation, .workers = WORKERS, .reflection = false, .alloc = render_alloc,
    };
    if (!r3d_init(&g_r3d, &cfg, maxv, maxt)) {
        ESP_LOGE(TAG, "renderer init failed (out of memory?)");
        return false;
    }
    if (g_model_ok && !r3d_set_model(&g_r3d, &g_model)) g_model_ok = false;
    ESP_LOGI(TAG, "renderer ready: %d bands of %d rows, free internal %u, psram %u", g_r3d.nbands, bh,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return true;
}

void render_start(void)
{
    /* helper first: the render task notifies it as soon as it starts */
    xTaskCreatePinnedToCore(helper_task, "r3d_helper", 6144, NULL, 4, &s_helper_task, 0);
    xTaskCreatePinnedToCore(render_task, "render", 8192, NULL, 5, &s_render_task, 1);
}

void *render_alloc(size_t n, r3d_mem_t kind)
{
    if (kind == R3D_MEM_FAST) return heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p ? p : heap_caps_malloc(n, MALLOC_CAP_8BIT);
}
