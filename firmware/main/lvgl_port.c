/*
 * lvgl_port.c - LVGL 9 display/touch glue. LVGL only runs while the settings (or error)
 * screen is shown; the band buffers of the 3D renderer double as LVGL draw buffers.
 * Software rotation + RGB565 byte swap happen in the flush callback.
 */
#include <string.h>
#include "lvgl.h"
#include "lvgl_port.h"
#include "render.h"
#include "display.h"
#include "board.h"
#include "view_ctrl.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static lv_display_t *s_disp;
static int s_rot;
static int s_tmp_idx;

static uint32_t tick_cb(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void logical_size(int rot, int *w, int *h)
{
    *w = (rot & 1) ? g_board->height : g_board->width;
    *h = (rot & 1) ? g_board->width : g_board->height;
}

static void flush_cb(lv_display_t *disp, const lv_area_t *a, uint8_t *px_map)
{
    const int W = g_board->width, H = g_board->height;
    const int aw = a->x2 - a->x1 + 1, ah = a->y2 - a->y1 + 1;
    uint16_t *src = (uint16_t *)px_map;
    if (s_rot == 0) {
        for (int i = 0; i < aw * ah; i++) src[i] = (uint16_t)((src[i] >> 8) | (src[i] << 8));
        disp_draw(a->x1, a->y1, a->x2, a->y2, src);
    } else {
        uint16_t *dst = render_band_buffer(1, s_tmp_idx);
        s_tmp_idx ^= 1;
        int px0, py0, px1, py1;
        switch (s_rot) {
        case 1: px0 = W - 1 - a->y2; px1 = W - 1 - a->y1; py0 = a->x1; py1 = a->x2; break;
        case 2: px0 = W - 1 - a->x2; px1 = W - 1 - a->x1; py0 = H - 1 - a->y2; py1 = H - 1 - a->y1; break;
        default: px0 = a->y1; px1 = a->y2; py0 = H - 1 - a->x2; py1 = H - 1 - a->x1; break;
        }
        const int pw = px1 - px0 + 1;
        for (int y = 0; y < ah; y++) {
            const uint16_t *s = src + y * aw;
            const int ly = a->y1 + y;
            for (int x = 0; x < aw; x++) {
                const int lx = a->x1 + x;
                int px, py;
                switch (s_rot) {
                case 1: px = W - 1 - ly; py = lx; break;
                case 2: px = W - 1 - lx; py = H - 1 - ly; break;
                default: px = ly; py = H - 1 - lx; break;
                }
                uint16_t c = s[x];
                dst[(py - py0) * pw + (px - px0)] = (uint16_t)((c >> 8) | (c << 8));
            }
        }
        disp_draw(px0, py0, px1, py1, dst);
    }
    lv_display_flush_ready(disp);
}

static void rounder_cb(lv_event_t *e)
{
    lv_area_t *a = (lv_area_t *)lv_event_get_param(e);
    int w, h;
    logical_size(s_rot, &w, &h);
    a->x1 &= ~1; a->y1 &= ~1;
    a->x2 |= 1;  a->y2 |= 1;
    if (a->x2 > w - 1) a->x2 = w - 1;
    if (a->y2 > h - 1) a->y2 = h - 1;
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    int x, y;
    bool down = view_ctrl_last_touch(&x, &y);
    data->point.x = x;
    data->point.y = y;
    data->state = down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/* CO5300 windows must start on even and end on odd coordinates: give LVGL a buffer that
 * holds an EVEN number of full logical rows, so every partial chunk keeps that alignment */
static void apply_buffers(void)
{
    int w, h;
    logical_size(s_rot, &w, &h);
    size_t rows = render_band_buffer_bytes() / ((size_t)w * 2);
    rows &= ~(size_t)1;
    lv_display_set_buffers(s_disp, render_band_buffer(0, 0), render_band_buffer(0, 1),
                           rows * (size_t)w * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
}

void lvgl_port_init(int rotation)
{
    lv_init();
    lv_tick_set_cb(tick_cb);
    s_rot = rotation & 3;
    int w, h;
    logical_size(s_rot, &w, &h);
    s_disp = lv_display_create(w, h);
    lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(s_disp, flush_cb);
    apply_buffers();
    lv_display_add_event_cb(s_disp, rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    lv_indev_t *in = lv_indev_create();
    lv_indev_set_type(in, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(in, touch_read_cb);
    /* a finger wobbles a few pixels while tapping: only start scrolling after 24 px, so
     * buttons and the slider keep their press */
    lv_indev_set_scroll_limit(in, 24);
    lv_timer_set_period(lv_indev_get_read_timer(in), 10);   /* default 30 ms misses quick taps */
#if CONFIG_CAR3D_TOUCH_CURSOR
    lv_obj_t *cur = lv_obj_create(lv_layer_sys());
    lv_obj_remove_style_all(cur);
    lv_obj_set_size(cur, 12, 12);
    lv_obj_set_style_radius(cur, 6, 0);
    lv_obj_set_style_bg_color(cur, lv_color_hex(0xFF3030), 0);
    lv_obj_set_style_bg_opa(cur, LV_OPA_COVER, 0);
    lv_indev_set_cursor(in, cur);
#endif
}

void lvgl_port_set_rotation(int rotation)
{
    s_rot = rotation & 3;
    int w, h;
    logical_size(s_rot, &w, &h);
    lv_display_set_resolution(s_disp, w, h);
    apply_buffers();
}

void lvgl_port_invalidate(void) { lv_obj_invalidate(lv_screen_active()); }

uint32_t lvgl_port_run(void) { return lv_timer_handler(); }

void lvgl_port_logical_size(int *w, int *h) { logical_size(s_rot, w, h); }
