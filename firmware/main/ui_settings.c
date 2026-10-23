/*
 * ui_settings.c - Settings screen and error screen (LVGL 9).
 *
 * The 1.8" panel has ~317 ppi: 46 px is under 4 mm, a fingertip is 8-10 mm. So the
 * settings are a list of big rows (>= 72 px, 56 px buttons, large switches and slider)
 * under a fixed header; only the list scrolls, without momentum (a tap during a "thrown"
 * scroll would only stop it).
 */
#include <stdio.h>
#include "lvgl.h"
#include "ui_settings.h"
#include "app.h"
#include "board.h"
#include "lvgl_port.h"
#include "view_ctrl.h"
#include "render.h"
#include "model_store.h"

#define COL_BG      lv_color_hex(0x05070C)
#define COL_CARD    lv_color_hex(0x161C28)
#define COL_CARD2   lv_color_hex(0x262F42)
#define COL_ACCENT  lv_color_hex(0x3D7EFF)
#define COL_TEXT    lv_color_hex(0xF2F4F8)
#define COL_MUTED   lv_color_hex(0x8A93A6)

#define BTN_H       56                       /* ~4.5 mm on the 1.8" panel */
#define ROW_PAD     12

static lv_obj_t *s_scr;
static lv_obj_t *s_bat_label, *s_bri_value;
static lv_obj_t *s_btn_motion, *s_btn_touch, *s_btn_free, *s_btn_yaw, *s_btn_land, *s_btn_port;
static lv_timer_t *s_bat_timer;
static bool s_wide;

static lv_obj_t *label(lv_obj_t *parent, const char *txt, const lv_font_t *f, lv_color_t col)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, col, 0);
    return l;
}

static lv_obj_t *plain(lv_obj_t *parent)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

static void style_seg(lv_obj_t *btn, bool active)
{
    lv_obj_set_style_bg_color(btn, active ? COL_ACCENT : COL_CARD2, 0);
    lv_obj_set_style_text_color(btn, active ? lv_color_white() : COL_MUTED, 0);
}

static void update_battery(void)
{
    if (!s_bat_label) return;
    bool chg = false;
    int pct = app_battery_percent(&chg);
    char buf[24];
    if (pct < 0) snprintf(buf, sizeof(buf), chg ? LV_SYMBOL_USB : "");
    else {
        const char *sym = pct > 80 ? LV_SYMBOL_BATTERY_FULL : pct > 55 ? LV_SYMBOL_BATTERY_3 :
                          pct > 30 ? LV_SYMBOL_BATTERY_2 : pct > 10 ? LV_SYMBOL_BATTERY_1 : LV_SYMBOL_BATTERY_EMPTY;
        snprintf(buf, sizeof(buf), "%d%% %s%s", pct, chg ? LV_SYMBOL_CHARGE " " : "", sym);
    }
    lv_label_set_text(s_bat_label, buf);
}

static void bat_timer_cb(lv_timer_t *t) { update_battery(); }

/* ---------------------------------- callbacks --------------------------------------- */
static void on_back(lv_event_t *e)
{
    settings_save();
    app_request_screen(SCREEN_VIEW3D);
}

static void on_brightness(lv_event_t *e)
{
    lv_obj_t *s = lv_event_get_target(e);
    g_settings.brightness = (uint8_t)lv_slider_get_value(s);
    settings_apply_brightness();
    lv_label_set_text_fmt(s_bri_value, "%d%%", g_settings.brightness);
}

static void on_mode(lv_event_t *e)
{
    int mode = (int)(intptr_t)lv_event_get_user_data(e);
    g_settings.mode = (uint8_t)mode;
    if (mode == CTRL_MOTION) view_ctrl_recenter();
    style_seg(s_btn_motion, mode == CTRL_MOTION);
    style_seg(s_btn_touch, mode == CTRL_TOUCH);
}

static void on_switch(lv_event_t *e)
{
    bool *flag = (bool *)lv_event_get_user_data(e);
    *flag = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
}

static void on_row_toggle(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_user_data(e);
    if (lv_obj_has_state(sw, LV_STATE_CHECKED)) lv_obj_remove_state(sw, LV_STATE_CHECKED);
    else lv_obj_add_state(sw, LV_STATE_CHECKED);
    lv_obj_send_event(sw, LV_EVENT_VALUE_CHANGED, NULL);
}

static void on_rot_mode(lv_event_t *e)
{
    int m = (int)(intptr_t)lv_event_get_user_data(e);
    g_settings.rot_mode = (uint8_t)m;
    style_seg(s_btn_free, m == ROT_FREE);
    style_seg(s_btn_yaw, m == ROT_YAW_ONLY);
}

static void on_orientation(lv_event_t *e)
{
    const int rot = (int)(intptr_t)lv_event_get_user_data(e);
    if (rot == g_settings.rotation) return;
    g_settings.rotation = (uint8_t)rot;
    settings_save();
    app_request_ui_rebuild();          /* render task rotates LVGL and rebuilds this screen */
}

static void on_next_model(lv_event_t *e)
{
    app_request_next_model();          /* handled by the render task, refreshes this screen */
}

static void on_reset_view(lv_event_t *e)
{
    view_ctrl_reset();
    settings_save();
    app_request_screen(SCREEN_VIEW3D);
}

/* ---------------------------------- building blocks --------------------------------- */
static lv_obj_t *button(lv_obj_t *parent, const char *txt, lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = lv_button_create(parent);
    lv_obj_set_height(b, BTN_H);
    lv_obj_set_style_radius(b, 14, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_pad_hor(b, 10, 0);
    lv_obj_set_style_bg_color(b, COL_CARD2, 0);
    lv_obj_set_style_text_color(b, COL_TEXT, 0);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    return b;
}

static lv_obj_t *seg(lv_obj_t *parent, const char *txt, bool active, lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = button(parent, txt, cb, ud);
    lv_obj_set_flex_grow(b, 1);
    lv_obj_set_width(b, 1);
    style_seg(b, active);
    return b;
}

/*
 * One list row: dark card with the title on the left (landscape) or on top (portrait)
 * and a control area. Returns the control area (a flex row).
 */
static lv_obj_t *item(lv_obj_t *list, const char *title, bool stacked, lv_obj_t **title_row)
{
    lv_obj_t *c = plain(list);
    lv_obj_set_width(c, lv_pct(100));
    lv_obj_set_height(c, LV_SIZE_CONTENT);
    lv_obj_set_style_min_height(c, 72, 0);
    lv_obj_set_style_bg_color(c, COL_CARD, 0);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(c, 16, 0);
    lv_obj_set_style_pad_all(c, ROW_PAD, 0);
    lv_obj_set_style_pad_gap(c, 10, 0);
    stacked = stacked || !s_wide;
    lv_obj_set_flex_flow(c, stacked ? LV_FLEX_FLOW_COLUMN : LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(c, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    lv_obj_t *tr = plain(c);
    lv_obj_set_flex_flow(tr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(tr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_height(tr, LV_SIZE_CONTENT);
    if (stacked) lv_obj_set_width(tr, lv_pct(100));
    else lv_obj_set_width(tr, 150);
    if (title) label(tr, title, &lv_font_montserrat_24, COL_TEXT);
    if (title_row) *title_row = tr;

    lv_obj_t *ctl = plain(c);
    lv_obj_set_flex_flow(ctl, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctl, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(ctl, 10, 0);
    lv_obj_set_height(ctl, LV_SIZE_CONTENT);
    if (stacked) lv_obj_set_width(ctl, lv_pct(100));
    else {
        lv_obj_set_width(ctl, 1);
        lv_obj_set_flex_grow(ctl, 1);
    }
    return ctl;
}

static void switch_item(lv_obj_t *list, const char *title, bool *flag)
{
    lv_obj_t *c = plain(list);
    lv_obj_set_size(c, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_min_height(c, 72, 0);
    lv_obj_set_style_bg_color(c, COL_CARD, 0);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(c, 16, 0);
    lv_obj_set_style_pad_all(c, ROW_PAD, 0);
    lv_obj_set_flex_flow(c, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(c, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    label(c, title, &lv_font_montserrat_24, COL_TEXT);
    lv_obj_t *sw = lv_switch_create(c);
    lv_obj_set_size(sw, 84, 44);
    lv_obj_set_style_bg_color(sw, COL_CARD2, 0);
    lv_obj_set_style_bg_color(sw, COL_ACCENT, LV_PART_INDICATOR | LV_STATE_CHECKED);
    if (*flag) lv_obj_add_state(sw, LV_STATE_CHECKED);
    lv_obj_add_event_cb(sw, on_switch, LV_EVENT_VALUE_CHANGED, flag);
    /* the whole row toggles: a tap anywhere on it flips the switch */
    lv_obj_add_flag(c, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(c, on_row_toggle, LV_EVENT_CLICKED, sw);
}

void ui_settings_show(void)
{
    if (!s_scr) {
        s_scr = lv_obj_create(NULL);
        s_bat_timer = lv_timer_create(bat_timer_cb, 2000, NULL);
    }
    int sw, sh;
    lvgl_port_logical_size(&sw, &sh);
    s_wide = sw > sh;

    lv_obj_clean(s_scr);
    s_bat_label = NULL;
    lv_obj_set_style_bg_color(s_scr, COL_BG, 0);
    lv_obj_set_style_bg_opa(s_scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_scr, 0, 0);
    lv_obj_set_style_pad_row(s_scr, 0, 0);
    lv_obj_set_flex_flow(s_scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_scr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_remove_flag(s_scr, LV_OBJ_FLAG_SCROLLABLE);

    /* fixed header: [ < ]  Settings ........ battery */
    lv_obj_t *hdr = plain(s_scr);
    lv_obj_set_size(hdr, lv_pct(100), 72);
    lv_obj_set_style_pad_hor(hdr, 8, 0);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(hdr, 14, 0);
    lv_obj_t *back = button(hdr, LV_SYMBOL_LEFT, on_back, NULL);
    lv_obj_set_size(back, 84, BTN_H);
    lv_obj_set_style_radius(back, BTN_H / 2, 0);
    lv_obj_set_style_text_font(lv_obj_get_child(back, 0), &lv_font_montserrat_24, 0);
    lv_obj_t *title = label(hdr, "Settings", &lv_font_montserrat_24, COL_TEXT);
    lv_obj_set_width(title, 1);
    lv_obj_set_flex_grow(title, 1);
    lv_label_set_long_mode(title, LV_LABEL_LONG_CLIP);
    s_bat_label = label(hdr, "", &lv_font_montserrat_16, COL_MUTED);
    update_battery();

    /* scrolling list of big rows */
    lv_obj_t *list = lv_obj_create(s_scr);
    lv_obj_remove_style_all(list);
    lv_obj_set_width(list, lv_pct(100));
    lv_obj_set_flex_grow(list, 1);
    lv_obj_set_style_pad_hor(list, 8, 0);
    lv_obj_set_style_pad_bottom(list, 16, 0);
    lv_obj_set_style_pad_row(list, 8, 0);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(list, LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_ELASTIC);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);

    /* brightness: title + value, big slider below */
    lv_obj_t *tr;
    lv_obj_t *ctl = item(list, LV_SYMBOL_EYE_OPEN "  Brightness", true, &tr);
    s_bri_value = label(tr, "", &lv_font_montserrat_24, COL_MUTED);
    lv_label_set_text_fmt(s_bri_value, "%d%%", g_settings.brightness);
    lv_obj_set_style_pad_hor(ctl, 22, 0);               /* room for the knob */
    lv_obj_t *sl = lv_slider_create(ctl);
    lv_obj_set_width(sl, lv_pct(100));
    lv_obj_set_height(sl, 20);
    lv_slider_set_range(sl, 5, 100);
    lv_slider_set_value(sl, g_settings.brightness, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(sl, COL_CARD2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sl, COL_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sl, lv_color_white(), LV_PART_KNOB);
    lv_obj_set_style_pad_all(sl, 12, LV_PART_KNOB);
    lv_obj_set_style_margin_ver(sl, 16, 0);
    lv_obj_set_ext_click_area(sl, 24);
    lv_obj_add_event_cb(sl, on_brightness, LV_EVENT_VALUE_CHANGED, NULL);

    ctl = item(list, "Control", false, NULL);
    s_btn_motion = seg(ctl, "Motion", g_settings.mode == CTRL_MOTION, on_mode, (void *)(intptr_t)CTRL_MOTION);
    s_btn_touch = seg(ctl, "Touch", g_settings.mode == CTRL_TOUCH, on_mode, (void *)(intptr_t)CTRL_TOUCH);

    ctl = item(list, "Rotation", false, NULL);
    s_btn_free = seg(ctl, "Free", g_settings.rot_mode == ROT_FREE, on_rot_mode, (void *)(intptr_t)ROT_FREE);
    s_btn_yaw = seg(ctl, "Left/Right", g_settings.rot_mode == ROT_YAW_ONLY, on_rot_mode,
                    (void *)(intptr_t)ROT_YAW_ONLY);

    ctl = item(list, "Screen", false, NULL);
    s_btn_land = seg(ctl, "Landscape", g_settings.rotation & 1, on_orientation, (void *)(intptr_t)3);
    s_btn_port = seg(ctl, "Portrait", !(g_settings.rotation & 1), on_orientation, (void *)(intptr_t)0);

    /* model: one big button "name  2/4  >" = next model */
    ctl = item(list, "Model", false, NULL);
    char info[96];
    const int n = model_store_count();
    if (g_model_ok) snprintf(info, sizeof(info), "%s  %d/%d  " LV_SYMBOL_NEXT, g_model.name, g_settings.model_index + 1, n);
    else snprintf(info, sizeof(info), "no model");
    lv_obj_t *mb = seg(ctl, info, false, on_next_model, NULL);
    lv_obj_set_style_text_color(mb, COL_TEXT, 0);
    lv_label_set_long_mode(lv_obj_get_child(mb, 0), LV_LABEL_LONG_DOT);
    lv_obj_set_width(lv_obj_get_child(mb, 0), lv_pct(100));
    lv_obj_set_style_text_align(lv_obj_get_child(mb, 0), LV_TEXT_ALIGN_CENTER, 0);
    if (n < 2) lv_obj_add_state(mb, LV_STATE_DISABLED);

    switch_item(list, "Auto spin", &g_settings.auto_spin);
    switch_item(list, "Smooth edges", &g_settings.edge_aa);
    switch_item(list, "FPS counter", &g_settings.show_fps);

    ctl = item(list, NULL, true, NULL);
    lv_obj_t *b = seg(ctl, LV_SYMBOL_HOME "  Reset view", false, on_reset_view, NULL);
    lv_obj_set_style_text_color(b, COL_TEXT, 0);

    lv_obj_t *tip = label(list, "2x tap: reset view    3x tap: next model\n"
                                "BOOT button: sleep / wake",
                          &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_long_mode(tip, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(tip, lv_pct(100));

    lv_screen_load(s_scr);
}

void ui_error_show(const char *msg)
{
    if (!s_scr) {
        s_scr = lv_obj_create(NULL);
        s_bat_timer = lv_timer_create(bat_timer_cb, 2000, NULL);
    }
    lv_obj_clean(s_scr);
    s_bat_label = NULL;
    lv_obj_set_style_bg_color(s_scr, COL_BG, 0);
    lv_obj_set_flex_flow(s_scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_scr, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(s_scr, 16, 0);
    label(s_scr, LV_SYMBOL_WARNING, &lv_font_montserrat_24, lv_color_hex(0xFFB020));
    lv_obj_t *l = label(s_scr, msg, &lv_font_montserrat_16, COL_TEXT);
    lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(l, lv_pct(100));
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_t *h = label(s_scr, "Flash a model: idf.py flash  (or tools/flash_model.py)", &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_long_mode(h, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(h, lv_pct(100));
    lv_obj_set_style_text_align(h, LV_TEXT_ALIGN_CENTER, 0);
    lv_screen_load(s_scr);
}
