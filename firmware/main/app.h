/*
 * app.h - shared application state
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "r3d.h"

typedef enum { CTRL_TOUCH = 0, CTRL_MOTION = 1 } ctrl_mode_t;
typedef enum { SCREEN_VIEW3D = 0, SCREEN_SETTINGS = 1, SCREEN_ERROR = 2 } app_screen_t;

typedef enum { ROT_FREE = 0, ROT_YAW_ONLY = 1 } rot_mode_t;

typedef struct {
    uint8_t brightness;      /* 5..100 % */
    uint8_t mode;            /* ctrl_mode_t */
    uint8_t rotation;        /* 0..3: screen orientation (Settings: Landscape / Portrait) */
    bool    reflection;      /* fixed off (costs ~10 fps) */
    bool    auto_spin;
    bool    show_fps;
    uint8_t rot_mode;        /* rot_mode_t: free orbit or left/right only */
    uint8_t model_index;     /* selected model in the model pack */
    bool    edge_aa;         /* smooth stair-stepped edges (costs 1-3 fps) */
} app_settings_t;

extern app_settings_t g_settings;
extern r3d_t          g_r3d;
extern r3d_model_t    g_model;
extern bool           g_model_ok;
extern char           g_model_error[64];

void settings_load(void);
void settings_save(void);
void settings_apply_brightness(void);

/* screen switching (handled by the render task, which owns the display) */
void app_request_screen(app_screen_t s);
app_screen_t app_current_screen(void);

/* model pack: cycle to the next model (triple tap / settings button) */
void app_request_next_model(void);
void app_request_sleep(void);        /* BOOT button: fade out and enter deep sleep */
/* settings screen: orientation changed -> rotate LVGL and rebuild the screen */
void app_request_ui_rebuild(void);

/* battery as last measured by the input task */
int  app_battery_percent(bool *charging);
