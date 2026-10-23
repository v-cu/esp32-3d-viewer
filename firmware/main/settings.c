/*
 * settings.c - persistent user settings (NVS)
 */
#include <string.h>
#include "app.h"
#include "board.h"
#include "display.h"
#include "sdkconfig.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

app_settings_t g_settings;

#define NVS_NS  "car3d"           /* kept so existing settings survive the rename */
#define NVS_KEY "cfg_v3"

static void defaults(void)
{
    g_settings.brightness = 80;
    g_settings.mode = CTRL_TOUCH;
    g_settings.rotation = (uint8_t)g_board->default_rotation;
    g_settings.reflection = false;
    g_settings.rot_mode = ROT_FREE;
    g_settings.model_index = 0;
    g_settings.auto_spin = true;
    g_settings.edge_aa = true;
#if CONFIG_CAR3D_SHOW_FPS_DEFAULT
    g_settings.show_fps = true;
#else
    g_settings.show_fps = false;
#endif
}

void settings_load(void)
{
    defaults();
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    app_settings_t s;
    size_t len = sizeof(s);
    if (nvs_get_blob(h, NVS_KEY, &s, &len) == ESP_OK && len == sizeof(s)) {
        if (s.brightness >= 5 && s.brightness <= 100 && s.mode <= 1 && s.rot_mode <= 1 && s.rotation <= 3)
            g_settings = s;
    }
    nvs_close(h);
    /* fixed product decision: no floor reflection */
    g_settings.reflection = false;
}

void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY, &g_settings, sizeof(g_settings));
    nvs_commit(h);
    nvs_close(h);
}

void settings_apply_brightness(void)
{
    /* perceptual curve: 5..100 % -> 8..255 */
    int p = g_settings.brightness;
    int v = 8 + (p * p * 247) / 10000;
    disp_set_brightness((uint8_t)v);
}
