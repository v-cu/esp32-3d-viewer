/*
 * esp32-3d-viewer - real-time 3D model viewer for the Waveshare ESP32-S3-Touch-AMOLED-1.8
 *
 *   touch : drag = rotate (with inertia), double tap = reset,
 *           long press = settings
 *   motion: tilt the board left / right (like a steering wheel) to turn the model
 *
 * The model lives in its own flash partition ("model", see partitions.csv) and is
 * produced by tools/model_convert.py from any glTF/GLB/OBJ file.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_partition.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "app.h"
#include "board.h"
#include "display.h"
#include "touch.h"
#include "imu.h"
#include "view_ctrl.h"
#include "render.h"
#include "model_store.h"
#include "diag.h"
#include "lvgl_port.h"

static const char *TAG = "viewer";

r3d_model_t g_model;
bool g_model_ok;
char g_model_error[64];

static bool s_imu_ok;
static volatile int s_bat_pct = -1;
static volatile bool s_bat_chg;

int app_battery_percent(bool *charging)
{
    *charging = s_bat_chg;
    return s_bat_pct;
}

static void load_model(void)
{
    int n = model_store_init();
    if (n == 0) {
        snprintf(g_model_error, sizeof(g_model_error), "No valid model in flash");
        return;
    }
    if (g_settings.model_index >= n) g_settings.model_index = 0;
    r3d_err_t e = model_store_load(g_settings.model_index, &g_model);
    if (e != R3D_OK && g_settings.model_index != 0) {          /* fall back to the first one */
        g_settings.model_index = 0;
        e = model_store_load(0, &g_model);
    }
    if (e != R3D_OK) {
        snprintf(g_model_error, sizeof(g_model_error), "Model: %s", r3d_err_str(e));
        ESP_LOGE(TAG, "%s", g_model_error);
        return;
    }
    g_model_ok = true;
}

/* touch 100 Hz, IMU + fusion 200 Hz, battery every 5 s */
/*
 * I2C watchdog: touch and IMU share one bus. The ESP-IDF I2C master can get stuck
 * after a single glitch/timeout and then fails every transfer, which freezes touch
 * and motion together. After 25 failed reads in a row: reset the bus (9 clocks +
 * controller FSM reset); if it keeps failing, delete and re-create the whole bus.
 */
static void i2c_check(esp_err_t err)
{
    static int streak, level;
    static int64_t t_log;
    if (err == ESP_OK) {
        if (streak >= 25) level = 0;
        streak = 0;
        return;
    }
    g_diag_i2c_fail++;
    g_diag_i2c_last_err = err;
    if (++streak < 25) return;
    streak = 0;
    DIAG_STAGE(DIAG_INPUT, DI_I2C_RECOVER);
    g_diag_i2c_recover++;
    const int64_t now = esp_timer_get_time();
    if (now - t_log > 1000000) {
        ESP_LOGW(TAG, "I2C bus stuck (%s) -> %s", esp_err_to_name(err), level < 2 ? "bus reset" : "re-creating bus");
        t_log = now;
    }
    if (level < 2) {
        i2c_master_bus_reset(g_i2c_bus);
        level++;
    } else {
        touch_bus_detach();
        imu_bus_detach();
        board_i2c_rebuild();
        touch_bus_attach();
        imu_bus_attach();
        level = 0;
    }
}

static void input_task(void *arg)
{
    if (s_imu_ok) imu_calibrate(80);
    TickType_t last = xTaskGetTickCount();
    int64_t t_prev = esp_timer_get_time();
    int tick = 0;
    touch_data_t td = {0};
    int touch_bad = 0;
    float acc[3] = {0, 1, 0}, gyr[3] = {0, 0, 0};
    bool imu_valid = false;
    int64_t t_bat = 0;
    /* BOOT button: armed only after it was seen released (it may still be held after the
     * wake-up press); a press of >= 30 ms requests deep sleep */
    int btn_up_ms = 0, btn_down_ms = 0;
    bool btn_armed = false;
    for (;;) {
        DIAG_STAGE(DIAG_INPUT, DI_DELAY);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5) ? pdMS_TO_TICKS(5) : 1);
        DIAG_BEAT(DIAG_INPUT);
        if (board_boot_button_pressed()) {
            btn_up_ms = 0;
            btn_down_ms += 5;
            if (btn_armed && btn_down_ms >= 30) {
                app_request_sleep();
                /* stop using the I2C bus: the render task now powers everything down */
                g_diag[DIAG_INPUT].waiting = 1;
                for (;;) vTaskDelay(portMAX_DELAY);
            }
        } else {
            btn_down_ms = 0;
            if ((btn_up_ms += 5) >= 100) btn_armed = true;
        }
        int64_t now = esp_timer_get_time();
        float dt = (now - t_prev) * 1e-6f;
        t_prev = now;
        if (dt > 0.05f) dt = 0.05f;

        if (s_imu_ok) {
            float a[3], g[3];
            DIAG_STAGE(DIAG_INPUT, DI_IMU);
            bool ok = imu_read(a, g);
            i2c_check(imu_last_error());
            if (ok) {                        /* light low-pass, view_ctrl samples at 100 Hz */
                for (int k = 0; k < 3; k++) {
                    acc[k] += (a[k] - acc[k]) * 0.5f;
                    gyr[k] = g[k];
                }
                imu_valid = true;
            }
        }
        if (++tick & 1) {
            DIAG_STAGE(DIAG_INPUT, DI_TOUCH);
            touch_data_t now_td;
            if (touch_read(&now_td) || touch_last_error() == ESP_OK) {
                td = now_td;
                touch_bad = 0;
            } else if (++touch_bad > 5) {
                /* a missed read is not a finger lift (that would fake taps);
                 * only after ~50 ms of failed reads treat the screen as released */
                memset(&td, 0, sizeof(td));
            }
            i2c_check(touch_last_error());
            DIAG_STAGE(DIAG_INPUT, DI_VIEW);
            view_ctrl_update(&td, imu_valid ? acc : NULL, imu_valid ? gyr : NULL, dt * 2);
        }
        if (now - t_bat > 5000000) {
            bool chg;
            DIAG_STAGE(DIAG_INPUT, DI_BATTERY);
            s_bat_pct = board_battery_percent(&chg);
            s_bat_chg = chg;
            t_bat = now;
        }
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    board_init();
    settings_load();
    ESP_ERROR_CHECK(disp_init());
#if CONFIG_CAR3D_DISPLAY_SELFTEST
    disp_selftest();
#endif
    touch_init();
    s_imu_ok = imu_init();
    if (!s_imu_ok && g_settings.mode == CTRL_MOTION) g_settings.mode = CTRL_TOUCH;

    if (!render_alloc_buffers()) {
        ESP_LOGE(TAG, "not enough internal RAM for the band buffers");
        return;
    }
    load_model();
    view_ctrl_init();
    if (!render_init()) {
        ESP_LOGE(TAG, "not enough memory for the renderer");
        return;
    }
    lvgl_port_init(g_settings.rotation);
    xTaskCreatePinnedToCore(input_task, "input", 4096, NULL, 6, NULL, 0);
    render_start();
    diag_start();
}
