/*
 * diag.c - stall detector, see diag.h
 *
 * Log lines:
 *   "diag: STALL render 1500 ms in 'bands' (disp: pixels queued) ..."  -> a task stopped
 *   "diag: alive | input 200/s render 33/s | i2c fails 0, recoveries 0 | heap ..."  every 10 s
 */
#include "diag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "diag";

diag_slot_t g_diag[DIAG_NUM];
volatile uint8_t g_diag_disp;
volatile uint32_t g_diag_i2c_fail, g_diag_i2c_recover;
volatile int32_t g_diag_i2c_last_err;

static const char *const s_task_name[DIAG_NUM] = {"input", "render", "helper"};

static const char *stage_name(int task, int s)
{
    static const char *const in[] = {"?", "delay", "imu_read", "ahrs", "touch_read", "view_update",
                                     "battery", "i2c_recover"};
    static const char *const rd[] = {"?", "loop_top", "model_switch", "enter_screen", "lvgl", "idle",
                                     "transform", "bin", "bands", "first_frame", "frame_delay"};
    static const char *const hp[] = {"?", "wait_job", "transform", "bin", "bands", "notify"};
    switch (task) {
    case DIAG_INPUT:  return (s >= 0 && s < (int)(sizeof(in) / sizeof(in[0]))) ? in[s] : "?";
    case DIAG_RENDER: return (s >= 0 && s < (int)(sizeof(rd) / sizeof(rd[0]))) ? rd[s] : "?";
    default:          return (s >= 0 && s < (int)(sizeof(hp) / sizeof(hp[0]))) ? hp[s] : "?";
    }
}

static const char *disp_name(int s)
{
    static const char *const n[] = {"idle", "lock taken", "window cmd (waits for previous DMA)", "pixels queued"};
    return (s >= 0 && s < 4) ? n[s] : "?";
}

static void monitor_task(void *arg)
{
    uint32_t last_hb[DIAG_NUM] = {0};
    int64_t last_move[DIAG_NUM];
    bool reported[DIAG_NUM] = {false};
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < DIAG_NUM; i++) last_move[i] = now;
    uint32_t hb_10s[DIAG_NUM] = {0};
    int64_t t_alive = now;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        now = esp_timer_get_time();
        for (int i = 0; i < DIAG_NUM; i++) {
            const uint32_t hb = g_diag[i].hb;
            if (hb != last_hb[i]) {
                if (reported[i])
                    ESP_LOGW(TAG, "%s running again after %d ms", s_task_name[i], (int)((now - last_move[i]) / 1000));
                last_hb[i] = hb;
                last_move[i] = now;
                reported[i] = false;
                continue;
            }
            if (g_diag[i].waiting) { last_move[i] = now; continue; }     /* idle on purpose */
            const int ms = (int)((now - last_move[i]) / 1000);
            if (ms >= 1500 && (!reported[i] || ms % 5000 < 500)) {
                reported[i] = true;
                ESP_LOGE(TAG, "STALL %s %d ms in '%s' | render '%s', helper '%s'%s, input '%s' | disp: %s | i2c fails %u, last err %s",
                         s_task_name[i], ms, stage_name(i, g_diag[i].stage),
                         stage_name(DIAG_RENDER, g_diag[DIAG_RENDER].stage),
                         stage_name(DIAG_HELPER, g_diag[DIAG_HELPER].stage),
                         g_diag[DIAG_HELPER].waiting ? " (waiting)" : "",
                         stage_name(DIAG_INPUT, g_diag[DIAG_INPUT].stage), disp_name(g_diag_disp),
                         (unsigned)g_diag_i2c_fail, esp_err_to_name(g_diag_i2c_last_err));
            }
        }
        if (now - t_alive >= 10000000) {
            const float s = (now - t_alive) * 1e-6f;
            ESP_LOGI(TAG, "alive | loops/s: input %.0f, render %.0f | i2c fails %u (last %s), recoveries %u | free internal %u",
                     (g_diag[DIAG_INPUT].hb - hb_10s[DIAG_INPUT]) / s, (g_diag[DIAG_RENDER].hb - hb_10s[DIAG_RENDER]) / s,
                     (unsigned)g_diag_i2c_fail, esp_err_to_name(g_diag_i2c_last_err), (unsigned)g_diag_i2c_recover,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
            for (int i = 0; i < DIAG_NUM; i++) hb_10s[i] = g_diag[i].hb;
            t_alive = now;
        }
    }
}

void diag_start(void)
{
    /* high priority, tiny work: still reports when a lower priority task spins */
    xTaskCreate(monitor_task, "diag", 3072, NULL, configMAX_PRIORITIES - 2, NULL);
}
