/*
 * touch.c - CST820 capacitive touch (Waveshare AMOLED 1.8 V2), polled over I2C
 *
 * Register layout (CST8xx family): 0x02 finger count, 0x03/0x04 X high/low,
 * 0x05/0x06 Y high/low. Single touch only (no pinch zoom on this controller).
 * 0xFE = 1 disables auto sleep (otherwise it stops answering after a few seconds
 * without a touch).
 */
#include <string.h>
#include "touch.h"
#include "board.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "touch";
static i2c_master_dev_handle_t s_dev;
static bool s_present;                       /* controller added at boot */
static esp_err_t s_last_err = ESP_OK;

#define TOUCH_I2C_HZ   200000

static esp_err_t disable_auto_sleep(void)
{
    const uint8_t w[2] = {0xFE, 0x01};
    return i2c_master_transmit(s_dev, w, 2, 50);
}

static bool add_device(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = g_board->touch_addr,
        .scl_speed_hz = TOUCH_I2C_HZ,
    };
    if (i2c_master_bus_add_device(g_i2c_bus, &cfg, &s_dev) != ESP_OK) {
        s_dev = NULL;
        return false;
    }
    return true;
}

bool touch_init(void)
{
    if (!add_device()) return false;
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 10 && err != ESP_OK; i++) {     /* controller boots slower than the MCU */
        err = disable_auto_sleep();
        if (err != ESP_OK) vTaskDelay(pdMS_TO_TICKS(30));
    }
    /* keep the device even if it did not answer: an already sleeping CST820 only wakes up
     * on a touch, after that the reads work (and touch_read re-sends the sleep disable) */
    if (err != ESP_OK) ESP_LOGW(TAG, "CST820 @0x%02X not answering yet (touch the screen)", g_board->touch_addr);
    s_present = true;
    /* polled at 100 Hz: a missed read (e.g. controller busy) is harmless, don't spam logs */
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    ESP_LOGI(TAG, "CST820 ready");
    return true;
}

static void map_point(int rx, int ry, int *x, int *y)
{
    const board_desc_t *b = g_board;
    if (b->touch_swap_xy) { int t = rx; rx = ry; ry = t; }
    if (b->touch_mirror_x) rx = b->width - 1 - rx;
    if (b->touch_mirror_y) ry = b->height - 1 - ry;
    if (rx < 0) rx = 0;
    if (rx >= b->width) rx = b->width - 1;
    if (ry < 0) ry = 0;
    if (ry >= b->height) ry = b->height - 1;
    *x = rx; *y = ry;
}

bool touch_read(touch_data_t *t)
{
    static bool sleep_disabled = false;
    memset(t, 0, sizeof(*t));
    if (!s_dev) return false;
    uint8_t reg = 0x02, buf[5];
    s_last_err = i2c_master_transmit_receive(s_dev, &reg, 1, buf, sizeof(buf), 30);
    if (s_last_err != ESP_OK) {
        sleep_disabled = false;
        return false;
    }
    if (!sleep_disabled) sleep_disabled = disable_auto_sleep() == ESP_OK;
    const int n = (buf[0] & 0x0F) ? 1 : 0;
    if (n) {
        int rx = ((buf[1] & 0x0F) << 8) | buf[2];
        int ry = ((buf[3] & 0x0F) << 8) | buf[4];
        map_point(rx, ry, &t->x[0], &t->y[0]);
    }
    t->n = n;
    return true;
}

esp_err_t touch_last_error(void) { return s_dev ? s_last_err : ESP_OK; }

void touch_bus_detach(void)
{
    if (s_dev) i2c_master_bus_rm_device(s_dev);
    s_dev = NULL;
}

void touch_bus_attach(void)
{
    if (!s_present || s_dev) return;
    if (!add_device()) return;
    s_last_err = ESP_OK;
    disable_auto_sleep();
}

void touch_sleep(void)
{
    if (!s_dev) return;
    const uint8_t w[2] = {0xE5, 0x03};               /* CST8xx: enter deep sleep */
    i2c_master_transmit(s_dev, w, 2, 50);
}
