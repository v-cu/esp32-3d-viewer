/*
 * display.c - CO5300 AMOLED over QSPI (esp_lcd panel IO, no vendor driver needed)
 *
 * QSPI framing used by CO5300 / SH8601:
 *   register write : opcode 0x02, 24-bit address = cmd << 8, params on 1 line
 *   pixel write    : opcode 0x32, address 0x2C << 8, pixels on 4 lines
 * Pixels are RGB565 big-endian. Window coordinates must be even-aligned.
 *
 * Threading: the ESP-IDF SPI master is NOT thread safe for one device used from
 * several tasks (even with an external mutex its queued/polled transactions and bus
 * lock can deadlock). Both render workers (core 0 and core 1) send bands, so every
 * panel access goes through ONE display task. disp_*() post a request and wait until
 * the display task has executed it (for pixels: until they are queued for DMA).
 */
#include <string.h>
#include "display.h"
#include "board.h"
#include "diag.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "disp";

#define LCD_HOST       SPI2_HOST
#define QSPI_CMD(c)    ((0x02u << 24) | ((uint32_t)(c) << 8))
#define QSPI_PIXELS    ((0x32u << 24) | (0x2Cu << 8))

static esp_lcd_panel_io_handle_t s_io;
typedef enum { REQ_DRAW, REQ_WAIT_IDLE, REQ_CMD, REQ_SLEEP } req_kind_t;

typedef struct {
    req_kind_t kind;
    int x0, y0, x1, y1;
    const void *data;
    uint8_t cmd, param;
    SemaphoreHandle_t done;
} disp_req_t;

static QueueHandle_t s_req_q;
static TaskHandle_t s_disp_task;
static bool s_started;                      /* false during disp_init: calls run inline */
static uint8_t s_brightness = 0xFF;
static void disp_task(void *arg);

static void tx_cmd(uint8_t cmd, const uint8_t *data, int len)
{
    esp_lcd_panel_io_tx_param(s_io, (int)QSPI_CMD(cmd), len ? data : NULL, len);
}

esp_err_t disp_init(void)
{
    const board_desc_t *b = g_board;

    spi_bus_config_t bus = {
        .sclk_io_num = b->pin_clk,
        .data0_io_num = b->pin_d0,
        .data1_io_num = b->pin_d1,
        .data2_io_num = b->pin_d2,
        .data3_io_num = b->pin_d3,
        .max_transfer_sz = b->width * 64 * 2,
        .flags = SPICOMMON_BUSFLAG_QUAD,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO), TAG, "spi bus");

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = b->pin_cs,
        .dc_gpio_num = -1,
        .spi_mode = 0,
        .pclk_hz = CONFIG_CAR3D_QSPI_MHZ * 1000 * 1000,
        .trans_queue_depth = 6,
        .lcd_cmd_bits = 32,
        .lcd_param_bits = 8,
        .flags = {.quad_mode = 1},
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &s_io), TAG, "io");

    if (b->pin_rst >= 0) {
        gpio_config_t g = {.pin_bit_mask = 1ULL << b->pin_rst, .mode = GPIO_MODE_OUTPUT};
        gpio_config(&g);
        gpio_set_level(b->pin_rst, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(b->pin_rst, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    for (int i = 0; i < b->init_cmds_count; i++) {
        const panel_init_cmd_t *c = &b->init_cmds[i];
        tx_cmd(c->cmd, c->data, c->len);
        if (c->delay_ms) vTaskDelay(pdMS_TO_TICKS(c->delay_ms));
    }

    /* clear the whole GRAM (incl. offset area) to black before lighting up */
    const int rows = 16;
    uint16_t *blk = heap_caps_calloc(b->width * rows, 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (blk) {
        for (int y = 0; y < b->height; y += rows) {
            int y1 = y + rows - 1;
            if (y1 >= b->height) y1 = b->height - 1;
            disp_draw(0, y, b->width - 1, y1, blk);
        }
        disp_wait_idle();
        free(blk);
    }
    s_req_q = xQueueCreate(8, sizeof(disp_req_t));
    /* same core as the SPI interrupt (bus initialised from app_main on core 0) */
    xTaskCreatePinnedToCore(disp_task, "disp", 3072, NULL, 7, &s_disp_task, xPortGetCoreID());
    s_started = true;
    ESP_LOGI(TAG, "CO5300 %dx%d @ %d MHz QSPI (CS=%d CLK=%d D0..3=%d,%d,%d,%d RST=%d)", b->width, b->height,
             CONFIG_CAR3D_QSPI_MHZ, b->pin_cs, b->pin_clk, b->pin_d0, b->pin_d1, b->pin_d2, b->pin_d3, b->pin_rst);
    return ESP_OK;
}

void disp_selftest(void)
{
    /* 8 vertical colour bars, full brightness: proves pins, init and offsets
     * independently of the 3D renderer */
    const board_desc_t *b = g_board;
    static const uint16_t cols[8] = {0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x8410};
    const int rows = 16;
    uint16_t *blk = heap_caps_malloc((size_t)b->width * rows * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!blk) return;
    for (int x = 0; x < b->width; x++) {
        uint16_t c = cols[x * 8 / b->width];
        c = (uint16_t)((c >> 8) | (c << 8));
        for (int y = 0; y < rows; y++) blk[y * b->width + x] = c;
    }
    for (int y = 0; y < b->height; y += rows) {
        int y1 = y + rows - 1;
        if (y1 >= b->height) y1 = b->height - 1;
        disp_draw(0, y, b->width - 1, y1, blk);
    }
    disp_wait_idle();
    disp_set_brightness(0xFF);
    ESP_LOGI(TAG, "self-test: colour bars should be visible now");
    vTaskDelay(pdMS_TO_TICKS(1500));
    free(blk);
}

/* ---- executed only by the display task (or inline before it starts) ---------------- */
static void do_draw(int x0, int y0, int x1, int y1, const void *data)
{
    const board_desc_t *b = g_board;
    const int gx = b->x_gap, gy = b->y_gap;
    const int X0 = x0 + gx, X1 = x1 + gx, Y0 = y0 + gy, Y1 = y1 + gy;
    const uint8_t ca[4] = {(uint8_t)(X0 >> 8), (uint8_t)X0, (uint8_t)(X1 >> 8), (uint8_t)X1};
    const uint8_t ra[4] = {(uint8_t)(Y0 >> 8), (uint8_t)Y0, (uint8_t)(Y1 >> 8), (uint8_t)Y1};
    const size_t bytes = (size_t)(x1 - x0 + 1) * (y1 - y0 + 1) * 2;
    g_diag_disp = 2;
    tx_cmd(0x2A, ca, 4);          /* waits until the previous pixel transfer is done */
    tx_cmd(0x2B, ra, 4);
    esp_lcd_panel_io_tx_color(s_io, (int)QSPI_PIXELS, data, bytes);   /* queued (DMA) */
    g_diag_disp = 3;
}

static void do_wait_idle(void)
{
    g_diag_disp = 2;
    tx_cmd(0x00, NULL, 0);        /* NOP: returns once all queued pixels are out */
    g_diag_disp = 0;
}

static void disp_task(void *arg)
{
    disp_req_t r;
    for (;;) {
        if (xQueueReceive(s_req_q, &r, portMAX_DELAY) != pdTRUE) continue;
        switch (r.kind) {
        case REQ_DRAW:      do_draw(r.x0, r.y0, r.x1, r.y1, r.data); break;
        case REQ_WAIT_IDLE: do_wait_idle(); break;
        case REQ_CMD:       tx_cmd(r.cmd, &r.param, 1); break;
        case REQ_SLEEP:
            do_wait_idle();
            tx_cmd(0x28, NULL, 0);                  /* display off */
            vTaskDelay(pdMS_TO_TICKS(20));
            tx_cmd(0x10, NULL, 0);                  /* sleep in */
            vTaskDelay(pdMS_TO_TICKS(120));
            break;
        }
        xSemaphoreGive(r.done);
    }
}

/* one "done" semaphore per calling task (render, helper, init ...) */
#define MAX_CALLERS 6
static struct { TaskHandle_t task; SemaphoreHandle_t sem; } s_callers[MAX_CALLERS];
static portMUX_TYPE s_callers_mux = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t caller_sem(void)
{
    TaskHandle_t me = xTaskGetCurrentTaskHandle();
    for (int i = 0; i < MAX_CALLERS; i++)
        if (s_callers[i].task == me) return s_callers[i].sem;
    SemaphoreHandle_t sem = xSemaphoreCreateBinary();
    portENTER_CRITICAL(&s_callers_mux);
    for (int i = 0; i < MAX_CALLERS; i++)
        if (!s_callers[i].task) { s_callers[i].task = me; s_callers[i].sem = sem; break; }
    portEXIT_CRITICAL(&s_callers_mux);
    return sem;
}

static void submit(disp_req_t *r)
{
    r->done = caller_sem();
    xQueueSend(s_req_q, r, portMAX_DELAY);
    while (xSemaphoreTake(r->done, pdMS_TO_TICKS(1000)) != pdTRUE)
        ESP_LOGE(TAG, "display task not responding (SPI transfer stuck?)");
}

/* ---- public API ---------------------------------------------------------------------- */
void disp_draw(int x0, int y0, int x1, int y1, const void *data)
{
    if (!s_started) { do_draw(x0, y0, x1, y1, data); return; }
    disp_req_t r = {.kind = REQ_DRAW, .x0 = x0, .y0 = y0, .x1 = x1, .y1 = y1, .data = data};
    submit(&r);
}

void disp_wait_idle(void)
{
    if (!s_started) { do_wait_idle(); return; }
    disp_req_t r = {.kind = REQ_WAIT_IDLE};
    submit(&r);
}

void disp_set_brightness(uint8_t level)
{
    s_brightness = level;
    if (!s_started) { tx_cmd(0x51, &level, 1); return; }
    disp_req_t r = {.kind = REQ_CMD, .cmd = 0x51, .param = level};
    submit(&r);
}

void disp_sleep(void)
{
    if (!s_started) return;
    disp_req_t r = {.kind = REQ_SLEEP};
    submit(&r);
}

uint8_t disp_get_brightness(void) { return s_brightness; }
