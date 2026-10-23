/*
 * board.c - Waveshare ESP32-S3-Touch-AMOLED-1.8 (V2)
 *
 * Source: official Waveshare demo repository (ESP32-S3-Touch-AMOLED-1.8) and docs:
 *   V2 board: CO5300 AMOLED 368x448 over QSPI (X offset 16), CST820 touch @0x15,
 *   TCA9554 IO expander @0x20 (LCD reset, AMOLED power, touch reset), AXP2101 PMU @0x34,
 *   QMI8658 IMU, 16 MB flash, 8 MB octal PSRAM.
 *   (V1 had SH8601 + FT3168 and is not supported.)
 */
#include <string.h>
#include "board.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#if !CONFIG_IDF_TARGET_ESP32S3
#error "esp32-3d-viewer targets the ESP32-S3: run  idf.py set-target esp32s3  (VS Code: 'Set Espressif Device Target' -> esp32s3)"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_rom_sys.h"

static const char *TAG = "board";

#define BOOT_BUTTON_GPIO GPIO_NUM_0

i2c_master_bus_handle_t g_i2c_bus;

/* ------------------------------------------------------------------------------------ */
static const panel_init_cmd_t s_init[] = {
    {0xFE, 1, {0x00}, 0},                /* user command page */
    {0xC4, 1, {0x80}, 0},                /* SPI mode control (QSPI) */
    {0x3A, 1, {0x55}, 0},                /* RGB565 */
    {0x36, 1, {0x00}, 0},                /* MADCTL */
    {0x35, 1, {0x00}, 0},                /* tearing effect line on */
    {0x53, 1, {0x20}, 0},                /* brightness control on */
    {0x63, 1, {0xFF}, 0},                /* HBM brightness */
    {0x51, 1, {0x00}, 0},                /* brightness 0 until the first frame */
    {0x11, 0, {0}, 120},                 /* sleep out */
    {0x29, 0, {0}, 10},                  /* display on */
};

static const board_desc_t s_board = {
    .name = "Waveshare ESP32-S3-Touch-AMOLED-1.8 V2",
    .width = 368, .height = 448, .x_gap = 16, .y_gap = 0,
    .pin_cs = 12, .pin_clk = 11,
    .pin_d0 = 4, .pin_d1 = 5, .pin_d2 = 6, .pin_d3 = 7,
    .pin_rst = -1,                       /* TCA9554 P0 */
    .init_cmds = s_init, .init_cmds_count = sizeof(s_init) / sizeof(s_init[0]),
    .default_rotation = CONFIG_CAR3D_ROTATION,
    .pin_sda = 15, .pin_scl = 14,
    .touch_addr = 0x15,
};

const board_desc_t *g_board = &s_board;

/* ------------------------------------------------------------------------------------ */
#define TCA9554_ADDR     0x20
#define TCA_REG_OUTPUT   0x01
#define TCA_REG_CONFIG   0x03
#define TCA_LCD_RST      (1u << 0)
#define TCA_AMOLED_PWR   (1u << 1)
#define TCA_TOUCH_RST    (1u << 2)
#define TCA_SD_CS        (1u << 7)
#define AXP2101_ADDR     0x34

static i2c_master_dev_handle_t s_tca, s_axp;

static esp_err_t reg_write(i2c_master_dev_handle_t d, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_master_transmit(d, b, 2, 50);
}

static esp_err_t reg_read(i2c_master_dev_handle_t d, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(d, &reg, 1, val, 1, 50);
}

static i2c_master_dev_handle_t add_dev(uint8_t addr)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t h = NULL;
    if (i2c_master_bus_add_device(g_i2c_bus, &cfg, &h) != ESP_OK) return NULL;
    return h;
}

static esp_err_t i2c_bus_create(void)
{
    const board_desc_t *b = g_board;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = b->pin_sda,
        .scl_io_num = b->pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &g_i2c_bus);
}

void board_i2c_rebuild(void)
{
    const bool had_tca = s_tca != NULL, had_axp = s_axp != NULL;
    if (s_tca) i2c_master_bus_rm_device(s_tca);
    if (s_axp) i2c_master_bus_rm_device(s_axp);
    s_tca = s_axp = NULL;
    if (i2c_del_master_bus(g_i2c_bus) != ESP_OK) ESP_LOGW(TAG, "i2c bus delete failed");
    g_i2c_bus = NULL;
    if (i2c_bus_create() != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus re-create failed");
        return;
    }
    if (had_tca) s_tca = add_dev(TCA9554_ADDR);
    if (had_axp) s_axp = add_dev(AXP2101_ADDR);
}

/*
 * I2C bus recovery: a chip that was in the middle of a read when the ESP reset keeps SDA
 * low and the ESP-IDF driver then waits forever for a free bus. Clock SCL until the chip
 * lets SDA go (max 16 pulses), then send a STOP. Returns false if the bus stays stuck.
 */
static bool i2c_bus_unstick(void)
{
    const board_desc_t *b = g_board;
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << b->pin_sda) | (1ULL << b->pin_scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    gpio_set_level(b->pin_sda, 1);
    gpio_set_level(b->pin_scl, 1);
    esp_rom_delay_us(10);
    int pulses = 0;
    while (gpio_get_level(b->pin_sda) == 0 && pulses < 16) {
        gpio_set_level(b->pin_scl, 0);
        esp_rom_delay_us(10);
        gpio_set_level(b->pin_scl, 1);
        esp_rom_delay_us(10);
        pulses++;
    }
    /* STOP: SDA low -> high while SCL is high */
    gpio_set_level(b->pin_sda, 0);
    esp_rom_delay_us(10);
    gpio_set_level(b->pin_scl, 1);
    esp_rom_delay_us(10);
    gpio_set_level(b->pin_sda, 1);
    esp_rom_delay_us(10);
    const bool ok = gpio_get_level(b->pin_sda) && gpio_get_level(b->pin_scl);
    if (pulses) ESP_LOGW(TAG, "I2C bus was held low, recovered with %d clock pulses", pulses);
    gpio_reset_pin(b->pin_sda);
    gpio_reset_pin(b->pin_scl);
    return ok;
}

void board_init(void)
{
    const board_desc_t *b = g_board;
    if (!i2c_bus_unstick()) {
        ESP_LOGE(TAG, "I2C bus stuck low (SDA=%d SCL=%d): display, touch and IMU are not "
                 "reachable. Power the board off completely (USB + battery, or hold PWR ~10 s).",
                 gpio_get_level(b->pin_sda), gpio_get_level(b->pin_scl));
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));      /* do not hang inside the I2C driver */
    }
    ESP_ERROR_CHECK(i2c_bus_create());

    s_tca = add_dev(TCA9554_ADDR);
    if (s_tca) {
        const uint8_t outs = TCA_LCD_RST | TCA_AMOLED_PWR | TCA_TOUCH_RST | TCA_SD_CS;
        reg_write(s_tca, TCA_REG_CONFIG, (uint8_t)~outs);
        reg_write(s_tca, TCA_REG_OUTPUT, TCA_SD_CS);                  /* resets active */
        vTaskDelay(pdMS_TO_TICKS(20));
        reg_write(s_tca, TCA_REG_OUTPUT, outs);                       /* release */
        vTaskDelay(pdMS_TO_TICKS(150));
    } else {
        ESP_LOGW(TAG, "TCA9554 not found (display/touch reset not possible)");
    }
    gpio_config_t btn = {.pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&btn);
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) ESP_LOGI(TAG, "woke up from deep sleep (BOOT)");
    s_axp = add_dev(AXP2101_ADDR);
    if (s_axp) {
        /* enable battery voltage / percentage measurement (ADC channel enable) */
        uint8_t v = 0;
        if (reg_read(s_axp, 0x30, &v) == ESP_OK) reg_write(s_axp, 0x30, v | 0x01);
    }
    ESP_LOGI(TAG, "%s: %dx%d", b->name, b->width, b->height);
}

int board_battery_percent(bool *charging)
{
    *charging = false;
    if (!s_axp) return -1;
    uint8_t st = 0, pct = 0, st0 = 0;
    if (reg_read(s_axp, 0x00, &st0) != ESP_OK) return -1;
    if (!(st0 & 0x08)) {                         /* no battery present */
        *charging = true;
        return -1;
    }
    if (reg_read(s_axp, 0x01, &st) == ESP_OK) *charging = ((st >> 5) & 0x03) == 0x01;
    if (reg_read(s_axp, 0xA4, &pct) != ESP_OK) return -1;
    return pct > 100 ? 100 : pct;
}

bool board_boot_button_pressed(void) { return gpio_get_level(BOOT_BUTTON_GPIO) == 0; }

void board_deep_sleep(void)
{
    /* Do NOT cut AMOLED_PWR or hold resets here: the touch controller hangs on that supply
     * and an unpowered chip clamps the shared I2C bus low, so the ESP could not reach the
     * TCA9554 after waking up. The panel is in sleep-in mode and the touch controller in
     * its own deep sleep instead (both draw only microamps). */
    /* sleep only after the button is released, otherwise it wakes up again at once */
    int released_ms = 0;
    while (released_ms < 60) {
        vTaskDelay(pdMS_TO_TICKS(10));
        released_ms = board_boot_button_pressed() ? 0 : released_ms + 10;
    }
    ESP_LOGI(TAG, "deep sleep - press BOOT to wake up");
    vTaskDelay(pdMS_TO_TICKS(20));                     /* let the log line out */
    rtc_gpio_pullup_en(BOOT_BUTTON_GPIO);
    rtc_gpio_pulldown_dis(BOOT_BUTTON_GPIO);
    esp_sleep_enable_ext0_wakeup(BOOT_BUTTON_GPIO, 0);
    esp_deep_sleep_start();
}
