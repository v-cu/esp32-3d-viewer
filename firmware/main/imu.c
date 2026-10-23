/*
 * imu.c - QMI8658 6-axis IMU + Mahony AHRS (orientation as quaternion)
 */
#include <math.h>
#include <string.h>
#include "imu.h"
#include "board.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu";

#define REG_WHO_AM_I 0x00
#define REG_CTRL1    0x02
#define REG_CTRL2    0x03
#define REG_CTRL3    0x04
#define REG_CTRL5    0x06
#define REG_CTRL7    0x08
#define REG_AX_L     0x35

#define ACC_LSB_PER_G   8192.0f          /* +-4 g    */
#define GYR_LSB_PER_DPS 64.0f            /* +-512 dps */

static i2c_master_dev_handle_t s_dev;
static uint8_t s_addr;                       /* 0 = not found */
static esp_err_t s_last_err = ESP_OK;
static int s_axis_idx[3] = {0, 1, 2};
static float s_axis_sgn[3] = {1, 1, 1};
static float s_bias[3];

static esp_err_t wr(uint8_t reg, uint8_t v)
{
    uint8_t b[2] = {reg, v};
    return i2c_master_transmit(s_dev, b, 2, 20);
}

static void configure(void)
{
    wr(REG_CTRL1, 0x40);                 /* address auto increment, little endian */
    wr(REG_CTRL2, 0x15);                 /* accel +-4 g, ~235 Hz */
    wr(REG_CTRL3, 0x55);                 /* gyro +-512 dps, ~235 Hz */
    wr(REG_CTRL5, 0x11);                 /* low-pass filters on */
    wr(REG_CTRL7, 0x03);                 /* enable accel + gyro */
}

static void parse_axes(const char *s)
{
    /* "+x+y+z" style: which signed IMU axis is panel right / up / out-of-screen */
    for (int i = 0; i < 3 && s[0] && s[1]; i++, s += 2) {
        s_axis_sgn[i] = (s[0] == '-') ? -1.0f : 1.0f;
        s_axis_idx[i] = (s[1] == 'x' || s[1] == 'X') ? 0 : (s[1] == 'y' || s[1] == 'Y') ? 1 : 2;
    }
}

bool imu_init(void)
{
    static const uint8_t addrs[2] = {0x6B, 0x6A};
    for (int i = 0; i < 2 && !s_dev; i++) {
        i2c_device_config_t cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addrs[i],
                                   .scl_speed_hz = 400000};
        i2c_master_dev_handle_t d;
        if (i2c_master_bus_add_device(g_i2c_bus, &cfg, &d) != ESP_OK) continue;
        uint8_t reg = REG_WHO_AM_I, id = 0;
        if (i2c_master_transmit_receive(d, &reg, 1, &id, 1, 20) == ESP_OK && id == 0x05) {
            s_dev = d;
            s_addr = addrs[i];
            ESP_LOGI(TAG, "QMI8658 at 0x%02X", addrs[i]);
        } else {
            i2c_master_bus_rm_device(d);
        }
    }
    if (!s_dev) {
        ESP_LOGW(TAG, "QMI8658 not found - motion mode disabled");
        return false;
    }
    configure();
    parse_axes(CONFIG_CAR3D_IMU_AXES);
    vTaskDelay(pdMS_TO_TICKS(30));
    return true;
}

bool imu_read(float acc[3], float gyr[3])
{
    if (!s_dev) return false;
    uint8_t reg = REG_AX_L, b[12];
    s_last_err = i2c_master_transmit_receive(s_dev, &reg, 1, b, 12, 30);
    if (s_last_err != ESP_OK) return false;
    float a[3], g[3];
    for (int i = 0; i < 3; i++) {
        a[i] = (int16_t)(b[i * 2] | (b[i * 2 + 1] << 8)) / ACC_LSB_PER_G;
        g[i] = (int16_t)(b[6 + i * 2] | (b[7 + i * 2] << 8)) / GYR_LSB_PER_DPS * 0.01745329f;
    }
    for (int i = 0; i < 3; i++) {
        acc[i] = s_axis_sgn[i] * a[s_axis_idx[i]];
        gyr[i] = s_axis_sgn[i] * g[s_axis_idx[i]] - s_bias[i];
    }
    return true;
}

void imu_calibrate(int samples)
{
    if (!s_dev) return;
    float sum[3] = {0}, a[3], g[3];
    float keep[3];
    memcpy(keep, s_bias, sizeof(keep));
    memset(s_bias, 0, sizeof(s_bias));
    int n = 0;
    for (int i = 0; i < samples; i++) {
        if (imu_read(a, g)) {
            if (fabsf(g[0]) + fabsf(g[1]) + fabsf(g[2]) > 0.3f) {   /* moving: abort */
                memcpy(s_bias, keep, sizeof(keep));
                return;
            }
            for (int k = 0; k < 3; k++) sum[k] += g[k];
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (n > samples / 2)
        for (int k = 0; k < 3; k++) s_bias[k] = sum[k] / n;
    else
        memcpy(s_bias, keep, sizeof(keep));
    ESP_LOGI(TAG, "gyro bias %.4f %.4f %.4f rad/s", s_bias[0], s_bias[1], s_bias[2]);
}

/* ---------------------------------- Mahony AHRS ------------------------------------- */
static r3d_quat_t s_q = {1, 0, 0, 0};
static float s_ix, s_iy, s_iz;
static bool s_init;

void ahrs_reset(void) { s_init = false; s_ix = s_iy = s_iz = 0; }

void ahrs_update(const float a_in[3], const float g_in[3], float dt)
{
    float ax = a_in[0], ay = a_in[1], az = a_in[2];
    float gx = g_in[0], gy = g_in[1], gz = g_in[2];
    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (!s_init) {
        if (an < 0.3f) return;
        /* align "up" with the measured gravity reaction (accelerometer) */
        ax /= an; ay /= an; az /= an;
        float w = sqrtf(fmaxf(0.0f, (1.0f + az) * 0.5f));
        if (w > 1e-3f) {
            r3d_quat_t q = {w, ay / (2 * w), -ax / (2 * w), 0};   /* rotates measured up -> world z */
            s_q = r3d_quat_normalize(q);
        } else {
            s_q = (r3d_quat_t){0, 1, 0, 0};
        }
        s_init = true;
        return;
    }
    float q0 = s_q.w, q1 = s_q.x, q2 = s_q.y, q3 = s_q.z;
    /* trust the accelerometer less while the device is being shaken */
    float kp = (an > 0.85f && an < 1.15f) ? 1.2f : 0.15f;
    const float ki = 0.03f;
    if (an > 0.1f) {
        ax /= an; ay /= an; az /= an;
        float vx = 2 * (q1 * q3 - q0 * q2);
        float vy = 2 * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        float ex = ay * vz - az * vy, ey = az * vx - ax * vz, ez = ax * vy - ay * vx;
        s_ix += ki * ex * dt; s_iy += ki * ey * dt; s_iz += ki * ez * dt;
        gx += kp * ex + s_ix; gy += kp * ey + s_iy; gz += kp * ez + s_iz;
    }
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 += qa * gx + qc * gz - q3 * gy;
    q2 += qa * gy - qb * gz + q3 * gx;
    q3 += qa * gz + qb * gy - qc * gx;
    s_q = r3d_quat_normalize((r3d_quat_t){q0, q1, q2, q3});
}

esp_err_t imu_last_error(void) { return s_dev ? s_last_err : ESP_OK; }

void imu_bus_detach(void)
{
    if (s_dev) i2c_master_bus_rm_device(s_dev);
    s_dev = NULL;
}

void imu_bus_attach(void)
{
    if (!s_addr || s_dev) return;
    i2c_device_config_t cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = s_addr,
                               .scl_speed_hz = 400000};
    if (i2c_master_bus_add_device(g_i2c_bus, &cfg, &s_dev) != ESP_OK) { s_dev = NULL; return; }
    s_last_err = ESP_OK;
    configure();
}

bool ahrs_ready(void) { return s_init; }
r3d_quat_t ahrs_quat(void) { return s_q; }

void imu_power_down(void)
{
    if (!s_dev) return;
    wr(REG_CTRL7, 0x00);                             /* accel + gyro off */
    wr(REG_CTRL1, 0x41);                             /* + internal oscillator off */
}
