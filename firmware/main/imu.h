/*
 * imu.h - QMI8658 + Mahony fusion. Vectors are in the PANEL frame:
 *         x = native panel right, y = native panel up, z = out of the screen.
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "r3d_math.h"

bool imu_init(void);
bool imu_read(float acc_g[3], float gyro_rads[3]);
esp_err_t imu_last_error(void);               /* result of the last I2C read (ESP_OK when no IMU) */
void imu_bus_detach(void);                    /* before the I2C bus is rebuilt */
void imu_bus_attach(void);                    /* after the I2C bus is rebuilt (re-configures the chip) */
void imu_calibrate(int samples);              /* gyro bias, device must be still */
void imu_power_down(void);                    /* sensors + oscillator off (before deep sleep) */

void ahrs_reset(void);
void ahrs_update(const float acc[3], const float gyr[3], float dt);
bool ahrs_ready(void);
r3d_quat_t ahrs_quat(void);                   /* body (panel) -> world */
