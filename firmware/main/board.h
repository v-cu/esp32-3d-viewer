/*
 * board.h - Waveshare ESP32-S3-Touch-AMOLED-1.8 (V2): board description + low level
 * board services (I2C, IO expander, battery)
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

typedef struct {
    uint8_t        cmd;
    uint8_t        len;
    uint8_t        data[4];
    uint16_t       delay_ms;
} panel_init_cmd_t;

typedef struct {
    const char    *name;
    /* display */
    int            width, height;       /* native panel resolution */
    int            x_gap, y_gap;        /* RAM offsets of the visible area */
    int            pin_cs, pin_clk, pin_d0, pin_d1, pin_d2, pin_d3;
    int            pin_rst;             /* -1 = via IO expander */
    const panel_init_cmd_t *init_cmds;
    int            init_cmds_count;
    int            default_rotation;    /* 0..3 (menuconfig: Screen orientation) */
    /* i2c */
    int            pin_sda, pin_scl;
    /* touch (CST820) */
    uint8_t        touch_addr;
    bool           touch_swap_xy, touch_mirror_x, touch_mirror_y;
} board_desc_t;

extern const board_desc_t *g_board;
extern i2c_master_bus_handle_t g_i2c_bus;

void board_i2c_rebuild(void);                  /* delete + recreate the I2C bus (touch/imu detached first) */
void board_init(void);                         /* I2C, IO expander, panel/touch resets, PMU */
int  board_battery_percent(bool *charging);    /* -1 if unknown */
bool board_boot_button_pressed(void);          /* BOOT button (GPIO0), true while held */
/* panel power off, touch + panel in reset, wait for the BOOT button to be released and
 * enter deep sleep; the next BOOT press wakes the board (= normal boot). Does not return. */
void board_deep_sleep(void);
