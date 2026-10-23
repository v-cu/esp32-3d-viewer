/*
 * display.h - CO5300 QSPI AMOLED
 */
#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t disp_init(void);
/*
 * Send an inclusive panel rectangle (even x0/y0, odd x1/y1). `data` = big-endian RGB565.
 * Thread safe (all SPI access runs in one display task). Returns once the transfer is queued; the buffer is free again once
 * the NEXT disp_draw()/disp_wait_idle() call returns (it waits for the previous DMA).
 */
void disp_draw(int x0, int y0, int x1, int y1, const void *data);
void disp_wait_idle(void);
void disp_selftest(void);   /* colour bars at full brightness for 1.5 s */
void disp_set_brightness(uint8_t level);
uint8_t disp_get_brightness(void);
void disp_sleep(void);       /* display off + sleep in (before the panel power is cut) */
