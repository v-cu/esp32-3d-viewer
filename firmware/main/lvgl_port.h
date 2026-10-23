#pragma once
#include <stdint.h>

void     lvgl_port_init(int rotation);
void     lvgl_port_set_rotation(int rotation);
void     lvgl_port_invalidate(void);
uint32_t lvgl_port_run(void);          /* lv_timer_handler(); returns ms until next call */
void     lvgl_port_logical_size(int *w, int *h);
