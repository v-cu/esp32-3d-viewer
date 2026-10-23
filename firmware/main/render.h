#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "r3d.h"

bool render_alloc_buffers(void); /* band buffers in internal DMA RAM - call before loading the model */
bool render_init(void);      /* renderer (after the model is loaded) */
void render_start(void);     /* starts the render (core 1) and helper (core 0) tasks */
void *render_alloc(size_t n, r3d_mem_t kind);

/* band buffers are reused by LVGL while the settings screen is shown */
uint16_t *render_band_buffer(int worker, int idx);
size_t render_band_buffer_bytes(void);
