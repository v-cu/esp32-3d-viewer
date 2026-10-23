#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    int n;          /* number of touch points (0..1, CST820 is single touch) */
    int x[2], y[2]; /* native panel coordinates */
} touch_data_t;

bool touch_init(void);
bool touch_read(touch_data_t *t);
esp_err_t touch_last_error(void);             /* result of the last I2C read (ESP_OK when no controller) */
void touch_bus_detach(void);                  /* before the I2C bus is rebuilt */
void touch_bus_attach(void);                  /* after the I2C bus is rebuilt */
void touch_sleep(void);                       /* CST820 deep sleep (before deep sleep of the ESP) */
