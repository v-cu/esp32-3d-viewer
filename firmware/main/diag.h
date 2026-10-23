/*
 * diag.h - lightweight stall detector
 *
 * Every task bumps a heartbeat counter and records which step it is in. A small
 * monitor task checks the counters twice a second; when one stops moving it logs
 * the task and its last step, so a freeze shows up in the log with its location.
 */
#pragma once
#include <stdint.h>

typedef enum { DIAG_INPUT = 0, DIAG_RENDER, DIAG_HELPER, DIAG_NUM } diag_task_t;

typedef struct {
    volatile uint32_t hb;        /* incremented once per loop iteration */
    volatile uint8_t  stage;     /* last step entered (see stage names in diag.c) */
    volatile uint8_t  waiting;   /* 1 = blocked on purpose (helper waiting for a job) */
} diag_slot_t;

extern diag_slot_t g_diag[DIAG_NUM];
extern volatile uint8_t g_diag_disp;          /* display: 0 idle, 1 lock, 2 window cmd, 3 pixels queued */
extern volatile uint32_t g_diag_i2c_fail, g_diag_i2c_recover;
extern volatile int32_t g_diag_i2c_last_err;

#define DIAG_STAGE(t, s) (g_diag[(t)].stage = (uint8_t)(s))
#define DIAG_BEAT(t)     (g_diag[(t)].hb++)

/* input task steps */
enum { DI_DELAY = 1, DI_IMU, DI_AHRS, DI_TOUCH, DI_VIEW, DI_BATTERY, DI_I2C_RECOVER };
/* render task steps */
enum { DR_TOP = 1, DR_MODEL_SWITCH, DR_ENTER_SCREEN, DR_LVGL, DR_IDLE, DR_TRANSFORM, DR_BIN, DR_BANDS,
       DR_FIRST_FRAME, DR_FRAME_DELAY };
/* helper task steps */
enum { DH_WAIT = 1, DH_TRANSFORM, DH_BIN, DH_BANDS, DH_NOTIFY };

void diag_start(void);
