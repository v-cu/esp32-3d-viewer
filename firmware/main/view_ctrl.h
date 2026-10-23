/*
 * view_ctrl.h - turns touch gestures and board tilt (IMU) into the camera/model view.
 * Runs at a fixed rate in the input task; the render task samples the result.
 */
#pragma once
#include <stdbool.h>
#include "r3d_math.h"
#include "touch.h"

typedef struct {
    r3d_quat_t orient;
    float      zoom;
    uint32_t   seq;          /* increments whenever the view changes */
} view_state_t;

void view_ctrl_init(void);
/* touch: panel coordinates; acc [g] / gyr [rad/s]: IMU in the panel frame (NULL if no IMU);
 * dt seconds */
void view_ctrl_update(const touch_data_t *t, const float *acc, const float *gyr, float dt);
void view_ctrl_get(view_state_t *out);
void view_ctrl_recenter(void);       /* motion mode: current pose becomes the reference */
void view_ctrl_reset(void);          /* animate back to the default 3/4 view */
void view_ctrl_set_enabled(bool on); /* false while the settings screen is open */

/* last touch in LOGICAL screen coordinates (for LVGL) */
bool view_ctrl_last_touch(int *lx, int *ly);
