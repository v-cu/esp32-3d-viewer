/*
 * view_ctrl.c - gestures (drag with inertia, double tap = reset, triple tap = next
 *               model, long press = settings), idle auto-spin and Motion mode.
 *
 * Motion mode = steering wheel: hold the board upright and tilt it left / right (rotation
 * about the axis that comes out of the screen) to turn the model left / right. The angle
 * is the direction of gravity in the screen plane (accelerometer: absolute, no drift)
 * fused with the gyro about the screen normal (smooth, no lag). In "Free" rotation mode
 * tilting the top of the board towards / away from you also changes the view height.
 * Double tap makes the current pose the neutral one.
 */
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "view_ctrl.h"
#include "app.h"
#include "board.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#define DEG2RAD           0.01745329f
#define DEFAULT_YAW       215.0f        /* front 3/4 view */
#define DEFAULT_PITCH     16.0f
#define PITCH_MIN         -12.0f
#define PITCH_MAX         82.0f
#define ZOOM_MIN          0.6f
#define ZOOM_MAX          2.2f
#define DRAG_DEG_PER_PX_X 0.50f
#define DRAG_DEG_PER_PX_Y 0.38f
#define INERTIA_TAU_S     0.9f
#define TAP_MAX_MS        250
#define MULTI_TAP_MS      350           /* max gap between taps of a double / triple tap */
#define LONG_PRESS_MS     700
#define DRAG_START_PX     9
#define AUTOSPIN_IDLE_S   6.0f
#define AUTOSPIN_DEG_S    24.0f
#define MOTION_YAW_GAIN   2.5f          /* model degrees per degree of left/right tilt */
#define MOTION_PITCH_GAIN 0.8f          /* view height per degree of forward/back tilt (Free) */
#define MOTION_PITCH_MAX  45.0f
#define MOTION_ACC_MIN    0.30f         /* g in the screen plane needed for an absolute angle */
#define MOTION_TAU_S      0.04f

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static view_state_t s_out;
static bool s_enabled = true;

/* touch orbit */
static float s_yaw = DEFAULT_YAW, s_pitch = DEFAULT_PITCH, s_zoom = 1.0f;
static float s_vyaw, s_vpitch;
static bool  s_anim;
/* gesture */
static bool  s_down, s_drag, s_long_fired, s_pinch;
static int   s_sx, s_sy, s_lx, s_ly;
static int64_t s_t_down, s_t_last_tap, s_t_last_input;
static int   s_taps;
static float s_pinch_d;
static float s_ema_vx, s_ema_vy;
/* motion */
static bool  s_want_recenter = true, s_have_ref;
static float s_roll;                            /* tilt about the screen normal [deg], wrapped */
static float s_roll_rel;                        /* continuous (unwrapped) tilt since recenter */
static float s_elev, s_elev_ref;                /* tilt towards / away from the user [deg] */
static float s_m_yaw, s_m_pitch;                /* smoothed motion offsets added to the view */
/* last touch for LVGL */
static bool s_lt_down, s_lt_latch;             /* latch: a press LVGL has not seen yet */
static int  s_lt_x, s_lt_y;

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static void panel_to_logical(int px, int py, int *lx, int *ly)
{
    const int W = g_board->width, H = g_board->height;
    switch (g_settings.rotation & 3) {
    default: *lx = px; *ly = py; break;
    case 1: *lx = py; *ly = W - 1 - px; break;
    case 2: *lx = W - 1 - px; *ly = H - 1 - py; break;
    case 3: *lx = H - 1 - py; *ly = px; break;
    }
}

void view_ctrl_init(void)
{
    s_t_last_input = now_ms();
    s_out.orient = r3d_quat_identity();
    s_out.zoom = 1.0f;
}

void view_ctrl_set_enabled(bool on)
{
    s_enabled = on;
    s_down = s_drag = s_pinch = false;
    s_t_last_input = now_ms();
}

void view_ctrl_recenter(void) { s_want_recenter = true; }

void view_ctrl_reset(void)
{
    s_anim = true;
    s_vyaw = s_vpitch = 0;
    s_want_recenter = true;
}

static float wrap360(float a)
{
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f) a += 360.0f;
    return a;
}

static void handle_touch(const touch_data_t *t, float dt)
{
    int64_t now = now_ms();
    int lx = 0, ly = 0;
    if (t->n > 0) panel_to_logical(t->x[0], t->y[0], &lx, &ly);

    portENTER_CRITICAL(&s_mux);
    s_lt_down = t->n > 0;
    if (t->n > 0) {
        s_lt_x = lx; s_lt_y = ly;
        s_lt_latch = true;
    }
    portEXIT_CRITICAL(&s_mux);

    if (!s_enabled) return;

    if (t->n >= 2) {                          /* pinch zoom */
        int ax, ay, bx, by;
        panel_to_logical(t->x[0], t->y[0], &ax, &ay);
        panel_to_logical(t->x[1], t->y[1], &bx, &by);
        float d = sqrtf((float)((ax - bx) * (ax - bx) + (ay - by) * (ay - by)));
        if (!s_pinch) { s_pinch = true; s_pinch_d = d; }
        else if (s_pinch_d > 10.0f && d > 10.0f) {
            s_zoom *= d / s_pinch_d;
            if (s_zoom < ZOOM_MIN) s_zoom = ZOOM_MIN;
            if (s_zoom > ZOOM_MAX) s_zoom = ZOOM_MAX;
            s_pinch_d = d;
        }
        s_drag = true;                        /* no tap / long press after a pinch */
        s_long_fired = true;
        s_vyaw = s_vpitch = 0;
        s_t_last_input = now;
        return;
    }
    s_pinch = false;

    if (t->n == 1) {
        if (!s_down) {
            s_down = true; s_drag = false; s_long_fired = false;
            s_sx = s_lx = lx; s_sy = s_ly = ly;
            s_t_down = now;
            s_vyaw = s_vpitch = 0;
            s_ema_vx = s_ema_vy = 0;
            s_anim = false;
        } else {
            int dx = lx - s_lx, dy = ly - s_ly;
            if (!s_drag && (abs(lx - s_sx) > DRAG_START_PX || abs(ly - s_sy) > DRAG_START_PX)) s_drag = true;
            if (s_drag) {
                s_taps = 0;
                s_yaw = wrap360(s_yaw + dx * DRAG_DEG_PER_PX_X);
                if (g_settings.rot_mode == ROT_YAW_ONLY) dy = 0;
                s_pitch += dy * DRAG_DEG_PER_PX_Y;
                if (s_pitch < PITCH_MIN) s_pitch = PITCH_MIN;
                if (s_pitch > PITCH_MAX) s_pitch = PITCH_MAX;
                if (dt > 0) {
                    s_ema_vx = s_ema_vx * 0.6f + 0.4f * (dx / dt);
                    s_ema_vy = s_ema_vy * 0.6f + 0.4f * (dy / dt);
                }
            }
            s_lx = lx; s_ly = ly;
            if (!s_drag && !s_long_fired && now - s_t_down > LONG_PRESS_MS) {
                s_long_fired = true;
                app_request_screen(SCREEN_SETTINGS);
            }
        }
        s_t_last_input = now;
        return;
    }

    /* released */
    if (s_down) {
        s_down = false;
        if (s_drag) {
            s_vyaw = s_ema_vx * DRAG_DEG_PER_PX_X;
            s_vpitch = s_ema_vy * DRAG_DEG_PER_PX_Y;
            const float vmax = 720.0f;
            if (s_vyaw > vmax) s_vyaw = vmax;
            if (s_vyaw < -vmax) s_vyaw = -vmax;
            if (s_vpitch > vmax) s_vpitch = vmax;
            if (s_vpitch < -vmax) s_vpitch = -vmax;
        } else if (!s_long_fired && now - s_t_down < TAP_MAX_MS) {
            /* count taps; the action fires once no further tap follows (see update) */
            s_taps = (now - s_t_last_tap < MULTI_TAP_MS) ? s_taps + 1 : 1;
            s_t_last_tap = now;
        }
        s_t_last_input = now;
    }
}

static float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* acc / gyr in the PANEL frame (x right, y up, z out of the screen), g and rad/s */
static void motion_update(const float *acc, const float *gyr, float dt)
{
    const float RAD2DEG = 57.2957795f;
    const float m = sqrtf(acc[0] * acc[0] + acc[1] * acc[1]);
    /* the board turns by +phi about +z -> "up" turns by -phi in board coordinates */
    const float roll_acc = -atan2f(acc[1], acc[0]) * RAD2DEG;
    const float elev_acc = atan2f(acc[2], m) * RAD2DEG;
    /* the neutral pose is only taken while the board is roughly upright: lying flat,
     * gravity has no direction in the screen plane and the angle would be noise */
    if (s_want_recenter || !s_have_ref) {
        if (m < 0.5f) {
            s_m_yaw *= 0.9f;                         /* wait, ease back to the touch view */
            s_m_pitch *= 0.9f;
            return;
        }
        s_roll = roll_acc;
        s_elev = elev_acc;
        s_elev_ref = elev_acc;
        s_roll_rel = 0.0f;
        s_have_ref = true;
        s_want_recenter = false;
    }
    /* gyro for smoothness, accelerometer pulls the angle back (no drift). The change is
     * accumulated without wrapping, so turning the board past 180 deg never makes the
     * model jump (a wrap would be 360 deg * gain). */
    const float prev = s_roll;
    float roll = s_roll + gyr[2] * RAD2DEG * dt;
    if (m > MOTION_ACC_MIN) roll += wrap180(roll_acc - roll) * (1.0f - expf(-dt / 0.25f));
    s_roll = wrap180(roll);
    s_roll_rel += wrap180(s_roll - prev);
    s_elev += (elev_acc - s_elev) * (1.0f - expf(-dt / 0.08f));

    float yaw = s_roll_rel * MOTION_YAW_GAIN;
#if CONFIG_CAR3D_MOTION_INVERT
    yaw = -yaw;
#endif
    float pitch = 0.0f;
    if (g_settings.rot_mode == ROT_FREE) {
        pitch = (s_elev - s_elev_ref) * MOTION_PITCH_GAIN;
        if (pitch > MOTION_PITCH_MAX) pitch = MOTION_PITCH_MAX;
        if (pitch < -MOTION_PITCH_MAX) pitch = -MOTION_PITCH_MAX;
    }
    const float k = 1.0f - expf(-dt / MOTION_TAU_S);
    const float dyaw = (yaw - s_m_yaw) * k;
    s_m_yaw += dyaw;
    s_m_pitch += (pitch - s_m_pitch) * k;
    if (fabsf(dyaw) > 0.3f) s_t_last_input = now_ms();       /* tilting = activity: no auto spin */
}

void view_ctrl_update(const touch_data_t *t, const float *acc, const float *gyr, float dt)
{
    handle_touch(t, dt);
    const int64_t now = now_ms();

    if (s_taps && !s_down && now - s_t_last_tap > MULTI_TAP_MS) {
        if (s_taps == 2) view_ctrl_reset();
        else if (s_taps >= 3) app_request_next_model();
        s_taps = 0;
    }
    if (g_settings.rot_mode == ROT_YAW_ONLY) {     /* left/right only: pitch stays fixed */
        s_vpitch = 0;
        if (!s_anim) s_pitch += (DEFAULT_PITCH - s_pitch) * (1.0f - expf(-dt / 0.15f));
    }

    if (!s_down) {
        /* inertia */
        s_yaw = wrap360(s_yaw + s_vyaw * dt);
        s_pitch += s_vpitch * dt;
        if (s_pitch < PITCH_MIN) { s_pitch = PITCH_MIN; s_vpitch = 0; }
        if (s_pitch > PITCH_MAX) { s_pitch = PITCH_MAX; s_vpitch = 0; }
        float k = expf(-dt / INERTIA_TAU_S);
        s_vpitch *= k;
        bool spin = s_enabled && g_settings.auto_spin && !s_anim &&
                    (now - s_t_last_input) > (int64_t)(AUTOSPIN_IDLE_S * 1000);
        if (spin) s_vyaw += (AUTOSPIN_DEG_S - s_vyaw) * (1.0f - expf(-dt / 1.5f));   /* ease in */
        else s_vyaw *= k;
        if (fabsf(s_vyaw) < 0.05f) s_vyaw = 0;
        if (fabsf(s_vpitch) < 0.05f) s_vpitch = 0;
        if (s_anim) {                         /* smooth reset to the default view */
            float a = 1.0f - expf(-dt / 0.18f);
            float dy = DEFAULT_YAW - s_yaw;
            while (dy > 180) dy -= 360;
            while (dy < -180) dy += 360;
            s_yaw = wrap360(s_yaw + dy * a);
            s_pitch += (DEFAULT_PITCH - s_pitch) * a;
            s_zoom += (1.0f - s_zoom) * a;
            if (fabsf(dy) < 0.05f && fabsf(DEFAULT_PITCH - s_pitch) < 0.05f && fabsf(1.0f - s_zoom) < 0.002f)
                s_anim = false;
        }
    }

    float yaw = s_yaw, pitch = s_pitch;
    if (g_settings.mode == CTRL_MOTION && acc && gyr && s_enabled) {
        motion_update(acc, gyr, dt);
        yaw += s_m_yaw;
        pitch += s_m_pitch;
        if (pitch < PITCH_MIN) pitch = PITCH_MIN;
        if (pitch > PITCH_MAX) pitch = PITCH_MAX;
    } else {
        s_have_ref = false;
        s_m_yaw = s_m_pitch = 0.0f;
    }
    r3d_quat_t qy = r3d_quat_axis_angle(0, 1, 0, yaw * DEG2RAD);
    r3d_quat_t qx = r3d_quat_axis_angle(1, 0, 0, pitch * DEG2RAD);
    r3d_quat_t orient = r3d_quat_mul(qx, qy);

    portENTER_CRITICAL(&s_mux);
    const r3d_quat_t o = s_out.orient;
    float d = fabsf(o.w - orient.w) + fabsf(o.x - orient.x) + fabsf(o.y - orient.y) + fabsf(o.z - orient.z);
    if (d > 2e-5f || fabsf(s_out.zoom - s_zoom) > 1e-4f) {
        s_out.orient = orient;
        s_out.zoom = s_zoom;
        s_out.seq++;
    }
    portEXIT_CRITICAL(&s_mux);
}

void view_ctrl_get(view_state_t *out)
{
    portENTER_CRITICAL(&s_mux);
    *out = s_out;
    portEXIT_CRITICAL(&s_mux);
}

bool view_ctrl_last_touch(int *lx, int *ly)
{
    portENTER_CRITICAL(&s_mux);
    /* a short tap may start and end between two LVGL reads: report it as pressed once */
    bool d = s_lt_down || s_lt_latch;
    s_lt_latch = false;
    *lx = s_lt_x; *ly = s_lt_y;
    portEXIT_CRITICAL(&s_mux);
    return d;
}
