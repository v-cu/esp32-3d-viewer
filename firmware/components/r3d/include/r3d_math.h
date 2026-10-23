/*
 * r3d_math.h - tiny vector / quaternion helpers (float, header-only)
 */
#pragma once
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float x, y, z; } r3d_vec3_t;
typedef struct { float w, x, y, z; } r3d_quat_t;      /* unit quaternion, w first */
typedef struct { float m[3][3]; } r3d_mat3_t;

static inline r3d_quat_t r3d_quat_identity(void) { r3d_quat_t q = {1, 0, 0, 0}; return q; }

static inline r3d_quat_t r3d_quat_mul(r3d_quat_t a, r3d_quat_t b)
{
    r3d_quat_t r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

static inline r3d_quat_t r3d_quat_conj(r3d_quat_t q) { r3d_quat_t r = {q.w, -q.x, -q.y, -q.z}; return r; }

static inline r3d_quat_t r3d_quat_normalize(r3d_quat_t q)
{
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12f) return r3d_quat_identity();
    float inv = 1.0f / n;
    r3d_quat_t r = {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
    return r;
}

/* rotation of `angle` radians around unit axis (ax, ay, az) */
static inline r3d_quat_t r3d_quat_axis_angle(float ax, float ay, float az, float angle)
{
    float s = sinf(angle * 0.5f);
    r3d_quat_t q = {cosf(angle * 0.5f), ax * s, ay * s, az * s};
    return q;
}

/* q^t for unit quaternions (scales the rotation angle) */
static inline r3d_quat_t r3d_quat_pow(r3d_quat_t q, float t)
{
    if (q.w < 0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
    float s = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
    if (s < 1e-7f) return r3d_quat_identity();
    float ang = 2.0f * atan2f(s, q.w) * t;
    float inv = 1.0f / s;
    return r3d_quat_axis_angle(q.x * inv, q.y * inv, q.z * inv, ang);
}

/* normalised lerp along the shortest arc - fine for per-frame smoothing */
static inline r3d_quat_t r3d_quat_nlerp(r3d_quat_t a, r3d_quat_t b, float t)
{
    float d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    float s = d < 0 ? -1.0f : 1.0f;
    r3d_quat_t r = {a.w + (s * b.w - a.w) * t, a.x + (s * b.x - a.x) * t,
                    a.y + (s * b.y - a.y) * t, a.z + (s * b.z - a.z) * t};
    return r3d_quat_normalize(r);
}

static inline r3d_mat3_t r3d_quat_to_mat3(r3d_quat_t q)
{
    r3d_mat3_t m;
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    m.m[0][0] = 1 - 2 * (yy + zz); m.m[0][1] = 2 * (xy - wz);     m.m[0][2] = 2 * (xz + wy);
    m.m[1][0] = 2 * (xy + wz);     m.m[1][1] = 1 - 2 * (xx + zz); m.m[1][2] = 2 * (yz - wx);
    m.m[2][0] = 2 * (xz - wy);     m.m[2][1] = 2 * (yz + wx);     m.m[2][2] = 1 - 2 * (xx + yy);
    return m;
}

static inline r3d_vec3_t r3d_quat_rotate(r3d_quat_t q, r3d_vec3_t v)
{
    r3d_mat3_t m = r3d_quat_to_mat3(q);
    r3d_vec3_t r = {m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z,
                    m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z,
                    m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z};
    return r;
}

#ifdef __cplusplus
}
#endif
