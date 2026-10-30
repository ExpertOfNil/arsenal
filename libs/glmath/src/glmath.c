#include "glmath.h"

#ifdef GLMATH_ENABLE_LAPACK
#include <lapacke.h>
#endif
#include <math.h>
#include <stdint.h>
#include <string.h>

void vec3f_zero(vec3f v) {
    static const vec3f vz = VEC3F_ZERO;
    memcpy(v, vz, sizeof(vec3f));
}
void vec3d_zero(vec3d v) {
    static const vec3d vz = VEC3D_ZERO;
    memcpy(v, vz, sizeof(vec3d));
}

void vec3f_add(const vec3f va, const vec3f vb, vec3f out) {
    out[0] = va[0] + vb[0];
    out[1] = va[1] + vb[1];
    out[2] = va[2] + vb[2];
}
void vec3d_add(const vec3d va, const vec3d vb, vec3d out) {
    out[0] = va[0] + vb[0];
    out[1] = va[1] + vb[1];
    out[2] = va[2] + vb[2];
}

void vec3f_sub(const vec3f va, const vec3f vb, vec3f out) {
    out[0] = va[0] - vb[0];
    out[1] = va[1] - vb[1];
    out[2] = va[2] - vb[2];
}
void vec3d_sub(const vec3d va, const vec3d vb, vec3d out) {
    out[0] = va[0] - vb[0];
    out[1] = va[1] - vb[1];
    out[2] = va[2] - vb[2];
}

void vec3f_scale(const vec3f va, float s, vec3f out) {
    out[0] = va[0] * s;
    out[1] = va[1] * s;
    out[2] = va[2] * s;
}
void vec3d_scale(const vec3d va, double s, vec3d out) {
    out[0] = va[0] * s;
    out[1] = va[1] * s;
    out[2] = va[2] * s;
}

float vec3f_dot(const vec3f va, const vec3f vb) {
    return va[0] * vb[0] + va[1] * vb[1] + va[2] * vb[2];
}
double vec3d_dot(const vec3d va, const vec3d vb) {
    return va[0] * vb[0] + va[1] * vb[1] + va[2] * vb[2];
}

float vec3f_length(const vec3f v) { return hypotf(hypotf(v[0], v[1]), v[2]); }
double vec3d_length(const vec3d v) { return hypot(hypot(v[0], v[1]), v[2]); }

int vec3f_normalize(vec3f v) {
    float mag = vec3f_length(v);
    if (mag == 0 || !isfinite(mag)) return -1;
    v[0] /= mag;
    v[1] /= mag;
    v[2] /= mag;
    return 0;
}
int vec3d_normalize(vec3d v) {
    double mag = vec3d_length(v);
    if (mag == 0 || !isfinite(mag)) return -1;
    v[0] /= mag;
    v[1] /= mag;
    v[2] /= mag;
    return 0;
}

void vec3f_cross(const vec3f va, const vec3f vb, vec3f out) {
    float x = va[1] * vb[2] - va[2] * vb[1];
    float y = va[2] * vb[0] - va[0] * vb[2];
    float z = va[0] * vb[1] - va[1] * vb[0];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}
void vec3d_cross(const vec3d va, const vec3d vb, vec3d out) {
    double x = va[1] * vb[2] - va[2] * vb[1];
    double y = va[2] * vb[0] - va[0] * vb[2];
    double z = va[0] * vb[1] - va[1] * vb[0];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

void vec4f_zero(vec4f v) {
    static const vec4f vz = VEC4F_ZERO;
    memcpy(v, vz, sizeof(vec4f));
}
void vec4d_zero(vec4d v) {
    static const vec4d vz = VEC4D_ZERO;
    memcpy(v, vz, sizeof(vec4d));
}

float vec4f_dot(const vec4f va, const vec4f vb) {
    return va[0] * vb[0] + va[1] * vb[1] + va[2] * vb[2] + va[3] * vb[3];
}
double vec4d_dot(const vec4d va, const vec4d vb) {
    return va[0] * vb[0] + va[1] * vb[1] + va[2] * vb[2] + va[3] * vb[3];
}

float vec4f_length(const vec4f v) {
    return hypotf(hypotf(v[0], v[1]), hypotf(v[2], v[3]));
}
double vec4d_length(const vec4d v) {
    return hypot(hypot(v[0], v[1]), hypot(v[2], v[3]));
}

int vec4f_normalize(vec4f v) {
    float mag = vec4f_length(v);
    if (mag == 0 || !isfinite(mag)) return -1;
    v[0] /= mag;
    v[1] /= mag;
    v[2] /= mag;
    v[3] /= mag;
    return 0;
}
int vec4d_normalize(vec4d v) {
    double mag = vec4d_length(v);
    if (mag == 0 || !isfinite(mag)) return -1;
    v[0] /= mag;
    v[1] /= mag;
    v[2] /= mag;
    v[3] /= mag;
    return 0;
}

void quatf_identity(quatf q) {
    static const quatf identity = QUATF_IDENTITY;
    memcpy(q, identity, sizeof(quatf));
}
void quatd_identity(quatd q) {
    static const quatd identity = QUATD_IDENTITY;
    memcpy(q, identity, sizeof(quatd));
}

int quatf_normalize(quatf q) { return vec4f_normalize(q); }
int quatd_normalize(quatd q) { return vec4d_normalize(q); }

void quatf_to_mat3f(const quatf q, mat3f m) { mat3f_from_quatf(q, m); }
void quatd_to_mat3d(const quatd q, mat3d m) { mat3d_from_quatd(q, m); }

void quatf_from_mat3f(const mat3f m, quatf q) {
    float four_w_sq_m1 = m[0] + m[4] + m[8];
    float four_x_sq_m1 = m[0] - m[4] - m[8];
    float four_y_sq_m1 = m[4] - m[0] - m[8];
    float four_z_sq_m1 = m[8] - m[0] - m[4];

    uint32_t biggest_index = 0;
    float four_biggest_sq_m1 = four_w_sq_m1;
    if (four_x_sq_m1 > four_biggest_sq_m1) {
        four_biggest_sq_m1 = four_x_sq_m1;
        biggest_index = 1;
    }
    if (four_y_sq_m1 > four_biggest_sq_m1) {
        four_biggest_sq_m1 = four_y_sq_m1;
        biggest_index = 2;
    }
    if (four_z_sq_m1 > four_biggest_sq_m1) {
        four_biggest_sq_m1 = four_z_sq_m1;
        biggest_index = 3;
    }

    float biggest_val = sqrtf(1.0 + four_biggest_sq_m1) * 0.5;
    float s = 0.25 / biggest_val;

    switch (biggest_index) {
        case 0:
            q[0] = (m[5] - m[7]) * s;
            q[1] = (m[6] - m[2]) * s;
            q[2] = (m[1] - m[3]) * s;
            q[3] = biggest_val;
            break;
        case 1:
            q[0] = biggest_val;
            q[1] = (m[1] + m[3]) * s;
            q[2] = (m[6] + m[2]) * s;
            q[3] = (m[5] - m[7]) * s;
            break;
        case 2:
            q[0] = (m[1] + m[3]) * s;
            q[1] = biggest_val;
            q[2] = (m[5] + m[7]) * s;
            q[3] = (m[6] - m[2]) * s;
            break;
        case 3:
            q[0] = (m[6] + m[2]) * s;
            q[1] = (m[5] + m[7]) * s;
            q[2] = biggest_val;
            q[3] = (m[1] - m[3]) * s;
            break;
    }
}
void quatd_from_mat3d(const mat3d m, quatd q) {
    double four_w_sq_m1 = m[0] + m[4] + m[8];
    double four_x_sq_m1 = m[0] - m[4] - m[8];
    double four_y_sq_m1 = m[4] - m[0] - m[8];
    double four_z_sq_m1 = m[8] - m[0] - m[4];

    uint32_t biggest_index = 0;
    double four_biggest_sq_m1 = four_w_sq_m1;
    if (four_x_sq_m1 > four_biggest_sq_m1) {
        four_biggest_sq_m1 = four_x_sq_m1;
        biggest_index = 1;
    }
    if (four_y_sq_m1 > four_biggest_sq_m1) {
        four_biggest_sq_m1 = four_y_sq_m1;
        biggest_index = 2;
    }
    if (four_z_sq_m1 > four_biggest_sq_m1) {
        four_biggest_sq_m1 = four_z_sq_m1;
        biggest_index = 3;
    }

    double biggest_val = sqrt(1.0 + four_biggest_sq_m1) * 0.5;
    double s = 0.25 / biggest_val;

    switch (biggest_index) {
        case 0:
            q[0] = (m[5] - m[7]) * s;
            q[1] = (m[6] - m[2]) * s;
            q[2] = (m[1] - m[3]) * s;
            q[3] = biggest_val;
            break;
        case 1:
            q[0] = biggest_val;
            q[1] = (m[1] + m[3]) * s;
            q[2] = (m[6] + m[2]) * s;
            q[3] = (m[5] - m[7]) * s;
            break;
        case 2:
            q[0] = (m[1] + m[3]) * s;
            q[1] = biggest_val;
            q[2] = (m[5] + m[7]) * s;
            q[3] = (m[6] - m[2]) * s;
            break;
        case 3:
            q[0] = (m[6] + m[2]) * s;
            q[1] = (m[5] + m[7]) * s;
            q[2] = biggest_val;
            q[3] = (m[1] - m[3]) * s;
            break;
    }
}

void quatf_slerp(const quatf start, const quatf end, float t, quatf out) {
    quatf a;
    memcpy(a, start, sizeof(a));
    quatf b;
    memcpy(b, end, sizeof(b));
    quatf result = QUATF_IDENTITY;

    float dot = vec4f_dot(a, b);

    // Choose the shorter arc between -q and q
    if (dot < 0.0) {
        for (size_t i = 0; i < 4; ++i) b[i] = -b[i];
        dot = -dot;
    }

    // Avoid ill-conditioned sin(theta) by doing nlerp instead
    if (dot > 0.9995) {
        for (size_t i = 0; i < 4; ++i) {
            result[i] = (1.0 - t) * a[i] + t * b[i];
        }

        (void)quatf_normalize(result);
    } else {
        // Guard agains rounding errors
        if (dot > 1.0) dot = 1.0;
        const float theta = acos(dot);
        const float inv_sin_theta = 1.0 / sin(theta);
        const float wa = sin((1.0 - t) * theta) * inv_sin_theta;
        const float wb = sin(t * theta) * inv_sin_theta;

        for (size_t i = 0; i < 4; ++i) {
            result[i] = wa * a[i] + wb * b[i];
        }
    }

    memcpy(out, result, sizeof(result));
}
void quatd_slerp(const quatd start, const quatd end, double t, quatd out) {
    quatd a;
    memcpy(a, start, sizeof(a));
    quatd b;
    memcpy(b, end, sizeof(b));
    quatd result = QUATD_IDENTITY;

    double dot = vec4d_dot(a, b);

    // Choose the shorter arc between -q and q
    if (dot < 0.0) {
        for (size_t i = 0; i < 4; ++i) b[i] = -b[i];
        dot = -dot;
    }

    // Avoid ill-conditioned sin(theta) by doing nlerp instead
    if (dot > 0.9995) {
        for (size_t i = 0; i < 4; ++i) {
            result[i] = (1.0 - t) * a[i] + t * b[i];
        }

        (void)quatd_normalize(result);
    } else {
        // Guard agains rounding errors
        if (dot > 1.0) dot = 1.0;
        const double theta = acos(dot);
        const double inv_sin_theta = 1.0 / sin(theta);
        const double wa = sin((1.0 - t) * theta) * inv_sin_theta;
        const double wb = sin(t * theta) * inv_sin_theta;

        for (size_t i = 0; i < 4; ++i) {
            result[i] = wa * a[i] + wb * b[i];
        }
    }

    memcpy(out, result, sizeof(result));
}

void mat3f_identity(mat3f m) {
    static const mat3f identity = MAT3F_IDENTITY;
    memcpy(m, identity, sizeof(mat3f));
}
void mat3d_identity(mat3d m) {
    static const mat3d identity = MAT3D_IDENTITY;
    memcpy(m, identity, sizeof(mat3d));
}

float mat3f_det(const mat3f m) {
    float a = m[0] * m[4] * m[8];
    float b = m[3] * m[7] * m[2];
    float c = m[6] * m[1] * m[5];
    float d = m[6] * m[4] * m[2];
    float e = m[3] * m[1] * m[8];
    float f = m[0] * m[7] * m[5];
    return a + b + c - d - e - f;
}
double mat3d_det(const mat3d m) {
    double a = m[0] * m[4] * m[8];
    double b = m[3] * m[7] * m[2];
    double c = m[6] * m[1] * m[5];
    double d = m[6] * m[4] * m[2];
    double e = m[3] * m[1] * m[8];
    double f = m[0] * m[7] * m[5];
    return a + b + c - d - e - f;
}

void mat3f_transpose(const mat3f m, mat3f out) {
    mat3f tmp;
    tmp[0] = m[0];
    tmp[1] = m[3];
    tmp[2] = m[6];
    tmp[3] = m[1];
    tmp[4] = m[4];
    tmp[5] = m[7];
    tmp[6] = m[2];
    tmp[7] = m[5];
    tmp[8] = m[8];

    memcpy(out, tmp, sizeof(mat3f));
}
void mat3d_transpose(const mat3d m, mat3d out) {
    mat3d tmp;
    tmp[0] = m[0];
    tmp[1] = m[3];
    tmp[2] = m[6];
    tmp[3] = m[1];
    tmp[4] = m[4];
    tmp[5] = m[7];
    tmp[6] = m[2];
    tmp[7] = m[5];
    tmp[8] = m[8];

    memcpy(out, tmp, sizeof(mat3d));
}

int mat3f_inverse(const mat3f m, mat3f out) {
    if (m == NULL || out == NULL) return -1;
    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    const double det = mat3f_det(m);
    if (!isfinite(det) || det == 0.0) return -1;
    const double inv_det = 1.0 / det;
    mat3f result;

    result[0] = (m[4] * m[8] - m[7] * m[5]) * inv_det;
    result[3] = (m[6] * m[5] - m[3] * m[8]) * inv_det;
    result[6] = (m[3] * m[7] - m[6] * m[4]) * inv_det;

    result[1] = (m[7] * m[2] - m[1] * m[8]) * inv_det;
    result[4] = (m[0] * m[8] - m[6] * m[2]) * inv_det;
    result[7] = (m[6] * m[1] - m[0] * m[7]) * inv_det;

    result[2] = (m[1] * m[5] - m[4] * m[2]) * inv_det;
    result[5] = (m[3] * m[2] - m[0] * m[5]) * inv_det;
    result[8] = (m[0] * m[4] - m[3] * m[1]) * inv_det;

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(result[i])) return -1;
    }

    memcpy(out, result, sizeof(result));
    return 0;
}
int mat3d_inverse(const mat3d m, mat3d out) {
    if (m == NULL || out == NULL) return -1;
    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    const double det = mat3d_det(m);
    if (!isfinite(det) || det == 0.0) return -1;
    const double inv_det = 1.0 / det;
    mat3d result;

    result[0] = (m[4] * m[8] - m[7] * m[5]) * inv_det;
    result[3] = (m[6] * m[5] - m[3] * m[8]) * inv_det;
    result[6] = (m[3] * m[7] - m[6] * m[4]) * inv_det;

    result[1] = (m[7] * m[2] - m[1] * m[8]) * inv_det;
    result[4] = (m[0] * m[8] - m[6] * m[2]) * inv_det;
    result[7] = (m[6] * m[1] - m[0] * m[7]) * inv_det;

    result[2] = (m[1] * m[5] - m[4] * m[2]) * inv_det;
    result[5] = (m[3] * m[2] - m[0] * m[5]) * inv_det;
    result[8] = (m[0] * m[4] - m[3] * m[1]) * inv_det;

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(result[i])) return -1;
    }

    memcpy(out, result, sizeof(result));
    return 0;
}

void mat3f_from_quatf(const quatf q, mat3f m) {
    float xx = q[0] * q[0];
    float yy = q[1] * q[1];
    float zz = q[2] * q[2];
    float xy = q[0] * q[1];
    float xz = q[0] * q[2];
    float yz = q[1] * q[2];
    float wx = q[3] * q[0];
    float wy = q[3] * q[1];
    float wz = q[3] * q[2];

    m[0] = 1.0 - 2.0 * (yy + zz);
    m[1] = 2.0 * (xy + wz);
    m[2] = 2.0 * (xz - wy);

    m[3] = 2.0 * (xy - wz);
    m[4] = 1.0 - 2.0 * (xx + zz);
    m[5] = 2.0 * (yz + wx);

    m[6] = 2.0 * (xz + wy);
    m[7] = 2.0 * (yz - wx);
    m[8] = 1.0 - 2.0 * (xx + yy);
}
void mat3d_from_quatd(const quatd q, mat3d m) {
    double xx = q[0] * q[0];
    double yy = q[1] * q[1];
    double zz = q[2] * q[2];
    double xy = q[0] * q[1];
    double xz = q[0] * q[2];
    double yz = q[1] * q[2];
    double wx = q[3] * q[0];
    double wy = q[3] * q[1];
    double wz = q[3] * q[2];

    m[0] = 1.0 - 2.0 * (yy + zz);
    m[1] = 2.0 * (xy + wz);
    m[2] = 2.0 * (xz - wy);

    m[3] = 2.0 * (xy - wz);
    m[4] = 1.0 - 2.0 * (xx + zz);
    m[5] = 2.0 * (yz + wx);

    m[6] = 2.0 * (xz + wy);
    m[7] = 2.0 * (yz - wx);
    m[8] = 1.0 - 2.0 * (xx + yy);
}

int mat3d_from_rotation_vector(const vec3d omega, mat3d out) {
    if (omega == NULL || out == NULL) return -1;
    for (size_t i = 0; i < 3; ++i) {
        if (!isfinite(omega[i])) {
            return -1;
        }
    }

    const double theta = hypot(hypot(omega[0], omega[1]), omega[2]);
    if (!isfinite(theta)) {
        return -1;
    }

    if (theta == 0.0) {
        mat3d_identity(out);
        return 0;
    }

    const double x = omega[0] / theta;
    const double y = omega[1] / theta;
    const double z = omega[2] / theta;

    const double sine = sin(theta);
    const double half_sine = sin(0.5 * theta);
    const double one_minus_cosine = 2.0 * half_sine * half_sine;

    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;

    mat3d result;
    result[0] = 1.0 - one_minus_cosine * (yy + zz);
    result[1] = one_minus_cosine * xy + sine * z;
    result[2] = one_minus_cosine * xz - sine * y;

    result[3] = one_minus_cosine * xy - sine * z;
    result[4] = 1.0 - one_minus_cosine * (xx + zz);
    result[5] = one_minus_cosine * yz + sine * x;

    result[6] = one_minus_cosine * xz + sine * y;
    result[7] = one_minus_cosine * yz - sine * x;
    result[8] = 1.0 - one_minus_cosine * (xx + yy);

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(result[i])) {
            return -1;
        }
    }

    for (size_t i = 0; i < 9; ++i) {
        out[i] = result[i];
    }

    return 0;
}

void mat3f_mul_vec3f(const mat3f m, const vec3f v, vec3f out) {
    vec3f tmp;
    tmp[0] = m[0] * v[0] + m[3] * v[1] + m[6] * v[2];
    tmp[1] = m[1] * v[0] + m[4] * v[1] + m[7] * v[2];
    tmp[2] = m[2] * v[0] + m[5] * v[1] + m[8] * v[2];
    memcpy(out, tmp, sizeof(vec3f));
}
void mat3d_mul_vec3d(const mat3d m, const vec3d v, vec3d out) {
    vec3d tmp;
    tmp[0] = m[0] * v[0] + m[3] * v[1] + m[6] * v[2];
    tmp[1] = m[1] * v[0] + m[4] * v[1] + m[7] * v[2];
    tmp[2] = m[2] * v[0] + m[5] * v[1] + m[8] * v[2];
    memcpy(out, tmp, sizeof(vec3d));
}

void mat3f_to_quatf(const mat3f m, quatf q) { quatf_from_mat3f(m, q); }
void mat3d_to_quatd(const mat3d m, quatd q) { quatd_from_mat3d(m, q); }

int mat3f_orthonormalize(const mat3f m, mat3f out) {
    if (m == NULL || out == NULL) return -1;
    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    vec3f x_axis = {m[0], m[1], m[2]};
    vec3f y_axis = {m[3], m[4], m[5]};
    vec3f z_axis;

    if (vec3f_normalize(x_axis) != 0) return -1;
    const double projection = vec3f_dot(y_axis, x_axis);
    y_axis[0] -= projection * x_axis[0];
    y_axis[1] -= projection * x_axis[1];
    y_axis[2] -= projection * x_axis[2];

    if (vec3f_normalize(y_axis) != 0) return -1;

    vec3f_cross(x_axis, y_axis, z_axis);
    if (vec3f_normalize(z_axis) != 0) return -1;

    mat3f result = {
        x_axis[0],
        x_axis[1],
        x_axis[2],
        y_axis[0],
        y_axis[1],
        y_axis[2],
        z_axis[0],
        z_axis[1],
        z_axis[2]
    };

    memcpy(out, result, sizeof(result));
    return 0;
}
int mat3d_orthonormalize(const mat3d m, mat3d out) {
    if (m == NULL || out == NULL) return -1;
    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    vec3d x_axis = {m[0], m[1], m[2]};
    vec3d y_axis = {m[3], m[4], m[5]};
    vec3d z_axis;

    if (vec3d_normalize(x_axis) != 0) return -1;
    const double projection = vec3d_dot(y_axis, x_axis);
    y_axis[0] -= projection * x_axis[0];
    y_axis[1] -= projection * x_axis[1];
    y_axis[2] -= projection * x_axis[2];

    if (vec3d_normalize(y_axis) != 0) return -1;

    vec3d_cross(x_axis, y_axis, z_axis);
    if (vec3d_normalize(z_axis) != 0) return -1;

    mat3d result = {
        x_axis[0],
        x_axis[1],
        x_axis[2],
        y_axis[0],
        y_axis[1],
        y_axis[2],
        z_axis[0],
        z_axis[1],
        z_axis[2]
    };

    memcpy(out, result, sizeof(result));
    return 0;
}

void mat3f_mul(const mat3f ma, const mat3f mb, mat3f out) {
    mat3f result;

    for (size_t col = 0; col < 3; ++col) {
        for (size_t row = 0; row < 3; ++row) {
            float sum = 0.0;

            for (size_t k = 0; k < 3; ++k) {
                sum += ma[k * 3 + row] * mb[col * 3 + k];
            }

            result[col * 3 + row] = sum;
        }
    }

    memcpy(out, result, sizeof(result));
}
void mat3d_mul(const mat3d ma, const mat3d mb, mat3d out) {
    mat3d result;

    for (size_t col = 0; col < 3; ++col) {
        for (size_t row = 0; row < 3; ++row) {
            double sum = 0.0;

            for (size_t k = 0; k < 3; ++k) {
                sum += ma[k * 3 + row] * mb[col * 3 + k];
            }

            result[col * 3 + row] = sum;
        }
    }

    memcpy(out, result, sizeof(result));
}

#ifdef GLMATH_ENABLE_LAPACK
int mat3f_svd(const mat3f m, mat3f u, vec3f singular_values, mat3f vt) {
    if (m == NULL || u == NULL || singular_values == NULL || vt == NULL) {
        return -1;
    }

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    mat3f a;
    mat3f result_u;
    mat3f result_vt;
    vec3f result_singular_values;
    float superb[2];

    // LAPACKE_sgesvd overwrites its input, so preserve m.
    memcpy(a, m, sizeof(a));

    lapack_int info = LAPACKE_sgesvd(
        LAPACK_COL_MAJOR,
        'A',
        'A',
        3,
        3,
        a,
        3,
        result_singular_values,
        result_u,
        3,
        result_vt,
        3,
        superb
    );

    if (info != 0) return (int)info;

    memcpy(u, result_u, sizeof(result_u));
    memcpy(
        singular_values, result_singular_values, sizeof(result_singular_values)
    );
    memcpy(vt, result_vt, sizeof(result_vt));

    return 0;
}
int mat3d_svd(const mat3d m, mat3d u, vec3d singular_values, mat3d vt) {
    if (m == NULL || u == NULL || singular_values == NULL || vt == NULL) {
        return -1;
    }

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    mat3d a;
    mat3d result_u;
    mat3d result_vt;
    vec3d result_singular_values;
    double superb[2];

    // LAPACKE_dgesvd overwrites its input, so preserve m.
    memcpy(a, m, sizeof(a));

    lapack_int info = LAPACKE_dgesvd(
        LAPACK_COL_MAJOR,
        'A',
        'A',
        3,
        3,
        a,
        3,
        result_singular_values,
        result_u,
        3,
        result_vt,
        3,
        superb
    );

    if (info != 0) return (int)info;

    memcpy(u, result_u, sizeof(result_u));
    memcpy(
        singular_values, result_singular_values, sizeof(result_singular_values)
    );
    memcpy(vt, result_vt, sizeof(result_vt));

    return 0;
}
#endif

#ifdef GLMATH_ENABLE_LAPACK
int mat3f_project_so3(const mat3f m, mat3f out) {
    if (out == NULL) return -1;

    mat3f u;
    mat3f vt;
    mat3f result;
    vec3f singular_values;

    int status = mat3f_svd(m, u, singular_values, vt);
    if (status != 0) return status;

    /*
     * det(U V^T) = det(U) det(V^T).
     * If negative, reverse the U column associated with the
     * smallest singular value.
     */
    float orientation = mat3f_det(u) * mat3f_det(vt);

    if (!isfinite(orientation)) return -1;

    if (orientation < 0.0) {
        u[6] = -u[6];
        u[7] = -u[7];
        u[8] = -u[8];
    }

    mat3f_mul(u, vt, result);
    memcpy(out, result, sizeof(result));

    return 0;
}
int mat3d_project_so3(const mat3d m, mat3d out) {
    if (out == NULL) return -1;

    mat3d u;
    mat3d vt;
    mat3d result;
    vec3d singular_values;

    int status = mat3d_svd(m, u, singular_values, vt);
    if (status != 0) return status;

    /*
     * det(U V^T) = det(U) det(V^T).
     * If negative, reverse the U column associated with the
     * smallest singular value.
     */
    double orientation = mat3d_det(u) * mat3d_det(vt);

    if (!isfinite(orientation)) return -1;

    if (orientation < 0.0) {
        u[6] = -u[6];
        u[7] = -u[7];
        u[8] = -u[8];
    }

    mat3d_mul(u, vt, result);
    memcpy(out, result, sizeof(result));

    return 0;
}
#endif

void mat4f_identity(mat4f m) {
    static const mat4f identity = MAT4F_IDENTITY;
    memcpy(m, identity, sizeof(mat4f));
}
void mat4d_identity(mat4d m) {
    static const mat4d identity = MAT4D_IDENTITY;
    memcpy(m, identity, sizeof(mat4d));
}

void mat4f_transpose(const mat4f m, mat4f out) {
    mat4f tmp;
    tmp[0] = m[0];
    tmp[1] = m[4];
    tmp[2] = m[8];
    tmp[3] = m[12];
    tmp[4] = m[1];
    tmp[5] = m[5];
    tmp[6] = m[9];
    tmp[7] = m[13];
    tmp[8] = m[2];
    tmp[9] = m[6];
    tmp[10] = m[10];
    tmp[11] = m[14];
    tmp[12] = m[3];
    tmp[13] = m[7];
    tmp[14] = m[11];
    tmp[15] = m[15];
    memcpy(out, tmp, sizeof(mat4f));
}
void mat4d_transpose(const mat4d m, mat4d out) {
    mat4d tmp;
    tmp[0] = m[0];
    tmp[1] = m[4];
    tmp[2] = m[8];
    tmp[3] = m[12];
    tmp[4] = m[1];
    tmp[5] = m[5];
    tmp[6] = m[9];
    tmp[7] = m[13];
    tmp[8] = m[2];
    tmp[9] = m[6];
    tmp[10] = m[10];
    tmp[11] = m[14];
    tmp[12] = m[3];
    tmp[13] = m[7];
    tmp[14] = m[11];
    tmp[15] = m[15];
    memcpy(out, tmp, sizeof(mat4d));
}

void mat4f_mul(const mat4f ma, const mat4f mb, mat4f out) {
    mat4f result;

    for (size_t col = 0; col < 4; ++col) {
        for (size_t row = 0; row < 4; ++row) {
            float sum = 0.0;
            for (size_t k = 0; k < 4; ++k) {
                sum += ma[row + 4 * k] * mb[k + 4 * col];
            }
            result[row + 4 * col] = sum;
        }
    }

    memcpy(out, result, sizeof(result));
}
void mat4d_mul(const mat4d ma, const mat4d mb, mat4d out) {
    mat4d result;

    for (size_t col = 0; col < 4; ++col) {
        for (size_t row = 0; row < 4; ++row) {
            double sum = 0.0;
            for (size_t k = 0; k < 4; ++k) {
                sum += ma[row + 4 * k] * mb[k + 4 * col];
            }
            result[row + 4 * col] = sum;
        }
    }

    memcpy(out, result, sizeof(result));
}

void mat4f_mul_vec4f(const mat4f m, const vec4f v, vec4f out) {
    vec4f tmp;
    tmp[0] = m[0] * v[0] + m[4] * v[1] + m[8] * v[2] + m[12] * v[3];
    tmp[1] = m[1] * v[0] + m[5] * v[1] + m[9] * v[2] + m[13] * v[3];
    tmp[2] = m[2] * v[0] + m[6] * v[1] + m[10] * v[2] + m[14] * v[3];
    tmp[3] = m[3] * v[0] + m[7] * v[1] + m[11] * v[2] + m[15] * v[3];
    memcpy(out, tmp, sizeof(vec4f));
}
void mat4d_mul_vec4d(const mat4d m, const vec4d v, vec4d out) {
    vec4d tmp;
    tmp[0] = m[0] * v[0] + m[4] * v[1] + m[8] * v[2] + m[12] * v[3];
    tmp[1] = m[1] * v[0] + m[5] * v[1] + m[9] * v[2] + m[13] * v[3];
    tmp[2] = m[2] * v[0] + m[6] * v[1] + m[10] * v[2] + m[14] * v[3];
    tmp[3] = m[3] * v[0] + m[7] * v[1] + m[11] * v[2] + m[15] * v[3];
    memcpy(out, tmp, sizeof(vec4d));
}

#ifdef GLMATH_ENABLE_LAPACK
int mat4f_inverse_cholesky(const mat4f m, mat4f out) {
    if (m == NULL || out == NULL) return -1;

    mat4f result;
    for (size_t i = 0; i < 16; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    // LAPACK overwrites the input.  On success: M = L * L^T
    memcpy(result, m, sizeof(result));
    lapack_int info = LAPACKE_spotrf(LAPACK_COL_MAJOR, 'L', 4, result, 4);
    if (info != 0) return (int)info;

    // Compute M^{-1} from its Cholesky factorization
    info = LAPACKE_spotri(LAPACK_COL_MAJOR, 'L', 4, result, 4);
    if (info != 0) return (int)info;

    for (size_t col = 0; col < 4; ++col) {
        for (size_t row = 0; row < col; ++row) {
            result[row + 4 * col] = result[col + 4 * row];
        }
    }

    for (size_t i = 0; i < 16; ++i) {
        if (!isfinite(result[i])) return -1;
    }

    memcpy(out, result, sizeof(result));
    return 0;
}
int mat4d_inverse_cholesky(const mat4d m, mat4d out) {
    if (m == NULL || out == NULL) return -1;

    mat4d result;
    for (size_t i = 0; i < 16; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    // LAPACK overwrites the input.  On success: M = L * L^T
    memcpy(result, m, sizeof(result));
    lapack_int info = LAPACKE_dpotrf(LAPACK_COL_MAJOR, 'L', 4, result, 4);
    if (info != 0) return (int)info;

    // Compute M^{-1} from its Cholesky factorization
    info = LAPACKE_dpotri(LAPACK_COL_MAJOR, 'L', 4, result, 4);
    if (info != 0) return (int)info;

    for (size_t col = 0; col < 4; ++col) {
        for (size_t row = 0; row < col; ++row) {
            result[row + 4 * col] = result[col + 4 * row];
        }
    }

    for (size_t i = 0; i < 16; ++i) {
        if (!isfinite(result[i])) return -1;
    }

    memcpy(out, result, sizeof(result));
    return 0;
}
#endif

void mat4f_inverse_rigid(const mat4f m, mat4f out) {
    mat4f tmp = MAT4F_IDENTITY;

    vec3f x = {m[0], m[1], m[2]};
    vec3f y = {m[4], m[5], m[6]};
    vec3f z = {m[8], m[9], m[10]};

    tmp[0] = x[0];
    tmp[1] = y[0];
    tmp[2] = z[0];

    tmp[4] = x[1];
    tmp[5] = y[1];
    tmp[6] = z[1];

    tmp[8] = x[2];
    tmp[9] = y[2];
    tmp[10] = z[2];

    tmp[12] = -(x[0] * m[12] + x[1] * m[13] + x[2] * m[14]);
    tmp[13] = -(y[0] * m[12] + y[1] * m[13] + y[2] * m[14]);
    tmp[14] = -(z[0] * m[12] + z[1] * m[13] + z[2] * m[14]);
    memcpy(out, tmp, sizeof(mat4f));
}
void mat4d_inverse_rigid(const mat4d m, mat4d out) {
    mat4d tmp = MAT4D_IDENTITY;

    vec3d x = {m[0], m[1], m[2]};
    vec3d y = {m[4], m[5], m[6]};
    vec3d z = {m[8], m[9], m[10]};

    tmp[0] = x[0];
    tmp[1] = y[0];
    tmp[2] = z[0];

    tmp[4] = x[1];
    tmp[5] = y[1];
    tmp[6] = z[1];

    tmp[8] = x[2];
    tmp[9] = y[2];
    tmp[10] = z[2];

    tmp[12] = -(x[0] * m[12] + x[1] * m[13] + x[2] * m[14]);
    tmp[13] = -(y[0] * m[12] + y[1] * m[13] + y[2] * m[14]);
    tmp[14] = -(z[0] * m[12] + z[1] * m[13] + z[2] * m[14]);
    memcpy(out, tmp, sizeof(mat4d));
}

int mat4f_look_dir(
    const vec3f eye, const vec3f direction, const vec3f up, mat4f m
) {
    vec3f f;
    memcpy(f, direction, sizeof(vec3f));
    if (vec3f_normalize(f) != 0) return -1;
    vec3f r;
    vec3f_cross(f, up, r);
    if (vec3f_normalize(r) != 0) return -1;
    vec3f u;
    vec3f_cross(r, f, u);

    mat4f_identity(m);
    m[0] = r[0];
    m[4] = r[1];
    m[8] = r[2];

    m[1] = u[0];
    m[5] = u[1];
    m[9] = u[2];

    m[2] = -f[0];
    m[6] = -f[1];
    m[10] = -f[2];

    m[12] = -vec3f_dot(r, eye);
    m[13] = -vec3f_dot(u, eye);
    m[14] = vec3f_dot(f, eye);
    return 0;
}
int mat4d_look_dir(
    const vec3d eye, const vec3d direction, const vec3d up, mat4d m
) {
    vec3d f;
    memcpy(f, direction, sizeof(vec3d));
    if (vec3d_normalize(f) != 0) return -1;
    vec3d r;
    vec3d_cross(f, up, r);
    if (vec3d_normalize(r) != 0) return -1;
    vec3d u;
    vec3d_cross(r, f, u);

    mat4d_identity(m);
    m[0] = r[0];
    m[4] = r[1];
    m[8] = r[2];

    m[1] = u[0];
    m[5] = u[1];
    m[9] = u[2];

    m[2] = -f[0];
    m[6] = -f[1];
    m[10] = -f[2];

    m[12] = -vec3d_dot(r, eye);
    m[13] = -vec3d_dot(u, eye);
    m[14] = vec3d_dot(f, eye);
    return 0;
}

int mat4f_look_at(
    const vec3f eye, const vec3f target, const vec3f up, mat4f out
) {
    vec3f direction;
    vec3f_sub(target, eye, direction);
    return mat4f_look_dir(eye, direction, up, out);
}
int mat4d_look_at(
    const vec3d eye, const vec3d target, const vec3d up, mat4d out
) {
    vec3d direction;
    vec3d_sub(target, eye, direction);
    return mat4d_look_dir(eye, direction, up, out);
}

int mat4f_orthonormalize(const mat4f m, mat4f out) {
    if (m == NULL || out == NULL) return -1;
    for (size_t i = 0; i < 16; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    const mat3f linear = {
        m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]
    };

    mat3f rotation;
    if (mat3f_orthonormalize(linear, rotation) != 0) return -1;

    mat4f result = MAT4F_IDENTITY;

    result[0] = rotation[0];
    result[1] = rotation[1];
    result[2] = rotation[2];

    result[4] = rotation[3];
    result[5] = rotation[4];
    result[6] = rotation[5];

    result[8] = rotation[6];
    result[9] = rotation[7];
    result[10] = rotation[8];

    result[12] = m[12];
    result[13] = m[13];
    result[14] = m[14];

    memcpy(out, result, sizeof(result));
    return 0;
}
int mat4d_orthonormalize(const mat4d m, mat4d out) {
    if (m == NULL || out == NULL) return -1;
    for (size_t i = 0; i < 16; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    const mat3d linear = {
        m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]
    };

    mat3d rotation;
    if (mat3d_orthonormalize(linear, rotation) != 0) return -1;

    mat4d result = MAT4D_IDENTITY;

    result[0] = rotation[0];
    result[1] = rotation[1];
    result[2] = rotation[2];

    result[4] = rotation[3];
    result[5] = rotation[4];
    result[6] = rotation[5];

    result[8] = rotation[6];
    result[9] = rotation[7];
    result[10] = rotation[8];

    result[12] = m[12];
    result[13] = m[13];
    result[14] = m[14];

    memcpy(out, result, sizeof(result));
    return 0;
}
