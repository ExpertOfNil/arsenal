#include "glmath.h"

#ifdef GLMATH_ENABLE_LAPACK
#include <lapacke.h>
#endif
#include <math.h>
#include <stdint.h>
#include <string.h>

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
    mat3f tmp;
    memcpy(tmp, MAT3F_IDENTITY, sizeof(mat3f));
    vec3f x_axis = {m[0], m[1], m[2]};
    vec3f y_axis = {m[3], m[4], m[5]};
    vec3f z_axis = {m[6], m[7], m[8]};
    // Ensure x-axis is orthogonal to the yz-plane
    vec3f_cross(y_axis, z_axis, x_axis);
    if (vec3f_normalize(x_axis) != 0) return -1;
    // Ensure y-axis is orthogonal to the xz-plane
    vec3f_cross(z_axis, x_axis, y_axis);
    if (vec3f_normalize(y_axis) != 0) return -1;
    // Ensure z-axis is normalized
    if (vec3f_normalize(z_axis) != 0) return -1;
    tmp[0] = x_axis[0];
    tmp[1] = x_axis[1];
    tmp[2] = x_axis[2];
    tmp[3] = y_axis[0];
    tmp[4] = y_axis[1];
    tmp[5] = y_axis[2];
    tmp[6] = z_axis[0];
    tmp[7] = z_axis[1];
    tmp[8] = z_axis[2];
    memcpy(out, tmp, sizeof(mat3f));
    return 0;
}
int mat3d_orthonormalize(const mat3d m, mat3d out) {
    mat3d tmp;
    memcpy(tmp, MAT3D_IDENTITY, sizeof(mat3d));
    vec3d x_axis = {m[0], m[1], m[2]};
    vec3d y_axis = {m[3], m[4], m[5]};
    vec3d z_axis = {m[6], m[7], m[8]};
    // Ensure x-axis is orthogonal to the yz-plane
    vec3d_cross(y_axis, z_axis, x_axis);
    if (vec3d_normalize(x_axis) != 0) return -1;
    // Ensure y-axis is orthogonal to the xz-plane
    vec3d_cross(z_axis, x_axis, y_axis);
    if (vec3d_normalize(y_axis) != 0) return -1;
    // Ensure z-axis is normalized
    if (vec3d_normalize(z_axis) != 0) return -1;
    tmp[0] = x_axis[0];
    tmp[1] = x_axis[1];
    tmp[2] = x_axis[2];
    tmp[3] = y_axis[0];
    tmp[4] = y_axis[1];
    tmp[5] = y_axis[2];
    tmp[6] = z_axis[0];
    tmp[7] = z_axis[1];
    tmp[8] = z_axis[2];
    memcpy(out, tmp, sizeof(mat3d));
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
int mat3f_project_so3(const mat3f m, mat3f out) {
    mat3f a;
    mat3f u;
    mat3f vt;
    mat3f result;

    float singular_values[3];
    float superb[2];

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    /* LAPACKE_dgesvd overwrites its input. */
    memcpy(a, m, sizeof(a));

    lapack_int info = LAPACKE_sgesvd(LAPACK_COL_MAJOR, 'A', 'A', 3, 3, a, 3,
                                     singular_values, u, 3, vt, 3, superb);

    if (info != 0) return (int)info;

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
    mat3d a;
    mat3d u;
    mat3d vt;
    mat3d result;

    double singular_values[3];
    double superb[2];

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(m[i])) return -1;
    }

    /* LAPACKE_dgesvd overwrites its input. */
    memcpy(a, m, sizeof(a));

    lapack_int info = LAPACKE_dgesvd(LAPACK_COL_MAJOR, 'A', 'A', 3, 3, a, 3,
                                     singular_values, u, 3, vt, 3, superb);

    if (info != 0) return (int)info;

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

void mat4f_inverse_rigid(const mat4f m, mat4f out) {
    mat4f tmp;
    memcpy(tmp, MAT4F_IDENTITY, sizeof(mat4f));

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
    mat4d tmp;
    memcpy(tmp, MAT4D_IDENTITY, sizeof(mat4d));

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

int mat4f_look_dir(const vec3f eye, const vec3f direction, const vec3f up,
                   mat4f m) {
    vec3f f;
    memcpy(f, direction, sizeof(vec3f));
    if (vec3f_normalize(f) != 0) return -1;
    vec3f r;
    vec3f_cross(f, up, r);
    if (vec3f_normalize(r) != 0) return -1;
    vec3f u;
    vec3f_cross(r, f, u);

    memcpy(m, MAT4F_IDENTITY, sizeof(mat4f));
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
int mat4d_look_dir(const vec3d eye, const vec3d direction, const vec3d up,
                   mat4d m) {
    vec3d f;
    memcpy(f, direction, sizeof(vec3d));
    if (vec3d_normalize(f) != 0) return -1;
    vec3d r;
    vec3d_cross(f, up, r);
    if (vec3d_normalize(r) != 0) return -1;
    vec3d u;
    vec3d_cross(r, f, u);

    memcpy(m, MAT4D_IDENTITY, sizeof(mat4d));
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

int mat4f_look_at(const vec3f eye, const vec3f target, const vec3f up,
                  mat4f out) {
    vec3f direction;
    vec3f_sub(target, eye, direction);
    return mat4f_look_dir(eye, direction, up, out);
}
int mat4d_look_at(const vec3d eye, const vec3d target, const vec3d up,
                  mat4d out) {
    vec3d direction;
    vec3d_sub(target, eye, direction);
    return mat4d_look_dir(eye, direction, up, out);
}

int mat4f_orthonormalize(const mat4f m, mat4f out) {
    mat4f tmp;
    memcpy(tmp, MAT4F_IDENTITY, sizeof(mat4f));
    vec3f x_axis = {m[0], m[1], m[2]};
    vec3f y_axis = {m[4], m[5], m[6]};
    vec3f z_axis = {m[8], m[9], m[10]};
    // Ensure x-axis is orthogonal to the yz-plane
    vec3f_cross(y_axis, z_axis, x_axis);
    if (vec3f_normalize(x_axis) != 0) return -1;
    // Ensure y-axis is orthogonal to the xz-plane
    vec3f_cross(z_axis, x_axis, y_axis);
    if (vec3f_normalize(y_axis) != 0) return -1;
    // Ensure z-axis is normalized
    if (vec3f_normalize(z_axis) != 0) return -1;
    tmp[0] = x_axis[0];
    tmp[1] = x_axis[1];
    tmp[2] = x_axis[2];
    tmp[4] = y_axis[0];
    tmp[5] = y_axis[1];
    tmp[6] = y_axis[2];
    tmp[8] = z_axis[0];
    tmp[9] = z_axis[1];
    tmp[10] = z_axis[2];
    tmp[12] = m[12];
    tmp[13] = m[13];
    tmp[14] = m[14];
    memcpy(out, tmp, sizeof(mat4f));
    return 0;
}
int mat4d_orthonormalize(const mat4d m, mat4d out) {
    mat4d tmp;
    memcpy(tmp, MAT4D_IDENTITY, sizeof(mat4d));
    vec3d x_axis = {m[0], m[1], m[2]};
    vec3d y_axis = {m[4], m[5], m[6]};
    vec3d z_axis = {m[8], m[9], m[10]};
    // Ensure x-axis is orthogonal to the yz-plane
    vec3d_cross(y_axis, z_axis, x_axis);
    if (vec3d_normalize(x_axis) != 0) return -1;
    // Ensure y-axis is orthogonal to the xz-plane
    vec3d_cross(z_axis, x_axis, y_axis);
    if (vec3d_normalize(y_axis) != 0) return -1;
    // Ensure z-axis is normalized
    if (vec3d_normalize(z_axis) != 0) return -1;
    tmp[0] = x_axis[0];
    tmp[1] = x_axis[1];
    tmp[2] = x_axis[2];
    tmp[4] = y_axis[0];
    tmp[5] = y_axis[1];
    tmp[6] = y_axis[2];
    tmp[8] = z_axis[0];
    tmp[9] = z_axis[1];
    tmp[10] = z_axis[2];
    tmp[12] = m[12];
    tmp[13] = m[13];
    tmp[14] = m[14];
    memcpy(out, tmp, sizeof(mat4d));
    return 0;
}
