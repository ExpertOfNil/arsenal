#ifndef GLMATH_H
#define GLMATH_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#define VEC3F_ZERO {0.0f, 0.0f, 0.0f}
#define VEC4F_ZERO {0.0f, 0.0f, 0.0f, 0.0f}
#define QUATF_IDENTITY {0.0f, 0.0f, 0.0f, 1.0f}
// clang-format off
#define MAT3F_IDENTITY { \
    1.0f, 0.0f, 0.0f, \
    0.0f, 1.0f, 0.0f, \
    0.0f, 0.0f, 1.0f, \
}
// clang-format on
// clang-format off
#define MAT4F_IDENTITY { \
    1.0f, 0.0f, 0.0f, 0.0f, \
    0.0f, 1.0f, 0.0f, 0.0f, \
    0.0f, 0.0f, 1.0f, 0.0f, \
    0.0f, 0.0f, 0.0f, 1.0f, \
}
// clang-format on
#define VEC3D_ZERO {0.0, 0.0, 0.0}
#define VEC4D_ZERO {0.0, 0.0, 0.0, 0.0}
#define QUATD_IDENTITY {0.0, 0.0, 0.0, 1.0}
// clang-format off
#define MAT3D_IDENTITY { \
    1.0, 0.0, 0.0, \
    0.0, 1.0, 0.0, \
    0.0, 0.0, 1.0, \
}
// clang-format on
// clang-format off
#define MAT4D_IDENTITY { \
    1.0, 0.0, 0.0, 0.0, \
    0.0, 1.0, 0.0, 0.0, \
    0.0, 0.0, 1.0, 0.0, \
    0.0, 0.0, 0.0, 1.0, \
}
// clang-format on

typedef float vec3f[3];
typedef float vec4f[4];
typedef float mat3f[9];
typedef float mat4f[16];

typedef double vec3d[3];
typedef double vec4d[4];
typedef double mat3d[9];
typedef double mat4d[16];
/* Quaternion with order (x,y,z,w) */
typedef vec4f quatf;
typedef vec4d quatd;

typedef struct rayf {
    vec3f position;
    vec3f direction;
} rayf;
typedef struct rayd {
    vec3d position;
    vec3d direction;
} rayd;

/* VEC3D */
void vec3f_zero(vec3f v);
void vec3f_add(const vec3f va, const vec3f vb, vec3f out);
void vec3f_sub(const vec3f va, const vec3f vb, vec3f out);
void vec3f_scale(const vec3f va, float s, vec3f out);
float vec3f_dot(const vec3f va, const vec3f vb);
float vec3f_length(const vec3f v);
int vec3f_normalize(vec3f v);
void vec3f_cross(const vec3f va, const vec3f vb, vec3f out);

void vec3d_zero(vec3d v);
void vec3d_add(const vec3d va, const vec3d vb, vec3d out);
void vec3d_sub(const vec3d va, const vec3d vb, vec3d out);
void vec3d_scale(const vec3d va, double s, vec3d out);
double vec3d_dot(const vec3d va, const vec3d vb);
double vec3d_length(const vec3d v);
int vec3d_normalize(vec3d v);
void vec3d_cross(const vec3d va, const vec3d vb, vec3d out);

/* VEC4D */
void vec4f_zero(vec4f v);
float vec4f_dot(const vec4f va, const vec4f vb);
float vec4f_length(const vec4f v);
int vec4f_normalize(vec4f v);

void vec4d_zero(vec4d v);
double vec4d_dot(const vec4d va, const vec4d vb);
double vec4d_length(const vec4d v);
int vec4d_normalize(vec4d v);

/* QUATD */
void quatf_identity(quatf q);
int quatf_normalize(quatf q);
void quatf_to_mat3f(const quatf q, mat3f m);
void quatf_from_mat3f(const mat3f m, quatf q);
void quatf_slerp(const quatf start, const quatf end, float t, quatf out);

void quatd_identity(quatd q);
int quatd_normalize(quatd q);
void quatd_to_mat3d(const quatd q, mat3d m);
void quatd_from_mat3d(const mat3d m, quatd q);
void quatd_slerp(const quatd start, const quatd end, double t, quatd out);

/* MAT3D */
void mat3f_identity(mat3f m);
float mat3f_det(const mat3f m);
int mat3f_inverse(const mat3f m, mat3f out);
void mat3f_transpose(const mat3f m, mat3f out);
void mat3f_from_quatf(const quatf q, mat3f m);
void mat3f_mul_vec3f(const mat3f m, const vec3f v, vec3f out);
int mat3f_orthonormalize(const mat3f m, mat3f out);
void mat3f_to_quatf(const mat3f m, quatf q);
void mat3f_mul(const mat3f ma, const mat3f mb, mat3f out);
#ifdef GLMATH_ENABLE_LAPACK
int mat3f_svd(const mat3f m, mat3f u, vec3f singular_values, mat3f vt);
int mat3f_project_so3(const mat3f m, mat3f out);
#endif

void mat3d_identity(mat3d m);
double mat3d_det(const mat3d m);
int mat3d_inverse(const mat3d m, mat3d out);
void mat3d_transpose(const mat3d m, mat3d out);
void mat3d_from_quatd(const quatd q, mat3d m);
int mat3d_from_rotation_vector(const vec3d omega, mat3d out);
void mat3d_mul_vec3d(const mat3d m, const vec3d v, vec3d out);
int mat3d_orthonormalize(const mat3d m, mat3d out);
void mat3d_to_quatd(const mat3d m, quatd q);
void mat3d_mul(const mat3d ma, const mat3d mb, mat3d out);
#ifdef GLMATH_ENABLE_LAPACK
int mat3d_svd(const mat3d m, mat3d u, vec3d singular_values, mat3d vt);
int mat3d_project_so3(const mat3d m, mat3d out);
#endif

/* MAT4D */
void mat4f_identity(mat4f m);
void mat4f_transpose(const mat4f m, mat4f out);
void mat4f_mul(const mat4f ma, const mat4f mb, mat4f out);
void mat4f_mul_vec4f(const mat4f m, const vec4f v, vec4f out);
void mat4f_inverse_rigid(const mat4f m, mat4f out);
int mat4f_look_dir(
    const vec3f eye, const vec3f direction, const vec3f up, mat4f m
);
int mat4f_look_at(
    const vec3f eye, const vec3f target, const vec3f up, mat4f out
);
int mat4f_orthonormalize(const mat4f m, mat4f out);

void mat4d_identity(mat4d m);
void mat4d_transpose(const mat4d m, mat4d out);
void mat4d_mul(const mat4d ma, const mat4d mb, mat4d out);
void mat4d_mul_vec4d(const mat4d m, const vec4d v, vec4d out);
void mat4d_inverse_rigid(const mat4d m, mat4d out);
int mat4d_look_dir(
    const vec3d eye, const vec3d direction, const vec3d up, mat4d m
);
int mat4d_look_at(
    const vec3d eye, const vec3d target, const vec3d up, mat4d out
);
int mat4d_orthonormalize(const mat4d m, mat4d out);
#ifdef GLMATH_ENABLE_LAPACK
int mat4f_inverse_cholesky(const mat4f m, mat4f out);
int mat4d_inverse_cholesky(const mat4d m, mat4d out);
#endif

#endif /* GLMATH_H */
