#ifndef GLMATH_H
#define GLMATH_H

#include <math.h>
#include <stdint.h>
#include <string.h>

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

static const vec3f VEC3F_ZERO = {0.0, 0.0, 0.0};
static const vec4f VEC4F_ZERO = {0.0, 0.0, 0.0, 0.0};
static const quatf QUATF_UNIT = {0.0, 0.0, 0.0, 1.0};
static const mat3f MAT3F_IDENTITY = {
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0
};
static const mat4f MAT4F_IDENTITY = {
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0
};
static const vec3d VEC3D_ZERO = {0.0, 0.0, 0.0};
static const vec4d VEC4D_ZERO = {0.0, 0.0, 0.0, 0.0};
static const quatd QUATD_UNIT = {0.0, 0.0, 0.0, 1.0};
static const mat3d MAT3D_IDENTITY = {
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0
};
static const mat4d MAT4D_IDENTITY = {
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0
};

/* VEC3D */
void vec3f_add(const vec3f va, const vec3f vb, vec3f out);
void vec3f_sub(const vec3f va, const vec3f vb, vec3f out);
void vec3f_scale(const vec3f va, float s, vec3f out);
float vec3f_dot(const vec3f va, const vec3f vb);
float vec3f_length(const vec3f v);
int vec3f_normalize(vec3f v);
void vec3f_cross(const vec3f va, const vec3f vb, vec3f out);

void vec3d_add(const vec3d va, const vec3d vb, vec3d out);
void vec3d_sub(const vec3d va, const vec3d vb, vec3d out);
void vec3d_scale(const vec3d va, double s, vec3d out);
double vec3d_dot(const vec3d va, const vec3d vb);
double vec3d_length(const vec3d v);
int vec3d_normalize(vec3d v);
void vec3d_cross(const vec3d va, const vec3d vb, vec3d out);

/* VEC4D */
float vec4f_dot(const vec4f va, const vec4f vb);
float vec4f_length(const vec4f v);
int vec4f_normalize(vec4f v);

double vec4d_dot(const vec4d va, const vec4d vb);
double vec4d_length(const vec4d v);
int vec4d_normalize(vec4d v);

/* QUATD */
int quatf_normalize(quatf q);
void quatf_to_mat3f(const quatf q, mat3f m);
void quatf_from_mat3f(const mat3f m, quatf q);

int quatd_normalize(quatd q);
void quatd_to_mat3d(const quatd q, mat3d m);
void quatd_from_mat3d(const mat3d m, quatd q);

/* MAT3D */
float mat3f_det(const mat3f m);
void mat3f_transpose(const mat3f m, mat3f out);
void mat3f_from_quatf(const quatf q, mat3f m);
void mat3f_mul_vec3f(const mat3f m, const vec3f v, vec3f out);
int mat3f_orthonormalize(const mat3f m, mat3f out);
void mat3f_to_quatf(const mat3f m, quatf q);
void mat3f_mul(const mat3f ma, const mat3f mb, mat3f out);

double mat3d_det(const mat3d m);
void mat3d_transpose(const mat3d m, mat3d out);
void mat3d_from_quatd(const quatd q, mat3d m);
void mat3d_mul_vec3d(const mat3d m, const vec3d v, vec3d out);
int mat3d_orthonormalize(const mat3d m, mat3d out);
void mat3d_to_quatd(const mat3d m, quatd q);
void mat3d_mul(const mat3d ma, const mat3d mb, mat3d out);
#ifdef GLMATH_ENABLE_LAPACK
int mat3f_project_so3(const mat3f m, mat3f out);
int mat3d_project_so3(const mat3d m, mat3d out);
#endif

/* MAT4D */
void mat4f_transpose(const mat4f m, mat4f out);
void mat4f_mul_vec4f(const mat4f m, const vec4f v, vec4f out);
void mat4f_inverse_rigid(const mat4f m, mat4f out);
int mat4f_look_dir(
    const vec3f eye, const vec3f direction, const vec3f up, mat4f m
);
int mat4f_look_at(
    const vec3f eye, const vec3f target, const vec3f up, mat4f out
);
int mat4f_orthonormalize(const mat4f m, mat4f out);

void mat4d_transpose(const mat4d m, mat4d out);
void mat4d_mul_vec4d(const mat4d m, const vec4d v, vec4d out);
void mat4d_inverse_rigid(const mat4d m, mat4d out);
int mat4d_look_dir(
    const vec3d eye, const vec3d direction, const vec3d up, mat4d m
);
int mat4d_look_at(
    const vec3d eye, const vec3d target, const vec3d up, mat4d out
);
int mat4d_orthonormalize(const mat4d m, mat4d out);

#endif /* GLMATH_H */
