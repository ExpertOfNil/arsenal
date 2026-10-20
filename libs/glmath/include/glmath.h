#ifndef GLMATH_H
#define GLMATH_H

#include <math.h>
#include <stdint.h>
#include <string.h>

typedef double vec3d[3];
typedef double vec4d[4];
typedef double mat3d[9];
typedef double mat4d[16];
/* Quaternion with order (x,y,z,w) */
typedef vec4d quatd;

typedef struct rayd {
    vec3d position;
    vec3d direction;
} rayd;

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
void vec3d_add(const vec3d va, const vec3d vb, vec3d out);
void vec3d_sub(const vec3d va, const vec3d vb, vec3d out);
void vec3d_scale(const vec3d va, double s, vec3d out);
double vec3d_dot(const vec3d va, const vec3d vb);
double vec3d_length(const vec3d v);
int vec3d_normalize(vec3d v);
void vec3d_cross(const vec3d va, const vec3d vb, vec3d out);

/* VEC4D */
double vec4d_dot(const vec4d va, const vec4d vb);
double vec4d_length(const vec4d v);
int vec4d_normalize(vec4d v);

/* QUATD */
int quatd_normalize(quatd q);
void quatd_to_mat3d(const quatd q, mat3d m);
void quatd_from_mat3d(const mat3d m, quatd q);

/* MAT3D */
double mat3d_det(const mat3d m);
void mat3d_transpose(const mat3d m, mat3d out);
void mat3d_from_quatd(const quatd q, mat3d m);
void mat3d_mul_vec3d(const mat3d m, const vec3d v, vec3d out);
int mat3d_orthonormalize(const mat3d m, mat3d out);
void mat3d_to_quat(const mat3d m, quatd q);
void mat3d_mul(const mat3d ma, const mat3d mb, mat3d out);
int mat3d_project_so3(const mat3d m, mat3d out);

/* MAT4D */
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
