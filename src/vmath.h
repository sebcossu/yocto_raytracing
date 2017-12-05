//
// LICENSE:
//
// Copyright (c) 2016 -- 2017 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef _VMATH_H_
#define _VMATH_H_

#include <cassert>
#include <cfloat>
#include <cmath>
#include <random>

struct vec2f {
    float x, y;
};

struct vec3f {
    float x, y, z;
};

struct vec4f {
    float x, y, z, w;
};

struct vec2i {
    int x, y;
};

struct vec3i {
    int x, y, z;
};

struct vec4i {
    int x, y, z, w;
};

struct vec4b {
    unsigned char x, y, z, w;
};

inline vec2f operator+(const vec2f& a, const vec2f& b) {
    return {a.x + b.x, a.y + b.y};
}

inline vec2f operator*(const vec2f& a, float b) { return {a.x * b, a.y * b}; }

inline vec3f operator-(const vec3f& a) { return {-a.x, -a.y, -a.z}; }

inline vec3f operator+(const vec3f& a, const vec3f& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline vec3f operator-(const vec3f& a, const vec3f& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline vec3f operator*(const vec3f& a, float b) {
    return {a.x * b, a.y * b, a.z * b};
}

inline vec3f operator/(const vec3f& a, float b) {
    return {a.x / b, a.y / b, a.z / b};
}

inline vec3f operator*(const vec3f& a, const vec3f& b) {
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

inline vec3f& operator*=(vec3f& a, const vec3f& b) { return a = a * b; }

inline vec4f operator+(const vec4f& a, const vec4f& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

inline vec4f operator*(const vec4f& a, float b) {
    return {a.x * b, a.y * b, a.z * b, a.w * b};
}

inline vec3f& operator+=(vec3f& a, const vec3f& b) { return a = a + b; }
inline vec3f& operator-=(vec3f& a, const vec3f& b) { return a = a - b; }
inline vec3f& operator*=(vec3f& a, float b) { return a = a * b; }

inline vec4f& operator+=(vec4f& a, const vec4f& b) { return a = a + b; }
inline vec4f& operator*=(vec4f& a, float b) { return a = a * b; }

inline bool operator==(const vec3f& a, const vec3f& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool operator!=(const vec3f& a, const vec3f& b) { return !(a == b); }

inline float dot(const vec3f& a, const vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float length(const vec3f& a) { return sqrtf(dot(a, a)); }

inline vec3f normalize(const vec3f& a) {
    auto l = length(a);
    if (l == 0) return a;
    return a * (1 / l);
}

inline vec3f cross(const vec3f& a, const vec3f& b) {
    return {
        a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct mat3f {
    vec3f x, y, z;
};

inline vec3f operator*(const mat3f& a, const vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline mat3f operator*(const mat3f& a, const mat3f& b) {
    return {a * b.x, a * b.y, a * b.z};
}

inline mat3f operator*(const mat3f& a, float b) {
    return {a.x * b, a.y * b, a.z * b};
}

struct frame3f {
    vec3f x, y, z, o;
};

const auto identity_frame3f =
    frame3f{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}};

inline vec3f transform_point(const frame3f& a, const vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.o;
}

inline vec3f transform_point_inverse(const frame3f& a, const vec3f& b) {
    auto bo = b - a.o;
    return {dot(a.x, bo), dot(a.y, bo), dot(a.z, bo)};
}

inline vec3f transform_vector(const frame3f& a, const vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline vec3f transform_vector_inverse(const frame3f& a, const vec3f& b) {
    return {dot(a.x, b), dot(a.y, b), dot(a.z, b)};
}

inline vec3f transform_direction(const frame3f& a, const vec3f& b) {
    return normalize(transform_vector(a, b));
}

inline vec3f transform_direction_inverse(const frame3f& a, const vec3f& b) {
    return normalize(transform_vector_inverse(a, b));
}

inline frame3f invert_frame(const frame3f& a) {
    auto mt = mat3f{
        {a.x.x, a.y.x, a.z.x}, {a.x.y, a.y.y, a.z.y}, {a.x.z, a.y.z, a.z.z}};
    return {mt.x, mt.y, mt.z, -(mt * a.o)};
}

inline vec3f orthonormalize(const vec3f& a, const vec3f& b) {
    return normalize(a - b * dot(a, b));
}

inline frame3f make_frame3_fromzx(
    const vec3f& o, const vec3f& z_, const vec3f& x_) {
    auto z = normalize(z_);
    auto x = orthonormalize(x_, z);
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

inline vec3f orthogonal(const vec3f& v) {
    return fabsf(v.x) > fabsf(v.z) ? vec3f{-v.y, v.x, 0} : vec3f{0, -v.z, v.y};
}

inline frame3f make_frame3_fromz(const vec3f& o, const vec3f& z_) {
    auto z = normalize(z_);
    auto x = normalize(orthogonal(z));
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

inline frame3f lookat_frame3f(const vec3f& o, const vec3f& c, const vec3f& u) {
    auto z = normalize(o - c);
    auto x = normalize(cross(u, z));
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

const auto pif = 3.141592653f;

inline float min(float x, float y) { return (x < y) ? x : y; }
inline float max(float x, float y) { return (x > y) ? x : y; }
inline float clamp(float x, float a, float b) { return min(max(x, a), b); }

inline int min(int x, int y) { return (x < y) ? x : y; }
inline int max(int x, int y) { return (x > y) ? x : y; }
inline int clamp(int x, int a, int b) { return min(max(x, a), b); }

inline float max_element(const vec3f& a) { return max(a.x, max(a.y, a.z)); }

inline vec3f triangle_normal(
    const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    return normalize(cross(v1 - v0, v2 - v0));
}

inline float triangle_area(const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    return length(cross(v1 - v0, v2 - v0)) / 2;
}

inline vec3f line_tangent(const vec3f& v0, const vec3f& v1) {
    return normalize(v1 - v0);
}

inline float line_length(const vec3f& v0, const vec3f& v1) {
    return length(v1 - v0);
}

inline float tetrahedron_volume(
    const vec3f& v0, const vec3f& v1, const vec3f& v2, const vec3f& v3) {
    return dot(cross(v1 - v0, v2 - v0), v3 - v0) / 6;
}

using rng_t = std::minstd_rand;

// generates a float in the [0,1) range
inline float randf(rng_t* rng) {
    auto dist = std::uniform_real_distribution<float>(0, 1);
    return std::min(dist(*rng), 1.0f - FLT_EPSILON);
}

// initializes nrngs independent random number generators
inline std::vector<rng_t> seed_rngs(int nrngs) {
    auto seeds = std::vector<uint32_t>(nrngs);
    std::seed_seq().generate(seeds.begin(), seeds.end());
    auto rngs = std::vector<rng_t>(nrngs);
    for (auto i = 0; i < nrngs; i++) rngs[i] = rng_t(seeds[i]);
    return rngs;
}

const float ray_eps = 1e-4f;

struct ray3f {
    vec3f o = {0, 0, 0};
    vec3f d = {0, 0, 1};
    float tmin = ray_eps;
    float tmax = FLT_MAX;
};

inline vec3f eval_ray(const ray3f& ray, float t) { return ray.o + ray.d * t; }

inline ray3f transform_ray_inverse(const frame3f& frame, const ray3f& ray) {
    return {transform_point_inverse(frame, ray.o),
        transform_direction_inverse(frame, ray.d), ray.tmin, ray.tmax};
}

struct bbox3f {
    vec3f min, max;
};

const auto invalid_bbox3f =
    bbox3f{{FLT_MAX, FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX, -FLT_MAX}};

inline bbox3f expand_bbox(const bbox3f& a, const vec3f& b) {
    return {{min(a.min.x, b.x), min(a.min.y, b.y), min(a.min.z, b.z)},
        {max(a.max.x, b.x), max(a.max.y, b.y), max(a.max.z, b.z)}};
}

inline bbox3f expand_bbox(const bbox3f& a, const bbox3f& b) {
    return {
        {min(a.min.x, b.min.x), min(a.min.y, b.min.y), min(a.min.z, b.min.z)},
        {max(a.max.x, b.max.x), max(a.max.y, b.max.y), max(a.max.z, b.max.z)}};
}

inline bool contain_bbox(const bbox3f& a, const vec3f& b) {
    if (b.x < a.min.x || b.x > a.max.x) return false;
    if (b.y < a.min.y || b.y > a.max.y) return false;
    if (b.z < a.min.z || b.z > a.max.z) return false;
    return true;
}

inline bool overlap_bbox(const bbox3f& bbox1, const bbox3f& bbox2) {
    if (bbox1.max.x < bbox2.min.x || bbox1.min.x > bbox2.max.x) return false;
    if (bbox1.max.y < bbox2.min.y || bbox1.min.y > bbox2.max.y) return false;
    if (bbox1.max.z < bbox2.min.z || bbox1.min.z > bbox2.max.z) return false;
    return true;
}

inline bbox3f bbox_to_world(const frame3f& a, const bbox3f& b) {
    vec3f corners[8] = {
        {b.min.x, b.min.y, b.min.z},
        {b.min.x, b.min.y, b.max.z},
        {b.min.x, b.max.y, b.min.z},
        {b.min.x, b.max.y, b.max.z},
        {b.max.x, b.min.y, b.min.z},
        {b.max.x, b.min.y, b.max.z},
        {b.max.x, b.max.y, b.min.z},
        {b.max.x, b.max.y, b.max.z},
    };
    auto bbox = invalid_bbox3f;
    for (auto p : corners) bbox = expand_bbox(bbox, transform_point(a, p));
    return bbox;
}

#endif
