///
/// # Yocto/Math
///
/// A collection of vector math functions and simple containers
/// used to implement YOCTO. Features include
///
/// - a few convenience math functions
/// - static length float vectors, with specialization for 2, 3, 4 length
/// - static length matrices, with specialization for 2x2, 3x3, 4x4
/// - static length rigid transforms (frames), specialized for 2d and 3d space
/// - linear algebra operations and transforms for fixed length matrices/vecs
/// - axis aligned bounding boxes
/// - rays
/// - ray-primitive intersection
/// - point-primitive distance and overlap tests
/// - normal amd tangent computation for meshes and lines
/// - generation of tesselated meshes
/// - random number generation via PCG32
/// - a few hash functions
/// - trivial image data structue and a few image operations
/// - BVH for intersection and closest point query
///
/// We developed our own library since we felt that all existing ones are either
/// complete, but unreadable or with lots of dependencies, or just as incomplete
/// and untested as ours.
///
/// This library has no dependencies.
///
/// Notes: Quads are internally handled as a pair of two triangles v0,v1,v3 and
/// v2,v3,v1, with the u/v coordinates of the second triangle corrected as 1-u
/// and 1-v to produce a quad parametrization where u and v go from 0 to 1. This
/// is equivalent to Intel's Embree.
///
///
/// This library includes code from the PCG random number generator,
/// boost hash_combine, Pixar multijittered sampling, code from "Real-Time
/// Collision Detection" by Christer Ericson and public domain code from
/// - https://github.com/sgorsten/linalg
/// - https://gist.github.com/badboy/6267743
///
///
/// ## History
///
/// - v 0.28: experimental support for quad intersect and sampling
/// - v 0.27: BVH for intersection and closest point queries
/// - v 0.26: fix small compiler bugs
/// - v 0.25: geodesic sphere and surface faceting
/// - v 0.24: tesselation function
/// - v 0.23: more camera navigation
/// - v 0.22: removed image lookup with arbitrary channels
/// - v 0.21: added more functions
/// - v 0.20: remove unused bbox overlap tests
/// - v 0.19: remove indexing from specializations
/// - v 0.18: bump to normal mapping convertion
/// - v 0.17: added example image geneation
/// - v 0.16: sampling
/// - v 0.15: enable specialization always
/// - v 0.14: move timer to Yocto/Utils
/// - v 0.13: more shape functions
/// - v 0.12: documentation update
/// - v 0.11: added more matrix and quaternion operations
/// - v 0.10: specialize some type and functions
/// - v 0.9: bbox containment tests
/// - v 0.8: remove std:array as base class for better control
/// - v 0.7: doxygen comments
/// - v 0.6: uniformed internal names
/// - v 0.5: simplification of constructors, raname bbox -> bbox
/// - v 0.4: overall type simplification
/// - v 0.3: internal C++ refactoring
/// - v 0.2: use of STL containers; removal of yocto containers
/// - v 0.1: C++ only implementation
/// - v 0.0: initial release in C99
///
///
namespace ym {}

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

//
//  LICENSE of included software
//
// This code also includes a small exerpt from http://www.pcg-random.org/
// licensed as follows
// *Really* minimal PCG32 code / (c) 2014 M.E. O'Neill / pcg-random.org
// Licensed under Apache License 2.0 (NO WARRANTY, etc. see website)
//

#ifndef _YMATH_H_
#define _YMATH_H_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <limits>
#include <unordered_map>
#include <vector>

// HACK to avoid compilation with MSVC2015 and C++11 without dirtying code
#if defined(_WIN32) || __cplusplus < 201402L
#define constexpr
#endif

//
// Compilation option
//
#define YM_FAST_RANDFLOAT 1

///
/// Math types and utlities for 3D graphics and imaging
///
namespace ym {

// -----------------------------------------------------------------------------
// BASIC TYPEDEFS
// -----------------------------------------------------------------------------

/// convenient typedef for bytes
using byte = unsigned char;

/// convenient typedef for bytes
using uint = unsigned int;

// -----------------------------------------------------------------------------
// MATH CONSTANTS
// -----------------------------------------------------------------------------

/// pi (float)
constexpr const auto pif = 3.14159265f;
/// pi (double)
constexpr const auto pi = 3.1415926535897932384626433832795;

/// shortcat for float max value
constexpr const auto flt_max = std::numeric_limits<float>::max();
/// shortcat for float min value
constexpr const auto flt_min = std::numeric_limits<float>::lowest();
/// shortcat for int max value
constexpr const auto int_max = std::numeric_limits<int>::max();
/// shortcat for int min value
constexpr const auto int_min = std::numeric_limits<int>::min();

// -----------------------------------------------------------------------------
// BASIC MATH FUNCTIONS
// -----------------------------------------------------------------------------

/// Safe minimum value.
template <typename T>
constexpr inline T min(T x, T y) {
    return (x < y) ? x : y;
}

/// Safe maximum value.
template <typename T>
constexpr inline T max(T x, T y) {
    return (x > y) ? x : y;
}

/// Clamp a value between a minimum and a maximum.
template <typename T>
constexpr inline T clamp(T x, T min_, T max_) {
    return min(max(x, min_), max_);
}

/// Linear interpolation.
template <typename T>
constexpr inline T lerp(T a, T b, T t) {
    return a * (1 - t) + b * t;
}

/// Integer power of two
constexpr inline int pow2(int x) { return 1 << x; }

/// Safe float to byte conversion
constexpr inline byte float_to_byte(float x) {
    return (byte)max(0, min(int(x * 256), 255));
}

/// Safe byte to float conversion
constexpr inline float byte_to_float(byte x) { return (float)x / 255.0f; }

// -----------------------------------------------------------------------------
// FUNCTIONS BROUGHT INTO NAMESPACE
// -----------------------------------------------------------------------------

/// sqrt
using std::sqrt;
/// pow
using std::pow;
/// pow
using std::exp;
/// log
using std::log;
/// log10
using std::log10;
/// sin
using std::sin;
/// cos
using std::cos;
/// tan
using std::tan;
/// asin
using std::asin;
/// acos
using std::acos;
/// atan2
using std::atan2;
/// abs
using std::abs;
/// floor
using std::floor;
/// ceil
using std::ceil;
/// round
using std::round;
/// isfinate
using std::isfinite;

// -----------------------------------------------------------------------------
// VECTORS
// -----------------------------------------------------------------------------

///
/// Vector of elements of compile time dimension with default initializer.
///
template <typename T, int N>
struct vec {
    /// default constructor
    constexpr vec() {
        for (auto i = 0; i < N; i++) v[i] = 0;
    }
    /// element constructor
    constexpr explicit vec(T vv) {
        for (auto i = 0; i < N; i++) v[i] = vv;
    }
    /// list constructor
    constexpr vec(const std::initializer_list<T>& vv) {
        assert(N == vv.size());
        auto i = 0;
        for (auto&& e : vv) v[i++] = e;
    }

    /// element access
    constexpr T& operator[](int i) { return v[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return v[i]; }

    /// data access
    constexpr T* data() { return v; }
    /// data access
    constexpr const T* data() const { return v; }

    /// element data
    T v[N];
};

///
/// Specialization of vectors for 1 component and float coordinates.
///
template <typename T>
struct vec<T, 1> {
    /// size
    constexpr static const int N = 1;

    /// default constructor
    constexpr vec() : x{0} {}
    /// element constructor
    constexpr vec(T x) : x{x} {}

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// element data
    T x;
};

///
/// Specialization of vectors for 2 components and float coordinates.
///
template <typename T>
struct vec<T, 2> {
    /// size
    constexpr static const int N = 2;

    /// default constructor
    constexpr vec() : x{0}, y{0} {}
    /// element constructor
    constexpr explicit vec(T vv) : x(vv), y(vv) {}
    /// element constructor
    constexpr vec(T x, T y) : x{x}, y{y} {}

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// element data
    T x;
    /// element data
    T y;
};

///
/// Specialization of vectors for 3 components and float coordinates.
///
template <typename T>
struct vec<T, 3> {
    /// size
    constexpr static const int N = 3;

    /// default constructor
    constexpr vec() : x{0}, y{0}, z{0} {}
    /// element constructor
    constexpr explicit vec(T vv) : x(vv), y(vv), z(vv) {}
    /// element constructor
    constexpr vec(T x, T y, T z) : x{x}, y{y}, z{z} {}

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// element data
    T x;
    /// element data
    T y;
    /// element data
    T z;
};

///
/// Specialization of vectors for 4 components and float coordinates.
///
template <typename T>
struct vec<T, 4> {
    /// size
    constexpr static const int N = 4;

    /// default constructor
    constexpr vec() : x{0}, y{0}, z{0}, w{0} {}
    /// element constructor
    constexpr explicit vec(T vv) : x(vv), y(vv), z(vv), w(vv) {}
    /// element constructor
    constexpr vec(T x, T y, T z, T w) : x{x}, y{y}, z{z}, w{w} {}
    /// constructor from smaller vector
    constexpr vec(const vec<T, 3>& xyz, T w)
        : x{xyz.x}, y{xyz.y}, z{xyz.z}, w{w} {}

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// access xyz components
    constexpr vec<T, 3>& xyz() { return *(vec<T, 3>*)&x; }
    /// access xyz components
    constexpr const vec<T, 3>& xyz() const { return *(vec<T, 3>*)&x; }

    /// element data
    T x;
    /// element data
    T y;
    /// element data
    T z;
    /// element data
    T w;
};

/// 1-dimensional float vector
using vec1f = vec<float, 1>;
/// 2-dimensional float vector
using vec2f = vec<float, 2>;
/// 3-dimensional float vector
using vec3f = vec<float, 3>;
/// 4-dimensional float vector
using vec4f = vec<float, 4>;

/// 1-dimensional int vector
using vec1i = vec<int, 1>;
/// 2-dimensional int vector
using vec2i = vec<int, 2>;
/// 3-dimensional int vector
using vec3i = vec<int, 3>;
/// 4-dimensional int vector
using vec4i = vec<int, 4>;

/// 1-dimensional byte vector
using vec1b = vec<byte, 1>;
/// 2-dimensional byte vector
using vec2b = vec<byte, 2>;
/// 3-dimensional byte vector
using vec3b = vec<byte, 3>;
/// 4-dimensional byte vector
using vec4b = vec<byte, 4>;

/// 1-dimensional float zero vector
const auto zero1f = vec<float, 1>();
/// 2-dimensional float zero vector
const auto zero2f = vec<float, 2>();
/// 3-dimensional float zero vector
const auto zero3f = vec<float, 3>();
/// 4-dimensional float zero vector
const auto zero4f = vec<float, 4>();

/// 1-dimensional float one vector
const auto one1f = vec<float, 1>(1);
/// 2-dimensional float one vector
const auto one2f = vec<float, 2>(1);
/// 3-dimensional float one vector
const auto one3f = vec<float, 3>(1);
/// 4-dimensional float one vector
const auto one4f = vec<float, 4>(1);

/// 1-dimensional int zero vector
const auto zero1i = vec<int, 1>();
/// 2-dimensional int zero vector
const auto zero2i = vec<int, 2>();
/// 3-dimensional int zero vector
const auto zero3i = vec<int, 3>();
/// 4-dimensional int zero vector
const auto zero4i = vec<int, 4>();

/// 1-dimensional byte zero vector
const auto zero1b = vec<byte, 1>();
/// 2-dimensional byte zero vector
const auto zero2b = vec<byte, 2>();
/// 3-dimensional byte zero vector
const auto zero3b = vec<byte, 3>();
/// 4-dimensional byte zero vector
const auto zero4b = vec<byte, 4>();

/// iteration support
template <typename T, int N>
constexpr inline T* begin(vec<T, N>& a) {
    return &a[0];
}

/// iteration support
template <typename T, int N>
constexpr inline const T* begin(const vec<T, N>& a) {
    return &a[0];
}

/// iteration support
template <typename T, int N>
constexpr inline T* end(vec<T, N>& a) {
    return &a[0] + N;
}

/// iteration support
template <typename T, int N>
constexpr inline const T* end(const vec<T, N>& a) {
    return &a[0] + N;
}

/// vector operator ==
template <typename T, int N>
constexpr inline bool operator==(const vec<T, N>& a, const vec<T, N>& b) {
    for (auto i = 0; i < N; i++)
        if (a[i] != b[i]) return false;
    return true;
}

/// vector operator !=
template <typename T, int N>
constexpr inline bool operator!=(const vec<T, N>& a, const vec<T, N>& b) {
    return !(a == b);
}

/// vector operator < (lexicographic order - useful for std::map)
template <typename T, int N>
constexpr inline bool operator<(const vec<T, N>& a, const vec<T, N>& b) {
    for (auto i = 0; i < N; i++) {
        if (a[i] < b[i]) return true;
        if (a[i] > b[i]) return false;
    }
    return false;
}

/// vector operator ==
template <>
constexpr inline bool operator==(const vec2f& a, const vec2f& b) {
    return a.x == b.x && a.y == b.y;
}

/// vector operator !=
template <>
constexpr inline bool operator!=(const vec2f& a, const vec2f& b) {
    return a.x != b.x || a.y != b.y;
}

/// vector operator ==
template <>
constexpr inline bool operator==(const vec3f& a, const vec3f& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

/// vector operator !=
template <>
constexpr inline bool operator!=(const vec3f& a, const vec3f& b) {
    return a.x != b.x || a.y != b.y || a.z != b.z;
}

/// vector operator ==
template <>
constexpr inline bool operator==(const vec4f& a, const vec4f& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

/// vector operator !=
template <>
constexpr inline bool operator!=(const vec4f& a, const vec4f& b) {
    return a.x != b.x || a.y != b.y || a.z != b.z || a.w != b.w;
}

/// vector operator +
template <typename T, int N>
constexpr inline vec<T, N> operator+(const vec<T, N>& a) {
    return a;
}

/// vector operator -
template <typename T, int N>
constexpr inline vec<T, N> operator-(const vec<T, N>& a) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = -a[i];
    return c;
}

/// vector operator +
template <typename T, int N>
constexpr inline vec<T, N> operator+(const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] + b[i];
    return c;
}

/// vector operator -
template <typename T, int N>
constexpr inline vec<T, N> operator-(const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] - b[i];
    return c;
}

/// vector operator +
template <typename T, int N>
constexpr inline vec<T, N> operator+(const vec<T, N>& a, const T b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] + b;
    return c;
}

/// vector operator -
template <typename T, int N>
constexpr inline vec<T, N> operator-(const vec<T, N>& a, const T b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] - b;
    return c;
}

/// vector operator +
template <typename T, int N>
constexpr inline vec<T, N> operator+(T a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a + b[i];
    return c;
}

/// vector operator -
template <typename T, int N>
constexpr inline vec<T, N> operator-(T a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a - b[i];
    return c;
}

/// vector operator *
template <typename T, int N>
constexpr inline vec<T, N> operator*(const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] * b[i];
    return c;
}

/// vector operator *
template <typename T, int N>
constexpr inline vec<T, N> operator*(const vec<T, N>& a, const T b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] * b;
    return c;
}

/// vector operator *
template <typename T, int N>
constexpr inline vec<T, N> operator*(const T a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a * b[i];
    return c;
}

/// vector operator /
template <typename T, int N>
constexpr inline vec<T, N> operator/(const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] / b[i];
    return c;
}

/// vector operator /
template <typename T, int N>
constexpr inline vec<T, N> operator/(const vec<T, N>& a, const T b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] / b;
    return c;
}

/// vector operator /
template <typename T, int N>
constexpr inline vec<T, N> operator/(const T a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a / b[i];
    return c;
}

/// vector operator +
template <>
constexpr inline vec2f operator+(const vec2f& a) {
    return a;
}

/// vector operator -
template <>
constexpr inline vec2f operator-(const vec2f& a) {
    return {-a.x, -a.y};
}

/// vector operator +
template <>
constexpr inline vec2f operator+(const vec2f& a, const vec2f& b) {
    return {a.x + b.x, a.y + b.y};
}

/// vector operator -
template <>
constexpr inline vec2f operator-(const vec2f& a, const vec2f& b) {
    return {a.x - b.x, a.y - b.y};
}

/// vector operator *
template <>
constexpr inline vec2f operator*(const vec2f& a, const vec2f& b) {
    return {a.x * b.x, a.y * b.y};
}

/// vector operator *
template <>
constexpr inline vec2f operator*(const vec2f& a, const float b) {
    return {a.x * b, a.y * b};
}

/// vector operator *
template <>
constexpr inline vec2f operator*(const float a, const vec2f& b) {
    return {a * b.x, a * b.y};
}

/// vector operator /
template <>
constexpr inline vec2f operator/(const vec2f& a, const vec2f& b) {
    return {a.x / b.x, a.y / b.y};
}

/// vector operator /
template <>
constexpr inline vec2f operator/(const vec2f& a, const float b) {
    return {a.x / b, a.y / b};
}

/// vector operator /
template <>
constexpr inline vec2f operator/(const float a, const vec2f& b) {
    return {a / b.x, a / b.y};
}

/// vector operator +
template <>
constexpr inline vec3f operator+(const vec3f& a) {
    return a;
}

/// vector operator -
template <>
constexpr inline vec3f operator-(const vec3f& a) {
    return {-a.x, -a.y, -a.z};
}

/// vector operator +
template <>
constexpr inline vec3f operator+(const vec3f& a, const vec3f& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

/// vector operator -
template <>
constexpr inline vec3f operator-(const vec3f& a, const vec3f& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

/// vector operator *
template <>
constexpr inline vec3f operator*(const vec3f& a, const vec3f& b) {
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

/// vector operator *
template <>
constexpr inline vec3f operator*(const vec3f& a, const float b) {
    return {a.x * b, a.y * b, a.z * b};
}

/// vector operator *
template <>
constexpr inline vec3f operator*(const float a, const vec3f& b) {
    return {a * b.x, a * b.y, a * b.z};
}

/// vector operator /
template <>
constexpr inline vec3f operator/(const vec3f& a, const vec3f& b) {
    return {a.x / b.x, a.y / b.y, a.z / b.z};
}

/// vector operator /
template <>
constexpr inline vec3f operator/(const vec3f& a, const float b) {
    return {a.x / b, a.y / b, a.z / b};
}

/// vector operator /
template <>
constexpr inline vec3f operator/(const float a, const vec3f& b) {
    return {a / b.x, a / b.y, a / b.z};
}

/// vector operator +
template <>
constexpr inline vec4f operator+(const vec4f& a) {
    return a;
}

/// vector operator -
template <>
constexpr inline vec4f operator-(const vec4f& a) {
    return {-a.x, -a.y, -a.z, -a.w};
}

/// vector operator +
template <>
constexpr inline vec4f operator+(const vec4f& a, const vec4f& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

/// vector operator -
template <>
constexpr inline vec4f operator-(const vec4f& a, const vec4f& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}

/// vector operator *
template <>
constexpr inline vec4f operator*(const vec4f& a, const vec4f& b) {
    return {a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w};
}

/// vector operator *
template <>
constexpr inline vec4f operator*(const vec4f& a, const float b) {
    return {a.x * b, a.y * b, a.z * b, a.w * b};
}

/// vector operator *
template <>
constexpr inline vec4f operator*(const float a, const vec4f& b) {
    return {a * b.x, a * b.y, a * b.z, a * b.w};
}

/// vector operator /
template <>
constexpr inline vec4f operator/(const vec4f& a, const vec4f& b) {
    return {a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w};
}

/// vector operator /
template <>
constexpr inline vec4f operator/(const vec4f& a, const float b) {
    return {a.x / b, a.y / b, a.z / b, a.w / b};
}

/// vector operator /
template <>
constexpr inline vec4f operator/(const float a, const vec4f& b) {
    return {a / b.x, a / b.y, a / b.z, a / b.w};
}

/// vector operator +=
template <typename T, int N>
constexpr inline vec<T, N>& operator+=(vec<T, N>& a, const vec<T, N>& b) {
    return a = a + b;
}

/// vector operator -=
template <typename T, int N>
constexpr inline vec<T, N>& operator-=(vec<T, N>& a, const vec<T, N>& b) {
    return a = a - b;
}

/// vector operator *=
template <typename T, int N>
constexpr inline vec<T, N>& operator*=(vec<T, N>& a, const vec<T, N>& b) {
    return a = a * b;
}

/// vector operator *=
template <typename T, int N>
constexpr inline vec<T, N>& operator*=(vec<T, N>& a, const T b) {
    return a = a * b;
}

/// vector operator /=
template <typename T, int N>
constexpr inline vec<T, N>& operator/=(vec<T, N>& a, const vec<T, N>& b) {
    return a = a / b;
}

/// vector operator /=
template <typename T, int N>
constexpr inline vec<T, N>& operator/=(vec<T, N>& a, const T b) {
    return a = a / b;
}

/// vector dot product
template <typename T, int N>
constexpr inline T dot(const vec<T, N>& a, const vec<T, N>& b) {
    auto c = T(0);
    for (auto i = 0; i < N; i++) c += a[i] * b[i];
    return c;
}

/// vector cross product (2d)
template <typename T>
constexpr inline T cross(const vec<T, 2>& a, const vec<T, 2>& b) {
    return a.x * b.y - a.y * b.x;
}

/// vector cross product (3d)
template <typename T>
constexpr inline vec<T, 3> cross(const vec<T, 3>& a, const vec<T, 3>& b) {
    return {
        a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// vector dot product
template <>
constexpr inline float dot(const vec2f& a, const vec2f& b) {
    return a.x * b.x + a.y * b.y;
}

/// vector dot product
template <>
constexpr inline float dot(const vec3f& a, const vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// vector dot product
template <>
constexpr inline float dot(const vec4f& a, const vec4f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

/// vector cross product (2d)
template <>
constexpr inline float cross(const vec2f& a, const vec2f& b) {
    return a.x * b.y - a.y * b.x;
}

/// vector cross product (3d)
template <>
constexpr inline vec3f cross(const vec3f& a, const vec3f& b) {
    return {
        a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// vector length
template <typename T, int N>
constexpr inline T length(const vec<T, N>& a) {
    return std::sqrt(dot(a, a));
}

/// vector length squared
template <typename T, int N>
constexpr inline T lengthsqr(const vec<T, N>& a) {
    return dot(a, a);
}

/// vector normalization
template <typename T, int N>
constexpr inline vec<T, N> normalize(const vec<T, N>& a) {
    auto l = length(a);
    if (l == 0) return a;
    return a * (1 / l);
}

/// point distance
template <typename T, int N>
constexpr inline T dist(const vec<T, N>& a, const vec<T, N>& b) {
    return length(a - b);
}

/// point distance squared
template <typename T, int N>
constexpr inline T distsqr(const vec<T, N>& a, const vec<T, N>& b) {
    return lengthsqr(a - b);
}

/// angle between normalized vectors
template <typename T, int N>
constexpr inline T uangle(const vec<T, N>& a, const vec<T, N>& b) {
    auto d = dot(a, b);
    return d > 1 ? 0 : acos(d < -1 ? -1 : d);
}

/// angle between vectors
template <typename T, int N>
constexpr inline T angle(const vec<T, N>& a, const vec<T, N>& b) {
    return uangle(normalize(a), normalize(b));
}

/// vector linear interpolation
template <typename T, int N>
constexpr inline vec<T, N> lerp(const vec<T, N>& a, const vec<T, N>& b, T t) {
    return a * (1 - t) + b * t;
}

/// vector normalized linear interpolation
template <typename T, int N>
constexpr inline vec<T, N> nlerp(const vec<T, N>& a, const vec<T, N>& b, T t) {
    return normalize(lerp(a, b, t));
}

/// vector spherical linear interpolation (vectors have to be normalized)
template <typename T, int N>
constexpr inline vec<T, N> slerp(const vec<T, N>& a, const vec<T, N>& b, T t) {
    auto th = uangle(a, b);
    return th == 0 ?
               a :
               a * (sin(th * (1 - t)) / sin(th)) + b * (sin(th * t) / sin(th));
}

/// orthogonal vector
// http://lolengine.net/blog/2013/09/21/picking-orthogonal-vector-combing-coconuts)
template <typename T>
constexpr inline vec<T, 3> orthogonal(const vec<T, 3>& v) {
    return std::abs(v.x) > std::abs(v.z) ? vec<T, 3>{-v.y, v.x, 0} :
                                           vec<T, 3>{0, -v.z, v.y};
}

/// orthonormalize two vectors
template <typename T>
constexpr inline vec<T, 3> orthonormalize(
    const vec<T, 3>& a, const vec<T, 3>& b) {
    return normalize(a - b * dot(a, b));
}

/// vector component-wise min
template <typename T, int N>
constexpr inline vec<T, N> min(
    const vec<T, N>& x, const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = min(x[i], a[i], b[i]);
    return c;
}

/// vector component-wise max
template <typename T, int N>
constexpr inline vec<T, N> max(
    const vec<T, N>& x, const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = max(x[i], a[i], b[i]);
    return c;
}

/// vector component-wise clamp
template <typename T, int N>
constexpr inline vec<T, N> clamp(
    const vec<T, N>& x, const T& min, const T& max) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = clamp(x[i], min, max);
    return c;
}

/// vector component-wise clamp
template <typename T, int N>
constexpr inline vec<T, N> clamp(
    const vec<T, N>& x, const vec<T, N>& min, const vec<T, N>& max) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = clamp(x[i], min[i], max[i]);
    return c;
}

/// clamp the length of a vector
template <typename T, int N, typename T1>
constexpr inline vec<T, N> clamplen(const vec<T, N> x, T1 max) {
    auto l = length(x);
    return (l > (T)max) ? x * (T)max / l : x;
}

/// index of the min vector element
template <typename T, int N>
constexpr inline int min_element_idx(const vec<T, N>& a) {
    auto v = std::numeric_limits<T>::max();
    auto pos = -1;
    for (auto i = 0; i < N; i++) {
        if (v > a[i]) {
            v = a[i];
            pos = i;
        }
    }
    return pos;
}

/// index of the max vector element
template <typename T, int N>
constexpr inline int max_element_idx(const vec<T, N>& a) {
    auto v = -std::numeric_limits<T>::max();
    auto pos = -1;
    for (auto i = 0; i < N; i++) {
        if (v < a[i]) {
            v = a[i];
            pos = i;
        }
    }
    return pos;
}

/// index of the min vector element
template <typename T, int N>
constexpr inline T min_element_val(const vec<T, N>& a) {
    auto v = std::numeric_limits<T>::max();
    for (auto i = 0; i < N; i++) {
        if (v > a[i]) v = a[i];
    }
    return v;
}

/// index of the max vector element
template <typename T, int N>
constexpr inline T max_element_val(const vec<T, N>& a) {
    auto v = -std::numeric_limits<T>::max();
    for (auto i = 0; i < N; i++) {
        if (v < a[i]) v = a[i];
    }
    return v;
}

/// Element-wise sqrt
template <typename T, int N>
constexpr inline vec<T, N> sqrt(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = sqrt(a[i]);
    return c;
}

/// Element-wise pow
template <typename T, int N>
constexpr inline vec<T, N> pow(const vec<T, N>& a, const T b) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = pow(a[i], b);
    return c;
}

/// Element-wise exp
template <typename T, int N>
constexpr inline vec<T, N> exp(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = exp(a[i]);
    return c;
}

/// Element-wise log
template <typename T, int N>
constexpr inline vec<T, N> log(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = log(a[i]);
    return c;
}

/// Element-wise log10
template <typename T, int N>
constexpr inline vec<T, N> log10(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = log10(a[i]);
    return c;
}

/// Element-wise sin
template <typename T, int N>
constexpr inline vec<T, N> sin(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = sin(a[i]);
    return c;
}

/// Element-wise cos
template <typename T, int N>
constexpr inline vec<T, N> cos(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = cos(a[i]);
    return c;
}

/// Element-wise tan
template <typename T, int N>
constexpr inline vec<T, N> tan(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = tan(a[i]);
    return c;
}

/// Element-wise asin
template <typename T, int N>
constexpr inline vec<T, N> asin(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = asin(a[i]);
    return c;
}

/// Element-wise acos
template <typename T, int N>
constexpr inline vec<T, N> acos(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = acos(a[i]);
    return c;
}

/// Element-wise atan
template <typename T, int N>
constexpr inline vec<T, N> atan(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = atan(a[i]);
    return c;
}

/// Element-wise abs
template <typename T, int N>
constexpr inline vec<T, N> abs(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = abs(a[i]);
    return c;
}

/// Element-wise floor
template <typename T, int N>
constexpr inline vec<T, N> floor(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = floor(a[i]);
    return c;
}

/// Element-wise ceil
template <typename T, int N>
constexpr inline vec<T, N> ceil(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = ceil(a[i]);
    return c;
}

/// Element-wise round
template <typename T, int N>
constexpr inline vec<T, N> round(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = round(a[i]);
    return c;
}

/// Element-wise round
template <typename T, int N>
constexpr inline vec<T, N> atan2(const vec<T, N>& a, const vec<T, N>& b) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = atan2(a[i], b[i]);
    return c;
}

/// Check if finite
template <typename T, int N>
constexpr inline bool isfinite(const vec<T, N>& a) {
    for (auto i = 0; i < N; i++)
        if (!isfinite(a[i])) return false;
    return true;
}

/// Element-wise conversion
template <int N>
constexpr inline vec<byte, N> float_to_byte(const vec<float, N>& a) {
    auto c = vec<byte, N>();
    for (auto i = 0; i < N; i++) c[i] = float_to_byte(a[i]);
    return c;
}

/// Element-wise conversion
template <int N>
constexpr inline vec<float, N> byte_to_float(const vec<byte, N>& a) {
    auto c = vec<float, N>();
    for (auto i = 0; i < N; i++) c[i] = byte_to_float(a[i]);
    return c;
}

/// Hash functor for vec<T,N> for use with std::unordered_map
template <typename T, int N>
struct vec_hash {
    // from boost::hash_combine
    constexpr static size_t hash_combine(size_t h, size_t h1) {
        h ^= h1 + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
    constexpr size_t operator()(const vec<T, N>& v) const {
        auto hash = std::hash<T>();
        auto h = (size_t)0;
        for (auto i = 0; i < N; i++) h = hash_combine(h, hash(v[i]));
        return h;
    }
};

// -----------------------------------------------------------------------------
// MATRICES
// -----------------------------------------------------------------------------

///
/// Matrix of elements of compile time dimensions, stored in column major
/// format, with default initializer.
/// Colums access via operator[].
///
template <typename T, int N, int M>
struct mat {
    /// column data type
    using V = vec<T, N>;

    /// default constructor
    constexpr mat() {
        for (auto j = 0; j < M; j++) v[j] = V{};
    }
    /// diagonal constructor
    constexpr explicit mat(T vv) {
        for (auto j = 0; j < M; j++)
            for (auto i = 0; i < N; i++) v[j][i] = (i == j) ? vv : T{};
    }
    /// list constructor
    constexpr mat(const std::initializer_list<V>& vv) {
        assert(M == vv.size());
        auto i = 0;
        for (auto&& e : vv) v[i++] = e;
    }

    /// element access
    constexpr V& operator[](int i) { return v[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return v[i]; }

    /// data access
    constexpr V* data() { return v; }
    /// data access
    constexpr const V* data() const { return v; }

    /// element data
    V v[M];
};

///
/// Specialization for 2x2 float matrices.
///
template <>
struct mat<float, 2, 2> {
    /// size
    constexpr static const int N = 2, M = 2;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// default constructor
    constexpr mat() : x{0, 0}, y{0, 0} {}
    /// diagonal constructor
    constexpr explicit mat(T vv) : x{vv, 0}, y{0, vv} {}
    /// list constructor
    constexpr mat(const V& x, const V& y) : x(x), y(y) {}

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// element data
    V x;
    /// element data
    V y;
};

///
/// Specialization for 3x3 float matrices.
///
template <>
struct mat<float, 3, 3> {
    /// size
    constexpr static const int N = 3, M = 3;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// default constructor
    constexpr mat() : x{0, 0, 0}, y{0, 0, 0}, z{0, 0, 0} {}
    /// diagonal constructor
    constexpr explicit mat(T vv) : x{vv, 0, 0}, y{0, vv, 0}, z{0, 0, vv} {}
    /// list constructor
    constexpr mat(const V& x, const V& y, const V& z) : x(x), y(y), z(z) {}

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// element data
    V x;
    /// element data
    V y;
    /// element data
    V z;
};

///
/// Specialization for 4x4 float matrices.
///
template <>
struct mat<float, 4, 4> {
    /// size
    constexpr static const int N = 4, M = 4;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// default constructor
    constexpr mat()
        : x{0, 0, 0, 0}, y{0, 0, 0, 0}, z{0, 0, 0, 0}, w{0, 0, 0, 0} {}
    /// diagonal constructor
    constexpr explicit mat(T vv)
        : x{vv, 0, 0, 0}, y{0, vv, 0, 0}, z{0, 0, vv, 0}, w{0, 0, 0, vv} {}
    /// list constructor
    constexpr mat(const V& x, const V& y, const V& z, const V& w)
        : x(x), y(y), z(z), w(w) {}

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// element data
    V x;
    /// element data
    V y;
    /// element data
    V z;
    /// element data
    V w;
};

/// 1-dimensional float matrix
using mat1f = mat<float, 1, 1>;
/// 2-dimensional float matrix
using mat2f = mat<float, 2, 2>;
/// 3-dimensional float matrix
using mat3f = mat<float, 3, 3>;
/// 4-dimensional float matrix
using mat4f = mat<float, 4, 4>;

/// Initialize an identity matrix.
template <typename T, int N>
constexpr inline mat<T, N, N> identity_mat() {
    mat<T, N, N> c;
    for (auto j = 0; j < N; j++)
        for (auto i = 0; i < N; i++) c[j][i] = (i == j) ? 1 : 0;
    return c;
}

/// Specialization for Initialize an identity matrix.
template <>
constexpr inline mat3f identity_mat() {
    return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
}

/// Specialization for Initialize an identity matrix.
template <>
constexpr inline mat4f identity_mat() {
    return {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
}

/// 1-dimensional float identity matrix
const auto identity_mat1f = identity_mat<float, 1>();
/// 2-dimensional float identity matrix
const auto identity_mat2f = identity_mat<float, 2>();
/// 3-dimensional float identity matrix
const auto identity_mat3f = identity_mat<float, 3>();
/// 4-dimensional float identity matrix
const auto identity_mat4f = identity_mat<float, 4>();

/// iteration support
template <typename T, int N, int M>
constexpr inline vec<T, N>* begin(mat<T, N, M>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N, int M>
constexpr inline const vec<T, N>* begin(const mat<T, N, M>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N, int M>
constexpr inline vec<T, N>* end(mat<T, N, M>& a) {
    return a.v + M;
}

/// iteration support
template <typename T, int N, int M>
constexpr inline const vec<T, N>* end(const mat<T, N, M>& a) {
    return a.v + M;
}

/// vector operator ==
template <typename T, int N, int M>
constexpr inline bool operator==(const mat<T, N, M>& a, const mat<T, N, M>& b) {
    for (auto i = 0; i < M; i++)
        if (a[i] != b[i]) return false;
    return true;
}

/// vector operator !=
template <typename T, int N, int M>
constexpr inline bool operator!=(const mat<T, N, M>& a, const mat<T, N, M>& b) {
    return !(a == b);
}

/// matrix operator -
template <typename T, int N, int M>
constexpr inline mat<T, M, N> operator-(const mat<T, N, M>& a) {
    mat<T, N, M> c;
    for (auto i = 0; i < M; i++) c[i] = -a[i];
    return c;
}

/// matrix operator +
template <typename T, int N, int M>
constexpr inline mat<T, M, N> operator+(
    const mat<T, N, M>& a, const mat<T, N, M>& b) {
    mat<T, N, M> c;
    for (auto i = 0; i < M; i++) c[i] = a[i] + b[i];
    return c;
}

/// matrix scalar multiply
template <typename T, int N, int M>
constexpr inline mat<T, M, N> operator*(const mat<T, N, M>& a, T b) {
    mat<T, N, M> c;
    for (auto i = 0; i < M; i++) c[i] = a[i] * b;
    return c;
}

/// matrix scalar division
template <typename T, int N, int M>
constexpr inline mat<T, M, N> operator/(const mat<T, N, M>& a, T b) {
    mat<T, N, M> c;
    for (auto i = 0; i < M; i++) c[i] = a[i] / b;
    return c;
}

/// matrix-vector right multiply
template <typename T, int N, int M>
constexpr inline vec<T, N> operator*(
    const mat<T, N, M>& a, const vec<T, M>& b) {
    auto c = vec<T, N>();
    for (auto j = 0; j < M; j++) c += a[j] * b[j];
    return c;
}

/// matrix-vector left multiply
template <typename T, int N, int M>
constexpr inline vec<T, M> operator*(
    const vec<T, N>& a, const mat<T, N, M>& b) {
    vec<T, M> c;
    for (auto j = 0; j < M; j++) c[j] = dot(a, b[j]);
    return c;
}

/// matrix-matrix multiply
template <typename T, int N, int M, int K>
constexpr inline mat<T, N, M> operator*(
    const mat<T, N, K>& a, const mat<T, K, M>& b) {
    mat<T, N, M> c;
    for (auto j = 0; j < M; j++) c[j] = a * b[j];
    return c;
}

/// matrix-vector right multiply
template <>
constexpr inline vec2f operator*(const mat2f& a, const vec2f& b) {
    return a.x * b.x + a.y * b.y;
}

/// matrix-vector left multiply
template <>
constexpr inline vec2f operator*(const vec2f& a, const mat2f& b) {
    return {dot(a, b.x), dot(a, b.y)};
}

/// matrix-matrix multiply
template <>
constexpr inline mat2f operator*(const mat2f& a, const mat2f& b) {
    return {a * b.x, a * b.y};
}

/// matrix-vector right multiply
template <>
constexpr inline vec3f operator*(const mat3f& a, const vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// matrix-vector left multiply
template <>
constexpr inline vec3f operator*(const vec3f& a, const mat3f& b) {
    return {dot(a, b.x), dot(a, b.y), dot(a, b.z)};
}

/// matrix-matrix multiply
template <>
constexpr inline mat3f operator*(const mat3f& a, const mat3f& b) {
    return {a * b.x, a * b.y, a * b.z};
}

/// matrix-vector right multiply
template <>
constexpr inline vec4f operator*(const mat4f& a, const vec4f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

/// matrix-vector left multiply
template <>
constexpr inline vec4f operator*(const vec4f& a, const mat4f& b) {
    return {dot(a, b.x), dot(a, b.y), dot(a, b.z), dot(a, b.w)};
}

/// matrix-matrix multiply
template <>
constexpr inline mat4f operator*(const mat4f& a, const mat4f& b) {
    return {a * b.x, a * b.y, a * b.z, a * b.w};
}

/// matrix sum assignment
template <typename T, int N, int M>
constexpr inline mat<T, M, N>& operator+=(
    mat<T, N, M>& a, const mat<T, N, M>& b) {
    return a = a + b;
}

/// matrix-matrix multiply assignment
template <typename T, int N, int M>
constexpr inline mat<T, M, N>& operator*=(
    mat<T, N, M>& a, const mat<T, N, M>& b) {
    return a = a * b;
}

/// matrix scaling assignment
template <typename T, int N, int M>
constexpr inline mat<T, M, N>& operator*=(mat<T, N, M>& a, const T& b) {
    return a = a * b;
}

/// matrix scaling assignment
template <typename T, int N, int M>
constexpr inline mat<T, M, N>& operator/=(mat<T, N, M>& a, const T& b) {
    return a = a / b;
}

/// matrix diagonal
template <typename T, int N>
constexpr vec<T, N> mat_diagonal(const mat<T, N, N>& a) {
    vec<T, N> d;
    for (auto i = 0; i < N; i++) d[i] = a[i][i];
    return d;
}

/// matrix transpose
template <typename T, int N, int M>
constexpr inline mat<T, M, N> transpose(const mat<T, N, M>& a) {
    mat<T, M, N> c;
    for (auto j = 0; j < M; j++) {
        for (auto i = 0; i < N; i++) { c[i][j] = a[j][i]; }
    }
    return c;
}

/// matrix adjugate (2x2)
template <typename T>
constexpr inline mat<T, 2, 2> adjugate(const mat<T, 2, 2>& a) {
    return {{a.y.y, -a.x.y}, {-a.y.x, a.x.x}};
}

/// matrix adjugate (3x3)
template <typename T>
constexpr inline mat<T, 3, 3> adjugate(const mat<T, 3, 3>& a) {
    return {{a.y.y * a.z.z - a.z.y * a.y.z, a.z.y * a.x.z - a.x.y * a.z.z,
                a.x.y * a.y.z - a.y.y * a.x.z},
        {a.y.z * a.z.x - a.z.z * a.y.x, a.z.z * a.x.x - a.x.z * a.z.x,
            a.x.z * a.y.x - a.y.z * a.x.x},
        {a.y.x * a.z.y - a.z.x * a.y.y, a.z.x * a.x.y - a.x.x * a.z.y,
            a.x.x * a.y.y - a.y.x * a.x.y}};
}

/// matrix adjugate (4x4)
template <typename T>
constexpr inline mat<T, 4, 4> adjugate(const mat<T, 4, 4>& a) {
    return {{a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w +
                    a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w -
                    a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w,
                a.x.y * a.w.z * a.z.w + a.z.y * a.x.z * a.w.w +
                    a.w.y * a.z.z * a.x.w - a.w.y * a.x.z * a.z.w -
                    a.z.y * a.w.z * a.x.w - a.x.y * a.z.z * a.w.w,
                a.x.y * a.y.z * a.w.w + a.w.y * a.x.z * a.y.w +
                    a.y.y * a.w.z * a.x.w - a.x.y * a.w.z * a.y.w -
                    a.y.y * a.x.z * a.w.w - a.w.y * a.y.z * a.x.w,
                a.x.y * a.z.z * a.y.w + a.y.y * a.x.z * a.z.w +
                    a.z.y * a.y.z * a.x.w - a.x.y * a.y.z * a.z.w -
                    a.z.y * a.x.z * a.y.w - a.y.y * a.z.z * a.x.w},
        {a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x + a.w.z * a.z.w * a.y.x -
                a.y.z * a.z.w * a.w.x - a.w.z * a.y.w * a.z.x -
                a.z.z * a.w.w * a.y.x,
            a.x.z * a.z.w * a.w.x + a.w.z * a.x.w * a.z.x +
                a.z.z * a.w.w * a.x.x - a.x.z * a.w.w * a.z.x -
                a.z.z * a.x.w * a.w.x - a.w.z * a.z.w * a.x.x,
            a.x.z * a.w.w * a.y.x + a.y.z * a.x.w * a.w.x +
                a.w.z * a.y.w * a.x.x - a.x.z * a.y.w * a.w.x -
                a.w.z * a.x.w * a.y.x - a.y.z * a.w.w * a.x.x,
            a.x.z * a.y.w * a.z.x + a.z.z * a.x.w * a.y.x +
                a.y.z * a.z.w * a.x.x - a.x.z * a.z.w * a.y.x -
                a.y.z * a.x.w * a.z.x - a.z.z * a.y.w * a.x.x},
        {a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y + a.z.w * a.w.x * a.y.y -
                a.y.w * a.w.x * a.z.y - a.z.w * a.y.x * a.w.y -
                a.w.w * a.z.x * a.y.y,
            a.x.w * a.w.x * a.z.y + a.z.w * a.x.x * a.w.y +
                a.w.w * a.z.x * a.x.y - a.x.w * a.z.x * a.w.y -
                a.w.w * a.x.x * a.z.y - a.z.w * a.w.x * a.x.y,
            a.x.w * a.y.x * a.w.y + a.w.w * a.x.x * a.y.y +
                a.y.w * a.w.x * a.x.y - a.x.w * a.w.x * a.y.y -
                a.y.w * a.x.x * a.w.y - a.w.w * a.y.x * a.x.y,
            a.x.w * a.z.x * a.y.y + a.y.w * a.x.x * a.z.y +
                a.z.w * a.y.x * a.x.y - a.x.w * a.y.x * a.z.y -
                a.z.w * a.x.x * a.y.y - a.y.w * a.z.x * a.x.y},
        {a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z + a.w.x * a.z.y * a.y.z -
                a.y.x * a.z.y * a.w.z - a.w.x * a.y.y * a.z.z -
                a.z.x * a.w.y * a.y.z,
            a.x.x * a.z.y * a.w.z + a.w.x * a.x.y * a.z.z +
                a.z.x * a.w.y * a.x.z - a.x.x * a.w.y * a.z.z -
                a.z.x * a.x.y * a.w.z - a.w.x * a.z.y * a.x.z,
            a.x.x * a.w.y * a.y.z + a.y.x * a.x.y * a.w.z +
                a.w.x * a.y.y * a.x.z - a.x.x * a.y.y * a.w.z -
                a.w.x * a.x.y * a.y.z - a.y.x * a.w.y * a.x.z,
            a.x.x * a.y.y * a.z.z + a.z.x * a.x.y * a.y.z +
                a.y.x * a.z.y * a.x.z - a.x.x * a.z.y * a.y.z -
                a.y.x * a.x.y * a.z.z - a.z.x * a.y.y * a.x.z}};
}

/// matrix determinant (2x2)
template <typename T>
constexpr inline T determinant(const mat<T, 2, 2>& a) {
    return a.x.x * a.y.y - a.x.y * a.y.x;
}

/// matrix determinant (3x3)
template <typename T>
constexpr inline T determinant(const mat<T, 3, 3>& a) {
    return a.x.x * (a.y.y * a.z.z - a.z.y * a.y.z) +
           a.x.y * (a.y.z * a.z.x - a.z.z * a.y.x) +
           a.x.z * (a.y.x * a.z.y - a.z.x * a.y.y);
}

/// matrix determinant (4x4)
template <typename T>
constexpr inline T determinant(const mat<T, 4, 4>& a) {
    return a.x.x * (a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w +
                       a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w -
                       a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w) +
           a.x.y * (a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x +
                       a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x -
                       a.w.z * a.y.w * a.z.x - a.z.z * a.w.w * a.y.x) +
           a.x.z * (a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y +
                       a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y -
                       a.z.w * a.y.x * a.w.y - a.w.w * a.z.x * a.y.y) +
           a.x.w * (a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z +
                       a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z -
                       a.w.x * a.y.y * a.z.z - a.z.x * a.w.y * a.y.z);
}

/// matrix inverse (uses adjugate and determinant)
template <typename T, int N>
constexpr inline mat<T, N, N> inverse(const mat<T, N, N>& a) {
    return adjugate(a) / determinant(a);
}

// -----------------------------------------------------------------------------
// RIGID BODY TRANSFORMS/FRAMES
// -----------------------------------------------------------------------------

///
/// Rigid transforms stored as a column-major affine matrix Nx(N+1).
/// In memory, this representation is equivalent to storing an NxN rotation
/// followed by a Nx1 translation. Viewed this way, the representation allows
/// also to retrive the axis of the coordinate frame as the first N column and
/// the translation as the N+1 column.
/// Colums access via operator[]. Access rotation and position with pos() and
/// rot().
///
template <typename T, int N>
struct frame {
    /// column data type
    using V = vec<T, N>;
    /// rotation data type
    using M = mat<T, N, N>;

    /// default constructor
    constexpr frame() {
        for (auto i = 0; i < N + 1; i++) v[i] = V{};
    }
    /// element constructor
    constexpr frame(const std::initializer_list<vec<T, N>>& vv) {
        assert(N + 1 == vv.size());
        auto i = 0;
        for (auto&& e : vv) v[i++] = e;
    }
    /// element constructor
    constexpr frame(const M& m, const V& t) {
        for (auto i = 0; i < N; i++) v[i] = m[i];
        v[N + 1] = t;
    }

    /// element access
    constexpr V& operator[](int i) { return v[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return v[i]; }

    /// data access
    constexpr V* data() { return v; }
    /// data access
    constexpr const V* data() const { return v; }

    /// access position
    constexpr V& pos() { return v[N]; }
    /// access position
    constexpr const V& pos() const { return v[N]; }

    /// access rotation
    constexpr M& rot() { return *(M*)v; }
    /// access rotation
    constexpr const M& rot() const { return *(M*)v; }

    /// element data
    V v[N + 1];
};

///
/// Specialization for 3D float frames.
///
template <>
struct frame<float, 2> {
    /// size
    constexpr static const int N = 2;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;
    /// rotation data type
    using M = mat<T, N, N>;

    /// default constructor
    constexpr frame() : x{0, 0}, y{0, 0}, o{0, 0} {}
    /// element constructor
    constexpr frame(const V& x, const V& y, const V& o) : x(x), y(y), o(o) {}
    /// element constructor
    constexpr frame(const M& m, const V& t) : x(m.x), y(m.y), o(t) {}

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// access position
    constexpr V& pos() { return o; }
    /// access position
    constexpr const V& pos() const { return o; }

    /// access rotation
    constexpr M& rot() { return *(M*)(&x); }
    /// access rotation
    constexpr const M& rot() const { return *(M*)(&x); }

    /// element data
    V x;
    /// element data
    V y;
    /// element data
    V o;
};

///
/// Specialization for 3D float frames.
///
template <>
struct frame<float, 3> {
    /// size
    constexpr static const int N = 3;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;
    /// rotation data type
    using M = mat<T, N, N>;

    /// default constructor
    constexpr frame() : x{0, 0, 0}, y{0, 0, 0}, z{0, 0, 0}, o{0, 0, 0} {}

    /// element constructor
    constexpr frame(const V& x, const V& y, const V& z, const V& o)
        : x(x), y(y), z(z), o(o) {}

    /// element constructor
    constexpr frame(const M& m, const V& t) : x(m.x), y(m.y), z(m.z), o(t) {}

    /// conversion from matrix (assumes the matrix is a frame, so dangerous!)
    constexpr frame(const mat<T, 4, 4>& m)
        : x(m.x.x, m.x.y, m.x.z)
        , y(m.y.x, m.y.y, m.y.z)
        , z(m.z.x, m.z.y, m.z.z)
        , o(m.w.x, m.w.y, m.w.z) {}

    /// conversion to matrix
    constexpr explicit operator mat<T, 4, 4>() const {
        return {{x.x, x.y, x.z, 0}, {y.x, y.y, y.z, 0}, {z.x, z.y, z.z, 0},
            {o.x, o.y, o.z, 1}};
    }

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// access position
    constexpr V& pos() { return o; }
    /// access position
    constexpr const V& pos() const { return o; }

    /// access rotation
    constexpr M& rot() { return *(M*)(&x); }
    /// access rotation
    constexpr const M& rot() const { return *(M*)(&x); }

    /// element data
    V x;
    /// element data
    V y;
    /// element data
    V z;
    /// element data
    V o;
};

/// 1-dimensional float frame
using frame1f = frame<float, 1>;
/// 2-dimensional float frame
using frame2f = frame<float, 2>;
/// 3-dimensional float frame
using frame3f = frame<float, 3>;
/// 4-dimensional float frame
using frame4f = frame<float, 4>;

/// Initialize an identity frame.
template <typename T, int N>
constexpr inline frame<T, N> identity_frame() {
    frame<T, N> c;
    for (auto j = 0; j < N; j++)
        for (auto i = 0; i < N; i++) c[j][i] = (i == j) ? 1 : 0;
    for (auto i = 0; i < N; i++) c[N][i] = 0;
    return c;
}

// initializes a frame3 from origin and z.
template <typename T>
constexpr inline frame<T, 3> make_frame3_fromz(
    const vec<T, 3>& o, const vec<T, 3>& z_) {
    auto z = normalize(z_);
    auto x = normalize(orthogonal(z));
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

// initializes a frame3 from origin, z and x.
template <typename T>
constexpr inline frame<T, 3> make_frame3_fromzx(
    const vec<T, 3>& o, const vec<T, 3>& z_, const vec<T, 3>& x_) {
    auto z = normalize(z_);
    auto x = orthonormalize(x_, z);
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

/// Initialize an identity frame.
template <>
constexpr inline frame2f identity_frame() {
    return {{1, 0}, {0, 1}, {0, 0}};
}

/// Initialize an identity frame.
template <>
constexpr inline frame3f identity_frame() {
    return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}};
}

// initializes a frame3 from origin and z.
template <>
constexpr inline frame3f make_frame3_fromz(const vec3f& o, const vec3f& z_) {
    auto z = normalize(z_);
    auto x = normalize(orthogonal(z));
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

// initializes a frame3 from origin, z and x.
template <typename T>
constexpr inline frame3f make_frame3_fromzx(
    const vec3f& o, const vec3f& z_, const vec3f& x_) {
    auto z = normalize(z_);
    auto x = orthonormalize(x_, z);
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

/// 1-dimensional float identity frame
const auto identity_frame1f = identity_frame<float, 1>();
/// 2-dimensional float identity frame
const auto identity_frame2f = identity_frame<float, 2>();
/// 3-dimensional float identity frame
const auto identity_frame3f = identity_frame<float, 3>();
/// 4-dimensional float identity frame
const auto identity_frame4f = identity_frame<float, 4>();

/// frame position const access
template <typename T, int N>
constexpr inline const vec<T, N>& pos(const frame<T, N>& f) {
    return f[N];
}

/// frame rotation const access
template <typename T, int N>
constexpr inline const mat<T, N, N>& rot(const frame<T, N>& f) {
    return *(mat<T, N, N>*)&f;
}

/// frame position reference
template <typename T, int N>
constexpr inline vec<T, N>& pos(frame<T, N>& f) {
    return f[N];
}

/// frame rotation reference
template <typename T, int N>
constexpr inline mat<T, N, N>& rot(frame<T, N>& f) {
    return *(mat<T, N, N>*)&f;
}

/// iteration support
template <typename T, int N>
constexpr inline vec<T, N>* begin(frame<T, N>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N>
constexpr inline const vec<T, N>* begin(const frame<T, N>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N>
constexpr inline vec<T, N>* end(frame<T, N>& a) {
    return a.v + N + 1;
}

/// iteration support
template <typename T, int N>
constexpr inline const vec<T, N>* end(const frame<T, N>& a) {
    return a.v + N + 1;
}

/// frame to matrix conversion
template <typename T, int N>
constexpr inline mat<T, N + 1, N + 1> to_mat(const frame<T, N>& a) {
    auto m = mat<T, N + 1, N + 1>();
    for (auto j = 0; j < N; j++) {
        (vec<T, N>&)m[j] = a[j];
        m[j][N] = 0;
    }
    (vec<T, N>&)m[N] = a[N];
    m[N][N] = 1;
    return m;
}

/// matrix to frame conversion
template <typename T, int N>
constexpr inline frame<T, N - 1> to_frame(const mat<T, N, N>& a) {
    auto f = frame<T, N - 1>();
    for (auto j = 0; j < N; j++) {
        for (auto i = 0; i < N - 1; i++) { f[j][i] = a[j][i]; }
    }
    return f;
}

/// vector operator ==
template <typename T, int N>
constexpr inline bool operator==(const frame<T, N>& a, const frame<T, N>& b) {
    for (auto i = 0; i < N + 1; i++)
        if (a[i] != b[i]) return false;
    return true;
}

/// vector operator !=
template <typename T, int N>
constexpr inline bool operator!=(const frame<T, N>& a, const frame<T, N>& b) {
    return !(a == b);
}

/// frame composition (equivalent to affine matrix multiply)
template <typename T, int N>
constexpr inline frame<T, N> operator*(
    const frame<T, N>& a, const frame<T, N>& b) {
    return {a.rot() * b.rot(), a.rot() * b.pos() + a.pos()};
}

/// frame inverse (equivalent to rigid affine inverse)
template <typename T, int N>
constexpr inline frame<T, N> inverse(const frame<T, N>& a) {
    auto minv = transpose(rot(a));
    return {minv, -(minv * pos(a))};
}

/// frame composition (equivalent to affine matrix multiply)
template <>
constexpr inline frame3f operator*(const frame3f& a, const frame3f& b) {
    return {a.rot() * b.rot(), a.rot() * b.pos() + a.pos()};
}

/// frame inverse (equivalent to rigid affine inverse)
template <>
constexpr inline frame3f inverse(const frame3f& a) {
    auto minv = transpose(a.rot());
    return {minv, -(minv * a.pos())};
}

// -----------------------------------------------------------------------------
// QUATERNIONS
// -----------------------------------------------------------------------------

///
/// Quaternion placeholder. Only helpful in the specialization.
///
template <typename T, int N>
struct quat;

///
/// Quaternions implemented as a vec<T,4>. Data access via operator[].
/// Quaterions are xi + yj + zk + w.
///
template <typename T>
struct quat<T, 4> {
    /// size
    constexpr static const int N = 4;

    /// default constructor
    constexpr quat() : x{0}, y{0}, z{0}, w{1} {}

    // list constructor
    constexpr quat(const T& x, const T& y, const T& z, const T& w)
        : x{x}, y{y}, z{z}, w{w} {}

    /// conversion from vec
    constexpr explicit quat(const vec<T, N>& vv)
        : x{vv.x}, y{vv.y}, z{vv.z}, w{vv.w} {}
    /// conversion to vec
    constexpr explicit operator vec<T, N>() const { return {x, y, z, w}; }

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// data
    T x;
    /// data
    T y;
    /// data
    T z;
    /// data
    T w;
};

/// float quaterion
using quat4f = quat<float, 4>;

/// float identity quaterion
const auto identity_quat4f = quat<float, 4>{0, 0, 0, 1};

/// vector operator ==
template <typename T, int N>
constexpr inline bool operator==(const quat<T, N>& a, const quat<T, N>& b) {
    for (auto i = 0; i < N; i++)
        if (a[i] != b[i]) return false;
    return true;
}

/// vector operator !=
template <typename T, int N>
constexpr inline bool operator!=(const quat<T, N>& a, const quat<T, N>& b) {
    return !(a == b);
}

/// quaterion multiply
template <typename T>
constexpr quat<T, 4> operator*(const quat<T, 4>& a, const quat<T, 4>& b) {
    return {a.x * b.w + a.w * b.x + a.y * b.w - a.z * b.y,
        a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
        a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

/// quaterion conjugate
template <typename T>
constexpr quat<T, 4> conjugate(const quat<T, 4>& v) {
    return {-v.x, -v.y, -v.z, v.w};
}

/// quaterion inverse
template <typename T>
constexpr quat<T, 4> inverse(const quat<T, 4>& v) {
    return conjugate(v) / lengthsqr(vec<T, 4>(v));
}

/// quaterion inverse
template <typename T>
constexpr quat<T, 4> normalize(const quat<T, 4>& v) {
    auto l = length(vec<T, 4>{v.x, v.y, v.z, v.w});
    if (!l) return {0, 0, 0, 1};
    return {v.x / l, v.y / l, v.z / l, v.w / l};
}

/// quaterion normalized linear interpolation
template <typename T>
constexpr quat<T, 4> nlerp(const quat<T, 4>& a, const quat<T, 4>& b, T t) {
    return (quat<T, 4>)nlerp(vec<T, 4>(a),
        dot(vec<T, 4>(a), vec<T, 4>(b)) < 0 ? -vec<T, 4>(b) : vec<T, 4>(b), t);
}

/// quaterion spherical linear interpolation
template <typename T>
constexpr quat<T, 4> slerp(const quat<T, 4>& a, const quat<T, 4>& b, T t) {
    return (quat<T, 4>)slerp(vec<T, 4>(a),
        dot(vec<T, 4>(a), vec<T, 4>(b)) < 0 ? -vec<T, 4>(b) : vec<T, 4>(b), t);
}

// -----------------------------------------------------------------------------
// AXIS ALIGNED BOUNDING BOXES
// -----------------------------------------------------------------------------

///
/// Axis aligned bounding box represented as a min/max vector pair.
/// Access min/max with operator[].
///
template <typename T, int N>
struct bbox {
    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox() {
        for (auto i = 0; i < N; i++) {
            min[i] = std::numeric_limits<T>::max();
            max[i] = std::numeric_limits<T>::lowest();
        }
    }

    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

///
/// Specialization for float 3D bounding boxes.
///
template <>
struct bbox<float, 1> {
    /// size
    constexpr static const int N = 1;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox() : min{flt_max}, max{flt_min} {}
    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

///
/// Specialization for float 3D bounding boxes.
///
template <>
struct bbox<float, 2> {
    /// size
    constexpr static const int N = 2;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox() : min{flt_max, flt_max}, max{flt_min, flt_min} {}
    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

///
/// Specialization for float 3D bounding boxes.
///
template <>
struct bbox<float, 3> {
    /// size
    constexpr static const int N = 3;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox()
        : min{flt_max, flt_max, flt_max}, max{flt_min, flt_min, flt_min} {}
    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

///
/// Specialization for float 3D bounding boxes.
///
template <>
struct bbox<float, 4> {
    /// size
    constexpr static const int N = 4;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox()
        : min{flt_max, flt_max, flt_max, flt_max}
        , max{flt_min, flt_min, flt_min, flt_min} {}
    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

/// 1-dimensional float bbox
using bbox1f = bbox<float, 1>;
/// 2-dimensional float bbox
using bbox2f = bbox<float, 2>;
/// 3-dimensional float bbox
using bbox3f = bbox<float, 3>;
/// 4-dimensional float bbox
using bbox4f = bbox<float, 4>;

/// 1-dimensional float bbox
using bbox1i = bbox<int, 1>;
/// 2-dimensional float bbox
using bbox2i = bbox<int, 2>;
/// 3-dimensional float bbox
using bbox3i = bbox<int, 3>;
/// 4-dimensional float bbox
using bbox4i = bbox<int, 4>;

/// initializes an empty bbox
template <typename T, int N>
constexpr inline bbox<T, N> invalid_bbox() {
    auto a = bbox<T, N>();
    for (auto i = 0; i < N; i++) {
        a.min[i] = std::numeric_limits<T>::max();
        a.max[i] = std::numeric_limits<T>::lowest();
    }
    return a;
}

/// initialize a bonding box from a list of points
template <typename T, int N>
constexpr inline bbox<T, N> make_bbox(int count, const vec<T, N>* v) {
    auto a = invalid_bbox<T, N>();
    for (auto j = 0; j < count; j++) {
        auto&& vv = v[j];
        for (auto i = 0; i < N; i++) {
            a.min[i] = min(a.min[i], vv[i]);
            a.max[i] = max(a.max[i], vv[i]);
        }
    }
    return a;
}

/// initialize a bonding box from a list of points
template <typename T, int N>
constexpr inline bbox<T, N> make_bbox(
    const std::initializer_list<vec<T, N>>& v) {
    auto a = invalid_bbox<T, N>();
    for (auto&& vv : v) {
        for (auto i = 0; i < N; i++) {
            a.min[i] = min(a.min[i], vv[i]);
            a.max[i] = max(a.max[i], vv[i]);
        }
    }
    return a;
}

/// 1-dimensional float empty bbox
const auto invalid_bbox1f = bbox1f();
/// 2-dimensional float empty bbox
const auto invalid_bbox2f = bbox2f();
/// 3-dimensional float empty bbox
const auto invalid_bbox3f = bbox3f();
/// 4-dimensional float empty bbox
const auto invalid_bbox4f = bbox4f();

/// computes the center of a bbox
template <typename T, int N>
constexpr inline vec<T, N> center(const bbox<T, N>& a) {
    return (a.min + a.max) / (T)2;
}

/// computes the diagonal of a bbox
template <typename T, int N>
constexpr inline vec<T, N> diagonal(const bbox<T, N>& a) {
    return a.max - a.min;
}

/// iteration support
template <typename T, int N>
constexpr inline vec<T, N>* begin(bbox<T, N>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N>
constexpr inline const vec<T, N>* begin(const bbox<T, N>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N>
constexpr inline vec<T, N>* end(bbox<T, N>& a) {
    return a.v + 2;
}

/// iteration support
template <typename T, int N>
constexpr inline const vec<T, N>* end(const bbox<T, N>& a) {
    return a.v + 2;
}

/// expands a bounding box with a point
template <typename T, int N>
constexpr inline bbox<T, N> expand(const bbox<T, N>& a, const vec<T, N>& b) {
    bbox<T, N> c;
    for (auto i = 0; i < N; i++) {
        c.x[i] = min(a.x[i], b[i]);
        c.y[i] = max(a.y[i], b[i]);
    }
    return c;
}

/// expands a bounding box with a bounding box
template <typename T, int N>
constexpr inline bbox<T, N> expand(const bbox<T, N>& a, const bbox<T, N>& b) {
    bbox<T, N> c;
    for (auto i = 0; i < N; i++) {
        c.x[i] = min(a.x[i], b.x[i]);
        c.y[i] = max(a.y[i], b.y[i]);
    }
    return c;
}

/// check if a bounding box contains a point
template <typename T, int N>
constexpr inline bool contains(const bbox<T, N>& a, const vec<T, N>& b) {
    for (auto i = 0; i < N; i++) {
        if (a.min[i] > b[i] || a.max[i] < b[i]) return false;
    }
    return true;
}

/// check if a bounding box contains a bounding box
template <typename T, int N>
constexpr inline bool contains(const bbox<T, N>& a, const bbox<T, N>& b) {
    for (auto i = 0; i < N; i++) {
        if (a.min[i] > b.max[i] || a.max[i] < b.min[i]) return false;
    }
    return true;
}

/// expands a bounding box with a point
template <>
constexpr inline bbox3f expand(const bbox3f& a, const vec3f& b) {
    return {{min(a.min.x, b.x), min(a.min.y, b.y), min(a.min.z, b.z)},
        {max(a.max.x, b.x), max(a.max.y, b.y), max(a.max.z, b.z)}};
}

/// expands a bounding box with a bounding box
template <>
constexpr inline bbox3f expand(const bbox3f& a, const bbox3f& b) {
    return {
        {min(a.min.x, b.min.x), min(a.min.y, b.min.y), min(a.min.z, b.min.z)},
        {max(a.max.x, b.max.x), max(a.max.y, b.max.y), max(a.max.z, b.max.z)}};
}

/// check if a bounding box contains a point
template <>
constexpr inline bool contains(const bbox3f& a, const vec3f& b) {
    if (a.min.x > b.x || a.max.x < b.x) return false;
    if (a.min.y > b.y || a.max.y < b.y) return false;
    if (a.min.z > b.z || a.max.z < b.z) return false;
    return true;
}

/// check if a bounding box contains a bounding box
template <>
constexpr inline bool contains(const bbox3f& a, const bbox3f& b) {
    if (a.min.x > b.max.x || a.max.x < b.min.x) return false;
    if (a.min.y > b.max.y || a.max.y < b.min.y) return false;
    if (a.min.z > b.max.z || a.max.z < b.min.z) return false;
    return true;
}

/// same as expand()
template <typename T, int N>
constexpr inline bbox<T, N> operator+(const bbox<T, N>& a, const T& b) {
    return expand(a, b);
}

/// same as expand()
template <typename T, int N>
constexpr inline bbox<T, N> operator+(
    const bbox<T, N>& a, const bbox<T, N>& b) {
    return expand(a, b);
}

/// assign to expand()
template <typename T, int N>
constexpr inline bbox<T, N>& operator+=(bbox<T, N>& a, const vec<T, N>& b) {
    return a = expand(a, b);
}

/// assign to expand()
template <typename T, int N>
constexpr inline bbox<T, N>& operator+=(bbox<T, N>& a, const bbox<T, N>& b) {
    return a = expand(a, b);
}

// -----------------------------------------------------------------------------
// RAYS
// -----------------------------------------------------------------------------

///
/// Rays with origin, direction and min/max t value.
///
template <typename T, int N>
struct ray {
    /// origin
    vec<T, N> o;
    /// direction
    vec<T, N> d;
    /// minimum distance
    T tmin;
    /// maximum distance
    T tmax;

    /// default constructor
    constexpr ray()
        : o(), d(0, 0, 1), tmin(0), tmax(std::numeric_limits<T>::max()) {}
    /// initializes a ray from its elements
    constexpr ray(const vec<T, N>& o, const vec<T, N>& d, T tmin = 0,
        T tmax = std::numeric_limits<T>::max())
        : o(o), d(d), tmin(tmin), tmax(tmax) {}
};

///
/// Sepcialization for 3D float rays.
///
template <>
struct ray<float, 3> {
    /// size
    constexpr static const int N = 3;
    /// type
    using T = float;

    /// origin
    vec<T, N> o;
    /// direction
    vec<T, N> d;
    /// minimum distance
    T tmin;
    /// maximum distance
    T tmax;

    /// default constructor
    constexpr ray() : o{0, 0, 0}, d{0, 0, 1}, tmin{0}, tmax{flt_max} {}
    /// initializes a ray from its elements
    constexpr ray(
        const vec<T, N>& o, const vec<T, N>& d, T tmin = 0, T tmax = flt_max)
        : o(o), d(d), tmin(tmin), tmax(tmax) {}
};

/// 1-dimensional float ray
using ray1f = ray<float, 1>;
/// 2-dimensional float ray
using ray2f = ray<float, 2>;
/// 3-dimensional float ray
using ray3f = ray<float, 3>;
/// 4-dimensional float ray
using ray4f = ray<float, 4>;

/// evalutes the position along the ray
template <typename T, int N>
constexpr inline vec<T, N> eval(const ray<T, N>& ray, T t) {
    return ray.o + t * ray.d;
}

// -----------------------------------------------------------------------------
// TRANSFORMS
// -----------------------------------------------------------------------------

/// transforms a point by a matrix
template <typename T, int N>
constexpr inline vec<T, N> transform_point(
    const mat<T, N + 1, N + 1>& a, const vec<T, N>& b) {
    // make it generic
    auto vb = vec<T, N + 1>();
    (vec<T, N>&)vb = b;
    vb[N] = 1;
    auto tvb = a * vb;
    return *(vec<T, N>*)(&tvb) / tvb[N];
}

/// transforms a vector by a matrix
template <typename T, int N>
constexpr inline vec<T, N> transform_vector(
    const mat<T, N + 1, N + 1>& a, const vec<T, N>& b) {
    // make it generic
    auto vb = vec<T, N + 1>();
    (vec<T, N>&)vb = b;
    vb[N] = 0;
    auto tvb = a * vb;
    return *(vec<T, N>*)(&tvb);
}

/// transforms a direction by a matrix
template <typename T, int N>
constexpr inline vec<T, N> transform_direction(
    const mat<T, N + 1, N + 1>& a, const vec<T, N>& b) {
    return normalize(transform_vector(a, b));
}

/// transforms a point by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_point(
    const frame<T, N>& a, const vec<T, N>& b) {
    return rot(a) * b + pos(a);
}

/// transforms a vector by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_vector(
    const frame<T, N>& a, const vec<T, N>& b) {
    return rot(a) * b;
}

/// transforms a direction by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_direction(
    const frame<T, N>& a, const vec<T, N>& b) {
    return rot(a) * b;
}

/// transforms a frame by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline frame<T, N> transform_frame(
    const frame<T, N>& a, const frame<T, N>& b) {
    return {rot(a) * rot(b), pos(a) * pos(b) + pos(a)};
}

/// inverse transforms a point by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_point_inverse(
    const frame<T, N>& a, const vec<T, N>& b) {
    return (b - pos(a)) * rot(a);
}

/// inverse transforms a vector by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_vector_inverse(
    const frame<T, N>& a, const vec<T, N>& b) {
    return b * rot(a);
}

/// inverse transforms a direction by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_direction_inverse(
    const frame<T, N>& a, const vec<T, N>& b) {
    return b * rot(a);
}

/// transforms a point by a matrix
template <>
constexpr inline vec3f transform_point(const mat4f& a, const vec3f& b) {
    auto vb = vec4f{b.x, b.y, b.z, 1};
    auto tvb = a * vb;
    return vec3f{tvb.x, tvb.y, tvb.z} / tvb.w;
}

/// transforms a vector by a matrix
template <>
constexpr inline vec3f transform_vector(const mat4f& a, const vec3f& b) {
    auto vb = vec4f{b.x, b.y, b.z, 0};
    auto tvb = a * vb;
    return vec3f{tvb.x, tvb.y, tvb.z};
}

/// transforms a direction by a matrix
template <>
constexpr inline vec3f transform_direction(const mat4f& a, const vec3f& b) {
    return normalize(transform_vector(a, b));
}

/// transforms a point by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_point(const frame3f& a, const vec3f& b) {
    return a.rot() * b + a.pos();
}

/// transforms a vector by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_vector(const frame3f& a, const vec3f& b) {
    return a.rot() * b;
}

/// transforms a direction by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_direction(const frame3f& a, const vec3f& b) {
    return a.rot() * b;
}

/// transforms a frame by a frame (rigid affine transform)
template <>
constexpr inline frame3f transform_frame(const frame3f& a, const frame3f& b) {
    return {a.rot() * b.rot(), a.rot() * b.pos() + a.pos()};
}

/// inverse transforms a point by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_point_inverse(
    const frame3f& a, const vec3f& b) {
    return (b - a.pos()) * a.rot();
}

/// inverse transforms a vector by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_vector_inverse(
    const frame3f& a, const vec3f& b) {
    return b * a.rot();
}

/// inverse transforms a direction by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_direction_inverse(
    const frame3f& a, const vec3f& b) {
    return b * a.rot();
}

/// transforms a ray by a matrix (direction is not normalized after)
template <typename T, int N>
constexpr inline ray<T, N> transform_ray(
    const mat<T, N + 1, N + 1>& a, const ray<T, N>& b) {
    return {transform_point(a, b.o), transform_vector(a, b.d), b.tmin, b.tmax};
}

/// transforms a bbox by a matrix
template <typename T>
constexpr inline bbox<T, 3> transform_bbox(
    const mat<T, 4, 4>& a, const bbox<T, 3>& b) {
    vec<T, 3> corners[8] = {
        {b.min.x, b.min.y, b.min.z}, {b.min.x, b.min.y, b.max.z},
        {b.min.x, b.max.y, b.min.z}, {b.min.x, b.max.y, b.max.z},
        {b.max.x, b.min.y, b.min.z}, {b.max.x, b.min.y, b.max.z},
        {b.max.x, b.max.y, b.min.z}, {b.max.x, b.max.y, b.max.z},
    };
    auto xformed = bbox<T, 3>();
    for (auto j = 0; j < 8; j++) xformed += transform_point(a, corners[j]);
    return xformed;
}

/// transforms a ray by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline ray<T, N> transform_ray(
    const frame<T, N>& a, const ray<T, N>& b) {
    return {
        transform_point(a, b.o), transform_direction(a, b.d), b.tmin, b.tmax};
}

/// transforms a bbox by a frame (rigid affine transform)
template <typename T>
constexpr inline bbox<T, 3> transform_bbox(
    const frame<T, 3>& a, const bbox<T, 3>& b) {
#if 0
    vec<T, 3> corners[8] = {
        {b.min.x, b.min.y, b.min.z}, {b.min.x, b.min.y, b.max.z},
        {b.min.x, b.max.y, b.min.z}, {b.min.x, b.max.y, b.max.z},
        {b.max.x, b.min.y, b.min.z}, {b.max.x, b.min.y, b.max.z},
        {b.max.x, b.max.y, b.min.z}, {b.max.x, b.max.y, b.max.z},
    };
    auto xformed = bbox<T, 3>();
    for (auto j = 0; j < 8; j++) xformed += transform_point(a, corners[j]);
    return xformed;
#else
    // Code from Real-time Collision Detection by Christer Ericson Sect. 4.2.6
    // Transform AABB a by the matrix m and translation t,
    // find maximum extents, and store result into AABB b.
    // start by adding in translation
    auto c = bbox<T, 3>{pos(a), pos(a)};
    // for all three axes
    for (auto i = 0; i < 3; i++) {
        // form extent by summing smaller and larger terms respectively
        for (auto j = 0; j < 3; j++) {
            auto e = a.rot()[j][i] * b.min[j];
            auto f = a.rot()[j][i] * b.max[j];
            if (e < f) {
                c.min[i] += e;
                c.max[i] += f;
            } else {
                c.min[i] += f;
                c.max[i] += e;
            }
        }
    }
    return c;
#endif
}

/// inverse transforms a ray by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline ray<T, N> transform_ray_inverse(
    const frame<T, N>& a, const ray<T, N>& b) {
    return {transform_point_inverse(a, b.o),
        transform_direction_inverse(a, b.d), b.tmin, b.tmax};
}

/// inverse transforms a bbox by a frame (rigid affine transform)
template <typename T>
constexpr inline bbox<T, 3> transform_bbox_inverse(
    const frame<T, 3>& a, const bbox<T, 3>& b) {
    return transform_bbox(inverse(a), b);
}

/// rotation matrix from axis-angle
template <typename T>
constexpr inline mat<T, 3, 3> rotation_mat3(const vec<T, 3>& axis, T angle) {
    auto s = sin(angle), c = cos(angle);
    auto vv = normalize(axis);
    return {{c + (1 - c) * vv.x * vv.x, (1 - c) * vv.x * vv.y + s * vv.z,
                (1 - c) * vv.x * vv.z - s * vv.y},
        {(1 - c) * vv.x * vv.y - s * vv.z, c + (1 - c) * vv.y * vv.y,
            (1 - c) * vv.y * vv.z + s * vv.x},
        {(1 - c) * vv.x * vv.z + s * vv.y, (1 - c) * vv.y * vv.z - s * vv.x,
            c + (1 - c) * vv.z * vv.z}};
}

/// translation frame
template <typename T>
constexpr inline frame<T, 3> translation_frame3(const vec<T, 3>& a) {
    return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, a};
}

/// translation matrix
template <typename T>
constexpr inline mat<T, 4, 4> translation_mat4(const vec<T, 3>& a) {
    return to_mat(translation_frame3(a));
}

/// scaling frame (this is not rigid and it is only here for symmatry of
/// API)
template <typename T>
constexpr inline frame<T, 3> scaling_frame3(const vec<T, 3>& a) {
    return {{a.x, 0, 0}, {0, a.y, 0}, {0, 0, a.z}, {0, 0, 0}};
}

/// scaling matrix
template <typename T>
constexpr inline mat<T, 4, 4> scaling_mat4(const vec<T, 3>& a) {
    return to_mat(scaling_frame3(a));
}

/// rotation frame
template <typename T>
constexpr inline frame<T, 3> rotation_frame3(const vec<T, 3>& axis, T angle) {
    return {rotation_mat3(axis, angle), {0, 0, 0}};
}

/// rotation matrix
template <typename T>
constexpr inline mat<T, 4, 4> rotation_mat4(const mat<T, 3, 3>& rot) {
    return mat<T, 4, 4>{{rot.x.x, rot.x.y, rot.x.z, 0},
        {rot.y.x, rot.y.y, rot.y.z, 0}, {rot.z.x, rot.z.y, rot.z.z, 0},
        {0, 0, 0, 1}};
}

/// rotation matrix
template <typename T>
constexpr inline mat<T, 4, 4> rotation_mat4(const vec<T, 3>& axis, T angle) {
    return rotation_mat4(rotation_frame3(axis, angle).rot());
}

/// quaternion axis-angle conversion
template <typename T>
constexpr inline vec<T, 4> rotation_axisangle4(const quat<T, 4>& a) {
    auto axis = normalize(vec<T, 3>{a.x, a.y, a.z});
    auto angle = std::acos(a.w) * 2;
    return {axis.x, axis.y, axis.z, angle};
}

/// axis-angle to quaternion
template <typename T>
constexpr inline quat<T, 4> rotation_quat4(const vec<T, 4>& axis_angle) {
    auto axis = vec<T, 3>{axis_angle.x, axis_angle.y, axis_angle.z};
    auto len = lenght(axis);
    auto angle = std::atan2(len, axis_angle.w);
    if (len)
        axis /= len;
    else
        axis = {0, 0, 1};
    return {axis.x, axis.y, axis.z, angle};
}

/// quaterion to matrix conversion
template <typename T>
constexpr inline mat<T, 3, 3> rotation_mat3(const quat<T, 4>& v) {
    return {{v.w * v.w + v.x * v.x - v.y * v.y - v.z * v.z,
                (v.x * v.y + v.z * v.w) * 2, (v.z * v.x - v.y * v.w) * 2},
        {(v.x * v.y - v.z * v.w) * 2,
            v.w * v.w - v.x * v.x + v.y * v.y - v.z * v.z,
            (v.y * v.z + v.x * v.w) * 2},
        {(v.z * v.x + v.y * v.w) * 2, (v.y * v.z - v.x * v.w) * 2,
            v.w * v.w - v.x * v.x - v.y * v.y + v.z * v.z}};
}

/// rotation matrix
template <typename T>
constexpr inline mat<T, 4, 4> rotation_mat4(const quat<T, 4>& v) {
    return rotation_mat4(rotation_mat3(v));
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
/// matrix to quaternion
template <typename T>
constexpr inline quat<T, 4> rotation_quat4(const mat<T, 3, 3>& m_) {
    auto q = quat<T, 4>();
    auto m = transpose(m_);
#if 1
    auto trace = m.x.x + m.y.y + m.z.z;
    if (trace > 0) {
        float s = (T)0.5 / std::sqrt(trace + 1);
        q.w = (T)0.25 / s;
        q.x = (m.z.y - m.y.z) * s;
        q.y = (m.x.z - m.z.x) * s;
        q.z = (m.y.x - m.x.y) * s;
    } else {
        if (m.x.x > m.y.y && m.x.x > m.z.z) {
            float s = 2 * std::sqrt(max((T)0, 1 + m.x.x - m.y.y - m.z.z));
            q.w = (m.z.y - m.y.z) / s;
            q.x = (T)0.25 * s;
            q.y = (m.x.y + m.y.x) / s;
            q.z = (m.x.z + m.z.x) / s;
        } else if (m.y.y > m.z.z) {
            float s = 2 * std::sqrt(max((T)0, 1 + m.y.y - m.x.x - m.z.z));
            q.w = (m.x.z - m.z.x) / s;
            q.x = (m.x.y + m.y.x) / s;
            q.y = (T)0.25 * s;
            q.z = (m.y.z + m.z.y) / s;
        } else {
            float s = 2 * std::sqrt(max((T)0, 1 + m.z.z - m.x.x - m.y.y));
            q.w = (m.y.x - m.x.y) / s;
            q.x = (m.x.z + m.z.x) / s;
            q.y = (m.y.z + m.z.y) / s;
            q.z = (T)0.25 * s;
        }
    }

#else
    q.w = std::sqrt(max(0, 1 + m.x.x + m.y.y + m.z.z)) / 2;
    q.x = std::sqrt(max(0, 1 + m.x.x - m.y.y - m.z.z)) / 2;
    q.y = std::sqrt(max(0, 1 - m.x.x + m.y.y - m.z.z)) / 2;
    q.z = std::sqrt(max(0, 1 - m.x.x - m.y.y + m.z.z)) / 2;
    Q.x = std::copysign(q.x, m.z.y - m.y.z);
    Q.y = std::copysign(q.y, m.x.z - m.z.x);
    Q.z = std::copysign(q.z, m.y.x - m.x.y);
#endif

    return q;
}

/// OpenGL lookat frame
template <typename T>
constexpr inline frame<T, 3> lookat_frame3(
    const vec<T, 3>& eye, const vec<T, 3>& center, const vec<T, 3>& up) {
    auto w = normalize(eye - center);
    auto u = normalize(cross(up, w));
    auto v = normalize(cross(w, u));
    return {u, v, w, eye};
}

/// OpenGL lookat matrix
template <typename T>
constexpr inline mat<T, 4, 4> lookat_mat4(
    const vec<T, 3>& eye, const vec<T, 3>& center, const vec<T, 3>& up) {
    return to_mat(lookat_frame3(eye, center, up));
}

/// OpenGL frustum matrix
template <typename T>
constexpr inline mat<T, 4, 4> frustum_mat4(T l, T r, T b, T t, T n, T f) {
    return {{2 * n / (r - l), 0, 0, 0}, {0, 2 * n / (t - b), 0, 0},
        {(r + l) / (r - l), (t + b) / (t - b), -(f + n) / (f - n), -1},
        {0, 0, -2 * f * n / (f - n), 0}};
}

/// OpenGL orthographic matrix
template <typename T>
constexpr inline mat<T, 4, 4> ortho_mat4(T l, T r, T b, T t, T n, T f) {
    return {{2 / (r - l), 0, 0, 0}, {0, 2 / (t - b), 0, 0},
        {0, 0, -2 / (f - n), 0},
        {-(r + l) / (r - l), -(t + b) / (t - b), -(f + n) / (f - n), 1}};
}

/// OpenGL orthographic 2D matrix
template <typename T>
constexpr inline mat<T, 4, 4> ortho2d_mat4(T left, T right, T bottom, T top) {
    return ortho_mat4(left, right, bottom, top, -1, 1);
}

/// OpenGL/GLTF orthographic matrix
template <typename T>
constexpr inline mat<T, 4, 4> ortho_mat4(T xmag, T ymag, T near, T far) {
    return {{1 / xmag, 0, 0, 0}, {0, 1 / ymag, 0, 0},
        {0, 0, 2 / (near - far), 0}, {0, 0, (far + near) / (near - far), 1}};
}

/// OpenGL/GLTF perspective matrix
template <typename T>
constexpr inline mat<T, 4, 4> perspective_mat4(
    T fovy, T aspect, T near, T far) {
    auto tg = std::tan(fovy / 2);
    return {{1 / (aspect * tg), 0, 0, 0}, {0, 1 / tg, 0, 0},
        {0, 0, (far + near) / (near - far), -1},
        {0, 0, 2 * far * near / (near - far), 0}};
}

/// OpenGL/GLTF infinite perspective matrix
template <typename T>
constexpr inline mat<T, 4, 4> perspective_mat4(T fovy, T aspect, T near) {
    auto tg = std::tan(fovy / 2);
    return {{1 / (aspect * tg), 0, 0, 0}, {0, 1 / tg, 0, 0}, {0, 0, -1, -1},
        {0, 0, 2 * near, 0}};
}

/// Decompose an affine matrix into translation, rotation, scale.
/// Assumes there is no shear and the matrix is affine.
template <typename T>
constexpr inline void decompose_mat4(const mat<T, 4, 4>& m,
    vec<T, 3>& translation, mat<T, 3, 3>& rotation, vec<T, 3>& scale) {
    translation = {m.w.x, m.w.y, m.w.z};
    rotation.x = {m.x.x, m.x.y, m.x.z};
    rotation.y = {m.y.x, m.y.y, m.y.z};
    rotation.z = {m.z.x, m.z.y, m.z.z};
    scale = {length(rotation.x), length(rotation.y), length(rotation.z)};
    rotation = {
        normalize(rotation.x), normalize(rotation.y), normalize(rotation.z)};
}

/// Decompose an affine matrix into translation, rotation, scale.
/// Assumes there is no shear and the matrix is affine.
template <typename T>
constexpr inline void decompose_mat4(const mat<T, 4, 4>& m,
    vec<T, 3>& translation, quat<T, 4>& rotation, vec<T, 3>& scale) {
    auto rot_matrix = mat<T, 3, 3>();
    decompose_mat4(m, translation, rot_matrix, scale);
    rotation = to_quat4(rotation_mat4(rot_matrix));
}

/// Decompose an affine matrix into translation, rotation, scale.
/// Assumes there is no shear and the matrix is affine.
template <typename T>
constexpr inline mat<T, 4, 4> compose_mat4(const vec<T, 3>& translation,
    const mat<T, 3, 3>& rotation, const vec<T, 3>& scale) {
    return translation_mat4(translation) * scaling_mat4(scale) *
           rotation_mat4(rotation);
}

/// Decompose an affine matrix into translation, rotation, scale.
/// Assumes there is no shear and the matrix is affine.
template <typename T>
constexpr inline mat<T, 4, 4> compose_mat4(const vec<T, 3>& translation,
    const quat<T, 4>& rotation, const vec<T, 3>& scale) {
    return translation_mat4(translation) * scaling_mat4(scale) *
           rotation_mat4(rotation);
}

// -----------------------------------------------------------------------------
// RANDOM NUMBER GENERATION
// -----------------------------------------------------------------------------

///
/// PCG random numbers. A family of random number generators that supports
/// multiple sequences. In our code, we allocate one sequence for each sample.
/// PCG32 from http://www.pcg-random.org/
///
struct rng_pcg32 {
    uint64_t state, inc;
};

/// Next random number
constexpr inline uint32_t next(rng_pcg32* rng) {
    uint64_t oldstate = rng->state;
    rng->state = oldstate * 6364136223846793005ull + (rng->inc | 1u);
    uint32_t xorshifted = (uint32_t)(((oldstate >> 18u) ^ oldstate) >> 27u);
    uint32_t rot = oldstate >> 59u;
    return (xorshifted >> rot) | (xorshifted << ((-((int32_t)rot)) & 31));
}

/// Init a random number generator with a state state from the sequence seq.
constexpr inline void init(rng_pcg32* rng, uint64_t state, uint64_t seq) {
    rng->state = 0U;
    rng->inc = (seq << 1u) | 1u;
    next(rng);
    rng->state += state;
    next(rng);
}

/// Next random int in [0,n) range. Does not use prope math but fast.
inline int next1i(rng_pcg32* rng, int n) { return next(rng) % n; }

/// Next random float in [0,1).
inline float next1f(rng_pcg32* rng) {
#if YM_FAST_RANDFLOAT == 0
    return (float)ldexp(next(rng), -32);
#else
    constexpr const static auto scale =
        (float)(1.0 / std::numeric_limits<uint32_t>::max());
    return next(rng) * scale;
#endif
}

/// Next random float in [0,1)x[0,1).
inline vec2f next2f(rng_pcg32* rng) { return {next1f(rng), next1f(rng)}; }

/// Next random float in [0,1)x[0,1)x[0,1).
inline vec3f next3f(rng_pcg32* rng) {
    return {next1f(rng), next1f(rng), next1f(rng)};
}

// -----------------------------------------------------------------------------
// MONETACARLO SAMPLING FUNCTIONS
// -----------------------------------------------------------------------------

/// sample hemispherical direction with uniform distribution
inline vec3f sample_hemisphere(const vec2f& ruv) {
    auto z = ruv.y;
    auto r = sqrt(1 - z * z);
    auto phi = 2 * pif * ruv.x;
    return vec3f(r * cos(phi), r * sin(phi), z);
}

/// pdf for hemispherical direction with uniform distribution
inline float sample_hemisphere_pdf(const vec3f& w) {
    return (w.z <= 0) ? 0 : 1 / (2 * pif);
}

/// spherical direction with uniform distribution
inline vec3f sample_sphere(const vec2f ruv) {
    auto z = 2 * ruv.y - 1;
    auto r = sqrt(1 - z * z);
    auto phi = 2 * pif * ruv.x;
    return vec3f(r * cos(phi), r * sin(phi), z);
}

/// pdf for spherical direction with uniform distribution
inline float sample_sphere_pdf(const vec3f& w) { return 1 / (4 * pif); }

/// hemispherical direction with cosine distribution
inline vec3f sample_hemisphere_cosine(const vec2f& ruv) {
    auto z = sqrt(ruv.y);
    auto r = sqrt(1 - z * z);
    auto phi = 2 * pif * ruv.x;
    return vec3f(r * cos(phi), r * sin(phi), z);
}

/// pdf for hemispherical direction with cosine distribution
inline float sample_hemisphere_cosine_pdf(const vec3f& w) {
    return (w.z <= 0) ? 0 : w.z / pif;
}

/// hemispherical direction with cosine power distribution
inline vec3f sample_hemisphere_cospower(const vec2f& ruv, float n) {
    auto z = pow(ruv.y, 1 / (n + 1));
    auto r = sqrt(1 - z * z);
    auto phi = 2 * pif * ruv.x;
    return vec3f(r * cos(phi), r * sin(phi), z);
}

/// pdf for hemispherical direction with cosine power distribution
inline float sample_hemisphere_cospower_pdf(const vec3f& w, float n) {
    return (w.z <= 0) ? 0 : pow(w.z, n) * (n + 1) / (2 * pif);
}

/// uniform disk
inline vec3f sample_disk(const vec2f& ruv) {
    auto r = sqrt(ruv.y);
    auto phi = 2 * pif * ruv.x;
    return vec3f(cos(phi) * r, sin(phi) * r, 0);
}

/// pdf for uniform disk
inline float sample_disk_pdf() { return 1 / pif; }

/// uniform cylinder
inline vec3f sample_cylinder(const vec2f& ruv) {
    auto phi = 2 * pif * ruv.x;
    return vec3f(sin(phi), cos(phi), ruv.y * 2 - 1);
}

/// pdf for uniform cylinder
inline float sample_cylinder_pdf() { return 1 / pif; }

/// uniform triangle
inline vec2f sample_triangle(const vec2f& ruv) {
    return {1 - sqrt(ruv.x), ruv.y * sqrt(ruv.x)};
}

/// uniform triangle
inline vec3f sample_triangle(
    const vec2f& ruv, const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    auto uv = sample_triangle(ruv);
    return v0 * (1 - uv.x - uv.y) + v1 * uv.x + v2 * uv.y;
}

/// pdf for uniform triangle (triangle area)
inline float sample_triangle_pdf(
    const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    return 2 / length(cross(v1 - v0, v2 - v0));
}

/// index with uniform distribution
inline int sample_index(float r, int size) {
    return clamp((int)(r * size), 0, size - 1);
}

/// pdf for index with uniform distribution
inline float sample_index_pdf(int size) { return 1.0f / size; }

// -----------------------------------------------------------------------------
// HASHING
// -----------------------------------------------------------------------------

/// Computes the i-th term of a permutation of l values keyed by p.
/// From Correlated Multi-Jittered Sampling by Kensler @ Pixar
constexpr inline uint32_t hash_permute(uint32_t i, uint32_t n, uint32_t key) {
    uint32_t w = n - 1;
    w |= w >> 1;
    w |= w >> 2;
    w |= w >> 4;
    w |= w >> 8;
    w |= w >> 16;
    do {
        i ^= key;
        i *= 0xe170893du;
        i ^= key >> 16;
        i ^= (i & w) >> 4;
        i ^= key >> 8;
        i *= 0x0929eb3f;
        i ^= key >> 23;
        i ^= (i & w) >> 1;
        i *= 1 | key >> 27;
        i *= 0x6935fa69;
        i ^= (i & w) >> 11;
        i *= 0x74dcb303;
        i ^= (i & w) >> 2;
        i *= 0x9e501cc3;
        i ^= (i & w) >> 2;
        i *= 0xc860a3df;
        i &= w;
        i ^= i >> 5;
    } while (i >= n);
    return (i + key) % n;
}

/// Computes a float value by hashing i with a key p.
/// From Correlated Multi-Jittered Sampling by Kensler @ Pixar
constexpr inline float hash_randfloat(uint32_t i, uint32_t key) {
    i ^= key;
    i ^= i >> 17;
    i ^= i >> 10;
    i *= 0xb36534e5;
    i ^= i >> 12;
    i ^= i >> 21;
    i *= 0x93fc4795;
    i ^= 0xdf6e307f;
    i ^= i >> 17;
    i *= 1 | key >> 18;
    return i * (1.0f / 4294967808.0f);
}

/// 64 bit integer hash. Public domain code.
constexpr inline uint64_t hash_uint64(uint64_t a) {
    a = (~a) + (a << 21);  // a = (a << 21) - a - 1;
    a ^= (a >> 24);
    a += (a << 3) + (a << 8);  // a * 265
    a ^= (a >> 14);
    a += (a << 2) + (a << 4);  // a * 21
    a ^= (a >> 28);
    a += (a << 31);
    return a;
}

/// 64-to-32 bit integer hash. Public domain code.
constexpr inline uint32_t hash_uint64_32(uint64_t a) {
    a = (~a) + (a << 18);  // a = (a << 18) - a - 1;
    a ^= (a >> 31);
    a *= 21;  // a = (a + (a << 2)) + (a << 4);
    a ^= (a >> 11);
    a += (a << 6);
    a ^= (a >> 22);
    return (uint32_t)a;
}

/// Combines two 64 bit hashes as in boost::hash_combine
constexpr inline size_t hash_combine(size_t a, size_t b) {
    return a ^ (b + 0x9e3779b9 + (a << 6) + (a >> 2));
}

/// Hash a vector with hash_combine() and std::hash
template <typename T, int N>
constexpr inline int hash_vec(const vec<T, N>& v) {
    std::hash<T> Th;
    int h = 0;
    for (auto i = 0; i < N; i++) {
        h ^= (Th(v[i]) + 0x9e3779b9 + (h << 6) + (h >> 2));
    }
    return h;
}

// -----------------------------------------------------------------------------
// GEOMETRY UTILITIES
// -----------------------------------------------------------------------------

/// line tangent
template <typename T>
constexpr inline vec<T, 3> line_tangent(
    const vec<T, 3>& v0, const vec<T, 3>& v1) {
    return normalize(v1 - v0);
}

/// line length
template <typename T>
constexpr inline T line_length(const vec<T, 3>& v0, const vec<T, 3>& v1) {
    return length(v1 - v0);
}

/// triangle normal
template <typename T>
constexpr inline vec<T, 3> triangle_normal(
    const vec<T, 3>& v0, const vec<T, 3>& v1, const vec<T, 3>& v2) {
    return normalize(cross(v1 - v0, v2 - v0));
}

/// triangle area
template <typename T>
constexpr inline T triangle_area(
    const vec<T, 3>& v0, const vec<T, 3>& v1, const vec<T, 3>& v2) {
    return length(cross(v1 - v0, v2 - v0)) / 2;
}

/// quad area
template <typename T>
constexpr inline T quad_area(const vec<T, 3>& v0, const vec<T, 3>& v1,
    const vec<T, 3>& v2, const vec<T, 3>& v3) {
    return triangle_area(v0, v1, v3) + triangle_area(v3, v2, v1);
}

/// tetrahedron volume
template <typename T>
constexpr inline T tetrahedron_volume(const vec<T, 3>& v0, const vec<T, 3>& v1,
    const vec<T, 3>& v2, const vec<T, 3>& v3) {
    return dot(cross(v1 - v0, v2 - v0), v3 - v0) / 6;
}

//
// Triangle tangent and bitangent from uv (not othornormalized with themselfves
// not the normal). Follows the definition in
// http://www.terathon.com/code/tangent.html and
// https://gist.github.com/aras-p/2843984
template <typename T>
constexpr inline std::pair<vec<T, 3>, vec<T, 3>> triangle_tangents_fromuv(
    const vec<T, 3>& v0, const vec<T, 3>& v1, const vec<T, 3>& v2,
    const vec<T, 2>& uv0, const vec<T, 2>& uv1, const vec<T, 2>& uv2) {
    // normal points up from texture space
    auto p = v1 - v0;
    auto q = v2 - v0;
    auto s = vec<T, 2>{uv1.x - uv0.x, uv2.x - uv0.x};
    auto t = vec<T, 2>{uv1.y - uv0.y, uv2.y - uv0.y};
    auto div = s.x * t.y - s.y * t.x;

    if (div != 0) {
        auto tu = vec<T, 3>{t.y * p.x - t.x * q.x, t.y * p.y - t.x * q.y,
                      t.y * p.z - t.x * q.z} /
                  div;
        auto tv = vec<T, 3>{s.x * q.x - s.y * p.x, s.x * q.y - s.y * p.y,
                      s.x * q.z - s.y * p.z} /
                  div;
        return {tu, tv};
    } else {
        return {{1, 0, 0}, {0, 1, 0}};
    }
}

/// triangle baricentric interpolation
template <typename T, typename T1>
constexpr inline T blerp(const T& a, const T& b, const T& c, const T1& w) {
    return a * w.x + b * w.y + c * w.z;
}

///
/// Compute smoothed tangents (for lines).
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - npoints/points: array of point indices
/// - nlines/lines: array of point indices
/// - ntriangles/triangles: array of point indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - tang: preallocated array of computed normals
///
inline void compute_tangents(int nlines, const vec2i* lines, int nverts,
    const vec3f* pos, vec3f* tang, bool weighted = true) {
    // clear tangents
    for (auto i = 0; i < nverts; i++) tang[i] = zero3f;

    // handle lines
    for (auto i = 0; i < nlines; i++) {
        auto line = lines[i];
        auto n = pos[line.y] - pos[line.x];
        if (!weighted) n = normalize(n);
        tang[line.x] += n;
        tang[line.y] += n;
    }

    // normalize result
    for (auto i = 0; i < nverts; i++) tang[i] = normalize(tang[i]);
}

///
/// Compute smoothed tangents.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - nlines/lines: array of point indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - tang: array of computed tangents
///
inline void compute_tangents(const std::vector<vec2i>& lines,
    const std::vector<vec3f>& pos, std::vector<vec3f>& tang,
    bool weighted = true) {
    tang.resize(pos.size());
    compute_tangents((int)lines.size(), lines.data(), (int)pos.size(),
        pos.data(), tang.data(), weighted);
}

///
/// Compute smoothed normals.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - ntriangles/triangles: array of triangle indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - norm: preallocated array of computed normals
///
inline void compute_normals(int ntriangles, const vec3i* triangles, int nverts,
    const vec3f* pos, vec3f* norm, bool weighted = true) {
    // clear normals
    for (auto i = 0; i < nverts; i++) norm[i] = zero3f;

    // handle triangles
    for (auto i = 0; i < ntriangles; i++) {
        auto triangle = triangles[i];
        auto n = cross(pos[triangle.y] - pos[triangle.x],
            pos[triangle.z] - pos[triangle.x]);
        if (!weighted) n = normalize(n);
        norm[triangle.x] += n;
        norm[triangle.y] += n;
        norm[triangle.z] += n;
    }

    // normalize result
    for (auto i = 0; i < nverts; i++) norm[i] = normalize(norm[i]);
}

///
/// Compute smoothed normals.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - triangles: array of triangle indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - norm: array of computed normals
///
inline void compute_normals(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    bool weighted = true) {
    norm.resize(pos.size());
    compute_normals((int)triangles.size(), triangles.data(), (int)pos.size(),
        pos.data(), norm.data(), weighted);
}

///
/// Compute smoothed normals.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - nquads/quads: array of point indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - norm: preallocated array of computed normals
///
inline void compute_normals(int nquads, const vec4i* quads, int nverts,
    const vec3f* pos, vec3f* norm, bool weighted = true) {
    // clear normals
    for (auto i = 0; i < nverts; i++) norm[i] = zero3f;

    // handle triangles
    for (auto i = 0; i < nquads; i++) {
        auto quad = quads[i];
        auto n = cross(pos[quad.y] - pos[quad.x], pos[quad.w] - pos[quad.x]) +
                 cross(pos[quad.w] - pos[quad.z], pos[quad.x] - pos[quad.z]);
        if (!weighted) n = normalize(n);
        norm[quad.x] += n;
        norm[quad.y] += n;
        norm[quad.z] += n;
        norm[quad.w] += n;
    }

    // normalize result
    for (auto i = 0; i < nverts; i++) norm[i] = normalize(norm[i]);
}

///
/// Compute smoothed normals.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - nquads/quads: array of point indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - norm: array of computed normals
///
inline void compute_normals(const std::vector<vec4i>& quads,
    const std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    bool weighted = true) {
    norm.resize(pos.size());
    compute_normals((int)quads.size(), quads.data(), (int)pos.size(),
        pos.data(), norm.data(), weighted);
}

///
/// Compute tangent frame for triangle mesh. Tangent space is defined by
/// a four component vector. The first three components are the tangent
/// with respect to the U texcoord. The fourth component is the sign of the
/// tangent wrt the V texcoord. Tangent frame is useful in normal mapping.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - ntriangles/triangles: array of point indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - tangsp: preallocated array of computed tangent space
///
inline void compute_tangent_frame(int ntriangles, const vec3i* triangles,
    int nverts, const vec3f* pos, const vec3f* norm, const vec2f* texcoord,
    vec4f* tangsp, bool weighted = true) {
    auto tangu = std::vector<vec3f>((unsigned int)nverts, {0, 0, 0});
    auto tangv = std::vector<vec3f>((unsigned int)nverts, {0, 0, 0});

    for (auto i = 0; i < ntriangles; i++) {
        auto t = triangles[i];
        auto tutv = triangle_tangents_fromuv(pos[t.x], pos[t.y], pos[t.z],
            texcoord[t.x], texcoord[t.y], texcoord[t.z]);
        if (!weighted) tutv = {normalize(tutv.first), normalize(tutv.second)};
        tangu[t.x] += tutv.first;
        tangu[t.y] += tutv.first;
        tangu[t.z] += tutv.first;
        tangv[t.x] += tutv.second;
        tangv[t.y] += tutv.second;
        tangv[t.z] += tutv.second;
    }

    for (auto i = 0; i < nverts; i++) {
        tangu[i] = normalize(tangu[i]);
        tangv[i] = normalize(tangv[i]);
    }

    for (auto i = 0; i < nverts; i++) {
        tangu[i] = orthonormalize(tangu[i], norm[i]);
        auto s = (dot(cross(norm[i], tangu[i]), tangv[i]) < 0) ? -1.0f : 1.0f;
        tangsp[i] = {tangu[i].x, tangu[i].y, tangu[i].z, s};
    }
}

///
/// Compute tangent frame for triangle mesh. Tangent space is defined by
/// a four component vector. The first three components are the tangent
/// with respect to the U texcoord. The fourth component is the sign of the
/// tangent wrt the V texcoord. Tangent frame is useful in normal mapping.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - ntriangles/triangles: array of point indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - tangsp: array of computed tangent space
///
inline void compute_tangent_frame(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& pos, const std::vector<vec3f>& norm,
    const std::vector<vec2f>& texcoord, std::vector<vec4f>& tangsp,
    bool weighted = true) {
    tangsp.resize(tangsp.size());
    compute_tangent_frame((int)triangles.size(), triangles.data(),
        (int)pos.size(), pos.data(), norm.data(), texcoord.data(),
        tangsp.data(), weighted);
}

/// Apply skinning
inline void compute_skinning(int nverts, const vec3f* pos, const vec3f* norm,
    const vec4f* weights, const vec4i* joints, const mat4f* xforms,
    vec3f* skinned_pos, vec3f* skinned_norm) {
    for (auto i = 0; i < nverts; i++) {
        skinned_pos[i] =
            transform_point(xforms[joints[i].x], pos[i]) * weights[i].x +
            transform_point(xforms[joints[i].y], pos[i]) * weights[i].y +
            transform_point(xforms[joints[i].z], pos[i]) * weights[i].z +
            transform_point(xforms[joints[i].w], pos[i]) * weights[i].w;
    }
    for (auto i = 0; i < nverts; i++) {
        skinned_norm[i] = normalize(
            transform_direction(xforms[joints[i].x], norm[i]) * weights[i].x +
            transform_direction(xforms[joints[i].y], norm[i]) * weights[i].y +
            transform_direction(xforms[joints[i].z], norm[i]) * weights[i].z +
            transform_direction(xforms[joints[i].w], norm[i]) * weights[i].w);
    }
}

/// Apply skinning
inline void compute_skinning(int nverts, const vec3f* pos, const vec3f* norm,
    const vec4f* weights, const vec4i* joints, const frame3f* xforms,
    vec3f* skinned_pos, vec3f* skinned_norm) {
    for (auto i = 0; i < nverts; i++) {
        skinned_pos[i] =
            transform_point(xforms[joints[i].x], pos[i]) * weights[i].x +
            transform_point(xforms[joints[i].y], pos[i]) * weights[i].y +
            transform_point(xforms[joints[i].z], pos[i]) * weights[i].z +
            transform_point(xforms[joints[i].w], pos[i]) * weights[i].w;
    }
    for (auto i = 0; i < nverts; i++) {
        skinned_norm[i] = normalize(
            transform_direction(xforms[joints[i].x], norm[i]) * weights[i].x +
            transform_direction(xforms[joints[i].y], norm[i]) * weights[i].y +
            transform_direction(xforms[joints[i].z], norm[i]) * weights[i].z +
            transform_direction(xforms[joints[i].w], norm[i]) * weights[i].w);
    }
}

/// Apply skinning
inline void compute_skinning(const std::vector<vec3f>& pos,
    const std::vector<vec3f>& norm, const std::vector<vec4f>& weights,
    const std::vector<vec4i>& joints, const std::vector<mat4f>& xforms,
    std::vector<vec3f>& skinned_pos, std::vector<vec3f>& skinned_norm) {
    skinned_pos.resize(pos.size());
    skinned_norm.resize(norm.size());
    compute_skinning((int)pos.size(), pos.data(), norm.data(), weights.data(),
        joints.data(), xforms.data(), skinned_pos.data(), skinned_norm.data());
}

/// Apply skinning
inline void compute_skinning(const std::vector<vec3f>& pos,
    const std::vector<vec3f>& norm, const std::vector<vec4f>& weights,
    const std::vector<vec4i>& joints, const std::vector<frame3f>& xforms,
    std::vector<vec3f>& skinned_pos, std::vector<vec3f>& skinned_norm) {
    skinned_pos.resize(pos.size());
    skinned_norm.resize(norm.size());
    compute_skinning((int)pos.size(), pos.data(), norm.data(), weights.data(),
        joints.data(), xforms.data(), skinned_pos.data(), skinned_norm.data());
}

/// Apply skinning as specified in Khronos glTF
inline void compute_matrix_skinning(int nverts, const vec3f* pos,
    const vec3f* norm, const vec4f* weights, const vec4i* joints,
    const mat4f* xforms, vec3f* skinned_pos, vec3f* skinned_norm) {
    for (auto i = 0; i < nverts; i++) {
        auto xform = xforms[joints[i].x] * weights[i].x +
                     xforms[joints[i].y] * weights[i].y +
                     xforms[joints[i].z] * weights[i].z +
                     xforms[joints[i].w] * weights[i].w;
        skinned_pos[i] = transform_point(xform, pos[i]);
        skinned_norm[i] = normalize(transform_direction(xform, norm[i]));
    }
}

/// Apply skinning as specified in Khronos glTF
inline void compute_matrix_skinning(const std::vector<vec3f>& pos,
    const std::vector<vec3f>& norm, const std::vector<vec4f>& weights,
    const std::vector<vec4i>& joints, const std::vector<mat4f>& xforms,
    std::vector<vec3f>& skinned_pos, std::vector<vec3f>& skinned_norm) {
    skinned_pos.resize(pos.size());
    skinned_norm.resize(norm.size());
    compute_matrix_skinning((int)pos.size(), pos.data(), norm.data(),
        weights.data(), joints.data(), xforms.data(), skinned_pos.data(),
        skinned_norm.data());
}

///
/// Create an array of edges.
///
inline std::vector<vec2i> make_edges(const std::vector<vec2i>& lines,
    const std::vector<vec3i>& triangles, const std::vector<vec4i>& quads) {
    auto edges = std::vector<vec2i>();
    auto edge_map = std::unordered_map<vec2i, int, vec_hash<int, 2>>();

    auto add_edge = [&edges, &edge_map](const vec2i& e) {
        auto ee = vec2i{min(e.x, e.y), max(e.x, e.y)};
        if (edge_map.find(ee) != edge_map.end()) return;
        // split in two to avoid undefined behaviour
        auto size = (int)edges.size();
        edge_map[ee] = size;
        edges.push_back(ee);
    };

    for (auto l : lines) add_edge(l);
    for (auto t : triangles) {
        add_edge({t.x, t.y});
        add_edge({t.y, t.z});
        add_edge({t.z, t.x});
    }
    for (auto t : quads) {
        add_edge({t.x, t.y});
        add_edge({t.y, t.z});
        add_edge({t.z, t.w});
        add_edge({t.w, t.x});
    }

    return edges;
}

///
/// Convert quads to triangles
///
inline std::vector<vec3i> convert_quads_to_triangles(
    const std::vector<vec4i>& quads) {
    auto triangles = std::vector<vec3i>();
    triangles.reserve(quads.size() * 2);
    for (auto& q : quads) {
        triangles.push_back({q.x, q.y, q.w});
        triangles.push_back({q.z, q.w, q.y});
    }
    return triangles;
}

///
/// Convert quads to triangles with a diamond-like topology.
/// Quads have to be consecutive one row after another.
///
inline std::vector<vec3i> convert_quads_to_triangles(
    const std::vector<vec4i>& quads, int row_length) {
    auto triangles = std::vector<vec3i>(quads.size() * 2);
    for (auto i = 0; i < quads.size(); i++) {
        auto q = quads[i];
        triangles[i * 2 + 0] = {q.x, q.y, q.w};
        triangles[i * 2 + 1] = {q.z, q.w, q.y};
    }
    return triangles;
}

///
/// Edge map structure
///
struct edge_map {
    /// an empty edge map
    edge_map();

    /// initialize the edge map with triangles
    edge_map(const std::vector<vec3i>& triangles) {
        for (auto& t : triangles) {
            add_edge({t.x, t.y});
            add_edge({t.y, t.z});
            add_edge({t.z, t.x});
        }
    }

    /// initialize the edge map with quads
    edge_map(const std::vector<vec4i>& quads) {
        for (auto& t : quads) {
            if (t.z == t.w) {
                add_edge({t.x, t.y});
                add_edge({t.y, t.z});
                add_edge({t.z, t.x});
            } else {
                add_edge({t.x, t.y});
                add_edge({t.y, t.z});
                add_edge({t.z, t.w});
                add_edge({t.w, t.x});
            }
        }
    }

    /// size
    auto size() const { return _map.size(); }

    /// iteration
    auto begin() const { return _map.begin(); }
    /// iteration
    auto end() const { return _map.end(); }
    /// iteration
    auto begin() { return _map.begin(); }
    /// iteration
    auto end() { return _map.end(); }

    /// add an edge to the edge map if not already present
    int add_edge(const vec2i& e) {
        auto ee = vec2i{min(e.x, e.y), max(e.x, e.y)};
        auto it = _map.find(ee);
        if (it != _map.end()) return it->second;
        // split in two to avoid undefined behaviour
        auto eid = (int)size();
        _map[ee] = eid;
        return eid;
    }

    /// add an edge to the edge map if not already present
    int operator[](const vec2i& e) { return add_edge(e); }

    /// get the edge index
    int at(const vec2i& e) const {
        return _map.at({min(e.x, e.y), max(e.x, e.y)});
    }

    /// get all edges
    const std::vector<vec2i> get_edges() const {
        auto edges = std::vector<vec2i>();
        edges.reserve(size());
        for (auto& kv : _map) edges.push_back(kv.first);
        return edges;
    }

    // implementation -------------------------------------------
   private:
    std::unordered_map<vec2i, int, vec_hash<int, 2>> _map = {};
};

///
/// Tesselate lines by splitting them in half, giving two output segments
/// for each input segment.
///
inline void tesselate_lines(std::vector<vec2i>& lines, std::vector<vec3f>& pos,
    std::vector<vec3f>& tang, std::vector<vec2f>& texcoord,
    std::vector<vec4f>& color, std::vector<float>& radius,
    bool normalize_tangents = true) {
    auto nverts = (int)pos.size();
    auto add_vertices = [&lines, nverts](auto& vert) {
        if (vert.empty()) return;
        vert.resize(vert.size() + lines.size());
        for (auto eid = 0; eid < (int)lines.size(); eid++) {
            auto e = lines[eid];
            vert[nverts + eid] = (vert[e.x] + vert[e.y]) / 2.0f;
        }
    };

    add_vertices(pos);
    add_vertices(tang);
    add_vertices(texcoord);
    add_vertices(color);
    add_vertices(radius);

    if (normalize_tangents) {
        for (auto& n : tang) n = normalize(n);
    }

    auto edge_id = [nverts](int lid) { return nverts + lid; };

    auto tlines = std::vector<vec2i>(lines.size() * 2);
    for (auto lid = 0; lid < lines.size(); lid++) {
        auto& l = lines[lid];
        tlines[lid * 2 + 0] = {l.x, edge_id(lid)};
        tlines[lid * 2 + 1] = {edge_id(lid), l.y};
    }
    std::swap(lines, tlines);
}

///
/// Tesselate triangles by splitting edges, producing 4 triangles per input
/// triangle.
///
inline void tesselate_triangles(std::vector<vec3i>& triangles,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord, std::vector<vec4f>& color,
    std::vector<float>& radius, bool normalize_normals = true) {
    auto nverts = (int)max(pos.size(),
        max(norm.size(),
            max(texcoord.size(), max(color.size(), radius.size()))));

    auto emap = edge_map(triangles);
    for (auto& e_kv : emap) e_kv.second += nverts;

    auto add_vertices = [&emap](auto& vert) {
        if (vert.empty()) return;
        vert.resize(vert.size() + emap.size());
        for (const auto& e_kv : emap) {
            const auto& e = e_kv.first;
            vert[e_kv.second] = (vert[e.x] + vert[e.y]) / 2.0f;
        }
    };

    add_vertices(pos);
    add_vertices(norm);
    add_vertices(texcoord);
    add_vertices(color);
    add_vertices(radius);

    if (normalize_normals) {
        for (auto& n : norm) n = normalize(n);
    }

    auto ttriangles = std::vector<vec3i>();
    ttriangles.reserve(triangles.size() * 4);
    for (auto& t : triangles) {
        ttriangles.push_back({t.x, emap.at({t.x, t.y}), emap.at({t.z, t.x})});
        ttriangles.push_back({t.y, emap.at({t.y, t.z}), emap.at({t.x, t.y})});
        ttriangles.push_back({t.z, emap.at({t.z, t.x}), emap.at({t.y, t.z})});
        ttriangles.push_back(
            {emap.at({t.x, t.y}), emap.at({t.y, t.z}), emap.at({t.z, t.x})});
    }
    std::swap(triangles, ttriangles);
}

///
/// Tesselate quads by splitting faces, producing 4 quads per input quad and
/// 3 quads per input triangle. Handles degenerate quads gracefully.
///
inline void tesselate_quads(std::vector<vec4i>& quads, std::vector<vec3f>& pos,
    std::vector<vec3f>& norm, std::vector<vec2f>& texcoord,
    std::vector<vec4f>& color, std::vector<float>& radius,
    bool normalize_normals = true) {
    auto nverts = (int)max(pos.size(),
        max(norm.size(),
            max(texcoord.size(), max(color.size(), radius.size()))));

    auto emap = edge_map(quads);
    for (auto& e_kv : emap) e_kv.second += nverts;

    auto fmap = std::unordered_map<vec4i, int, vec_hash<int, 4>>();
    for (auto& f : quads) fmap[f] = fmap.size() + nverts + emap.size();

    auto add_vertices = [&emap, &fmap](auto& vert) {
        if (vert.empty()) return;
        vert.resize(vert.size() + emap.size() + fmap.size());
        for (const auto& e_kv : emap) {
            const auto& e = e_kv.first;
            vert[e_kv.second] = (vert[e.x] + vert[e.y]) / 2.0f;
        }
        for (const auto& f_kv : fmap) {
            const auto& f = f_kv.first;
            if (f.z != f.w) {
                vert[f_kv.second] =
                    (vert[f.x] + vert[f.y] + vert[f.z] + vert[f.w]) / 4.0f;
            } else {
                vert[f_kv.second] = (vert[f.x] + vert[f.y] + vert[f.z]) / 3.0f;
            }
        }

    };

    add_vertices(pos);
    add_vertices(norm);
    add_vertices(texcoord);
    add_vertices(color);
    add_vertices(radius);

    if (normalize_normals) {
        for (auto& n : norm) n = normalize(n);
    }

    auto tquads = std::vector<vec4i>();
    tquads.reserve(quads.size() * 4);
    for (auto& f : quads) {
        if (f.z != f.w) {
            tquads.push_back(
                {f.x, emap.at({f.x, f.y}), fmap.at(f), emap.at({f.w, f.x})});
            tquads.push_back(
                {f.y, emap.at({f.y, f.z}), fmap.at(f), emap.at({f.x, f.y})});
            tquads.push_back(
                {f.z, emap.at({f.z, f.w}), fmap.at(f), emap.at({f.y, f.z})});
            tquads.push_back(
                {f.w, emap.at({f.w, f.x}), fmap.at(f), emap.at({f.z, f.w})});
        } else {
            tquads.push_back(
                {f.x, emap.at({f.x, f.y}), fmap.at(f), emap.at({f.z, f.x})});
            tquads.push_back(
                {f.y, emap.at({f.y, f.z}), fmap.at(f), emap.at({f.x, f.y})});
            tquads.push_back(
                {f.z, emap.at({f.z, f.x}), fmap.at(f), emap.at({f.y, f.z})});
        }
    }
    tquads.shrink_to_fit();
    std::swap(quads, tquads);
}

///
/// Tesselate quads by splitting faces, producing 4 quads per input quad and
/// 3 quads per input triangle. Handles degenerate quads gracefully.
///
inline void tesselate_catmullclark(std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord, std::vector<vec4f>& color,
    std::vector<float>& radius, bool normalize_normals = true) {
    auto nverts = (int)max(pos.size(),
        max(norm.size(),
            max(texcoord.size(), max(color.size(), radius.size()))));

    auto emap = edge_map(quads);
    for (auto& e_kv : emap) e_kv.second += nverts;

    auto fmap = std::unordered_map<vec4i, int, vec_hash<int, 4>>();
    for (auto& f : quads) fmap[f] = fmap.size() + nverts + emap.size();

    auto add_vertices = [&emap, &fmap](auto& vert) {
        if (vert.empty()) return;
        vert.resize(vert.size() + emap.size() + fmap.size());
        for (const auto& e_kv : emap) {
            const auto& e = e_kv.first;
            vert[e_kv.second] = (vert[e.x] + vert[e.y]) / 2.0f;
        }
        for (const auto& f_kv : fmap) {
            const auto& f = f_kv.first;
            if (f.z != f.w) {
                vert[f_kv.second] =
                    (vert[f.x] + vert[f.y] + vert[f.z] + vert[f.w]) / 4.0f;
            } else {
                vert[f_kv.second] = (vert[f.x] + vert[f.y] + vert[f.z]) / 3.0f;
            }
        }

    };

    add_vertices(pos);
    add_vertices(norm);
    add_vertices(texcoord);
    add_vertices(color);
    add_vertices(radius);

    if (normalize_normals) {
        for (auto& n : norm) n = normalize(n);
    }

    auto tquads = std::vector<vec4i>();
    tquads.reserve(quads.size() * 4);
    for (auto& f : quads) {
        if (f.z != f.w) {
            tquads.push_back(
                {f.x, emap.at({f.x, f.y}), fmap.at(f), emap.at({f.w, f.x})});
            tquads.push_back(
                {f.y, emap.at({f.y, f.z}), fmap.at(f), emap.at({f.x, f.y})});
            tquads.push_back(
                {f.z, emap.at({f.z, f.w}), fmap.at(f), emap.at({f.y, f.z})});
            tquads.push_back(
                {f.w, emap.at({f.w, f.x}), fmap.at(f), emap.at({f.z, f.w})});
        } else {
            tquads.push_back(
                {f.x, emap.at({f.x, f.y}), fmap.at(f), emap.at({f.z, f.x})});
            tquads.push_back(
                {f.y, emap.at({f.y, f.z}), fmap.at(f), emap.at({f.x, f.y})});
            tquads.push_back(
                {f.z, emap.at({f.z, f.x}), fmap.at(f), emap.at({f.y, f.z})});
        }
    }
    tquads.shrink_to_fit();
    std::swap(quads, tquads);

    // correct vertex data
    auto smooth_vertices = [&quads](auto& vert, auto zero) {
        if (vert.empty()) return;
        // averaging pass ----------------------------------
        // TEMPLATE PROBLEM HERE - FIX IT WITHOUT ELEGANCE
        // auto avg = std::vector<T>(vert.size(), T());
        auto avg = vert;
        for (auto& v : avg) v = zero;
        auto count = std::vector<int>(vert.size(), 0);
        for (auto& f : quads) {
            auto fc = (vert[f.x] + vert[f.y] + vert[f.z] + vert[f.w]) / 4.0f;
            for (auto vid : f) avg[vid] += fc;
            for (auto vid : f) count[vid] += 1;
        }
        for (auto i = 0; i < vert.size(); i++) { avg[i] /= (float)count[i]; }

        // correction pass ----------------------------------
        // p = p + (avg_p - p) * (4/avg_count)
        for (auto i = 0; i < vert.size(); i++) {
            vert[i] = vert[i] + (avg[i] - vert[i]) * (4.0f / count[i]);
        }
    };

    smooth_vertices(pos, zero3f);
    smooth_vertices(norm, zero3f);
    smooth_vertices(texcoord, zero2f);
    smooth_vertices(color, zero4f);
    smooth_vertices(radius, 0.0f);

    if (normalize_normals) {
        for (auto& n : norm) n = normalize(n);
    }
}

///
/// Generate a parametric surface with callbacks.
///
/// Parameters:
/// - usteps: subdivisions in u
/// - vsteps: subdivisions in v
/// - as_triangles: whether to use triangles or quads
/// - pos_fn: pos callbacks (vec2f -> vec3f)
/// - norm_fn: norm callbacks (vec2f -> vec3f)
/// - texcoord_fn: texcoord callbacks (vec2f -> vec2f)
///
/// Out Parameters:
/// - triangles: element array
/// - quads: element array
/// - pos/norm/texcoord: vertex position/normal/texcoords
///
template <typename PosFunc, typename NormFunc, typename TexcoordFunc>
inline void make_faces(int usteps, int vsteps, bool as_triangles,
    std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord, const PosFunc& pos_fn,
    const NormFunc& norm_fn, const TexcoordFunc& texcoord_fn) {
    auto vid = [usteps](int i, int j) { return j * (usteps + 1) + i; };
    pos.resize((usteps + 1) * (vsteps + 1));
    norm.resize((usteps + 1) * (vsteps + 1));
    texcoord.resize((usteps + 1) * (vsteps + 1));
    for (auto j = 0; j <= vsteps; j++) {
        for (auto i = 0; i <= usteps; i++) {
            auto uv = vec2f{i / (float)usteps, j / (float)vsteps};
            pos[vid(i, j)] = pos_fn(uv);
            norm[vid(i, j)] = norm_fn(uv);
            texcoord[vid(i, j)] = texcoord_fn(uv);
        }
    }

    if (as_triangles) {
        triangles.resize(usteps * vsteps * 2);
        for (auto j = 0; j < vsteps; j++) {
            for (auto i = 0; i < usteps; i++) {
                auto& f1 = triangles[(j * usteps + i) * 2 + 0];
                auto& f2 = triangles[(j * usteps + i) * 2 + 1];
                if ((i + j) % 2) {
                    f1 = {vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)};
                    f2 = {vid(i + 1, j + 1), vid(i, j + 1), vid(i, j)};
                } else {
                    f1 = {vid(i, j), vid(i + 1, j), vid(i, j + 1)};
                    f2 = {vid(i + 1, j + 1), vid(i, j + 1), vid(i + 1, j)};
                }
            }
        }
    } else {
        quads.resize(usteps * vsteps);
        for (auto j = 0; j < vsteps; j++) {
            for (auto i = 0; i < usteps; i++) {
                quads[j * usteps + i] = {
                    vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)};
            }
        }
    }
}

///
/// Generate a parametric surface with callbacks.
///
/// Parameters:
/// - usteps: subdivisions in u
/// - vsteps: subdivisions in v
/// - pos_fn: pos callbacks (vec2f -> vec3f)
/// - norm_fn: norm callbacks (vec2f -> vec3f)
/// - texcoord_fn: texcoord callbacks (vec2f -> vec2f)
///
/// Out Parameters:
/// - triangles: element array
/// - pos/norm/texcoord: vertex position/normal/texcoords
///
template <typename PosFunc, typename NormFunc, typename TexcoordFunc>
inline void make_triangles(int usteps, int vsteps,
    std::vector<vec3i>& triangles, std::vector<vec3f>& pos,
    std::vector<vec3f>& norm, std::vector<vec2f>& texcoord,
    const PosFunc& pos_fn, const NormFunc& norm_fn,
    const TexcoordFunc& texcoord_fn) {
    auto quads = std::vector<vec4i>();
    return make_faces(usteps, vsteps, true, triangles, quads, pos, norm,
        texcoord, pos_fn, norm_fn, texcoord_fn);
}

///
/// Generate a parametric surface with callbacks.
///
/// Parameters:
/// - usteps: subdivisions in u
/// - vsteps: subdivisions in v
/// - pos_fn: pos callbacks (vec2f -> vec3f)
/// - norm_fn: norm callbacks (vec2f -> vec3f)
/// - texcoord_fn: texcoord callbacks (vec2f -> vec2f)
///
/// Out Parameters:
/// - quads: element array
/// - pos/norm/texcoord: vertex position/normal/texcoords
///
template <typename PosFunc, typename NormFunc, typename TexcoordFunc>
inline void make_quads(int usteps, int vsteps, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord, const PosFunc& pos_fn,
    const NormFunc& norm_fn, const TexcoordFunc& texcoord_fn) {
    auto triangles = std::vector<vec3i>();
    return make_faces(usteps, vsteps, false, triangles, quads, pos, norm,
        texcoord, pos_fn, norm_fn, texcoord_fn);
}

///
/// Generate parametric lines with callbacks.
///
/// Parameters:
/// - usteps: subdivisions in u
/// - num: number of lines
/// - pos_fn: pos callbacks ((int, float) -> vec3f)
/// - tang_fn: tangent callbacks ((int, float) -> vec3f)
/// - texcoord_fn: texcoord callbacks ((int, float) -> vec2f)
/// - radius_fn: radius callbacks ((int, float) -> float)
///
/// Out Parameters:
/// - lines: element array
/// - pos/tang/texcoord/radius: vertex position/tangent/texcoords/radius
///
template <typename PosFunc, typename TangFunc, typename TexcoordFunc,
    typename RadiusFunc>
inline void make_lines(int num, int usteps, std::vector<vec2i>& lines,
    std::vector<vec3f>& pos, std::vector<vec3f>& tang,
    std::vector<vec2f>& texcoord, std::vector<float>& radius,
    const PosFunc& pos_fn, const TangFunc& tang_fn,
    const TexcoordFunc& texcoord_fn, const RadiusFunc& radius_fn) {
    auto vid = [usteps](int i, int j) { return j * (usteps + 1) + i; };
    pos.resize((usteps + 1) * num);
    tang.resize((usteps + 1) * num);
    texcoord.resize((usteps + 1) * num);
    radius.resize((usteps + 1) * num);
    for (auto j = 0; j < num; j++) {
        for (auto i = 0; i <= usteps; i++) {
            auto u = i / (float)usteps;
            ;
            pos[vid(i, j)] = pos_fn(j, u);
            tang[vid(i, j)] = tang_fn(j, u);
            texcoord[vid(i, j)] = texcoord_fn(j, u);
            radius[vid(i, j)] = radius_fn(j, u);
        }
    }

    lines.resize(usteps * num);
    for (int j = 0; j < num; j++) {
        for (int i = 0; i < usteps; i++) {
            lines[j * usteps + i] = {vid(i, j), vid(i + 1, j)};
        }
    }
}

///
/// Generate a parametric point set. Mostly here for completeness.
///
/// Parameters:
/// - num: number of points
/// - pos_fn: pos callbacks (int -> vec3f)
/// - norm_fn: norm callbacks (int -> vec3f)
/// - texcoord_fn: texcoord callbacks (int -> vec2f)
/// - radius_fn: radius callbacks (int -> float)
///
/// Out Parameters:
/// - points: element array
/// - pos/norm/texcoord/radius: vertex position/normal/texcoords/radius
///
template <typename PosFunc, typename NormFunc, typename TexcoordFunc,
    typename RadiusFunc>
inline void make_points(int num, std::vector<int>& points,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord, std::vector<float>& radius,
    const PosFunc& pos_fn, const NormFunc& norm_fn,
    const TexcoordFunc& texcoord_fn, const RadiusFunc& radius_fn) {
    pos.resize(num);
    norm.resize(num);
    texcoord.resize(num);
    radius.resize(num);
    for (auto i = 0; i < num; i++) {
        pos[i] = pos_fn(i);
        norm[i] = norm_fn(i);
        texcoord[i] = texcoord_fn(i);
        radius[i] = radius_fn(i);
    }

    points.resize(num);
    for (auto i = 0; i < num; i++) points[i] = i;
}

///
/// Merge a triangle mesh into another.
///
inline void merge_triangles(std::vector<vec3i>& triangles,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord, const std::vector<vec3i>& mtriangles,
    const std::vector<vec3f>& mpos, const std::vector<vec3f>& mnorm,
    const std::vector<vec2f>& mtexcoord) {
    auto o = (int)pos.size();
    for (auto t : mtriangles) triangles.push_back({t.x + o, t.y + o, t.z + o});
    for (auto p : mpos) pos.push_back(p);
    for (auto n : mnorm) norm.push_back(n);
    for (auto t : mtexcoord) texcoord.push_back(t);
}

///
/// Merge a quad mesh into another.
///
inline void merge_quads(std::vector<vec4i>& quads, std::vector<vec3f>& pos,
    std::vector<vec3f>& norm, std::vector<vec2f>& texcoord,
    const std::vector<vec4i>& mquads, const std::vector<vec3f>& mpos,
    const std::vector<vec3f>& mnorm, const std::vector<vec2f>& mtexcoord) {
    auto o = (int)pos.size();
    for (auto q : mquads) quads.push_back({q.x + o, q.y + o, q.z + o, q.w + o});
    for (auto p : mpos) pos.push_back(p);
    for (auto n : mnorm) norm.push_back(n);
    for (auto t : mtexcoord) texcoord.push_back(t);
}

///
/// Unshare shape data by duplicating all vertex data for each element,
/// giving a faceted look. Note that faceted tangents are not computed.
///
inline void facet_lines(std::vector<vec2i>& lines, std::vector<vec3f>& pos,
    std::vector<vec3f>& norm, std::vector<vec2f>& texcoord,
    std::vector<vec4f>& color, std::vector<float>& radius) {
    auto npos = std::vector<vec3f>();
    auto nnorm = std::vector<vec3f>();
    auto ntexcoord = std::vector<vec2f>();
    auto ncolor = std::vector<vec4f>();
    auto nradius = std::vector<float>();

    auto nlines = std::vector<vec2i>();
    for (auto l : lines) {
        nlines.push_back({(int)npos.size(), (int)npos.size() + 1});
        for (auto v : l) {
            if (!pos.empty()) npos.push_back(pos[v]);
            if (!norm.empty()) nnorm.push_back(norm[v]);
            if (!texcoord.empty()) ntexcoord.push_back(texcoord[v]);
            if (!color.empty()) ncolor.push_back(color[v]);
            if (!radius.empty()) nradius.push_back(radius[v]);
        }
    }

    std::swap(pos, npos);
    std::swap(norm, nnorm);
    std::swap(texcoord, ntexcoord);
    std::swap(color, ncolor);
    std::swap(radius, nradius);
    std::swap(lines, nlines);
}

///
/// Unshare shape data by duplicating all vertex data for each element,
/// giving a faceted look.
///
inline void facet_triangles(std::vector<vec3i>& triangles,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord, std::vector<vec4f>& color,
    std::vector<float>& radius) {
    auto npos = std::vector<vec3f>();
    auto nnorm = std::vector<vec3f>();
    auto ntexcoord = std::vector<vec2f>();
    auto ncolor = std::vector<vec4f>();
    auto nradius = std::vector<float>();

    auto ntriangles = std::vector<vec3i>();

    for (auto t : triangles) {
        ntriangles.push_back(
            {(int)npos.size(), (int)npos.size() + 1, (int)npos.size() + 2});
        for (auto v : t) {
            if (!pos.empty()) npos.push_back(pos[v]);
            if (!norm.empty()) nnorm.push_back(norm[v]);
            if (!texcoord.empty()) ntexcoord.push_back(texcoord[v]);
            if (!color.empty()) ncolor.push_back(color[v]);
            if (!radius.empty()) nradius.push_back(radius[v]);
        }
    }

    std::swap(pos, npos);
    std::swap(norm, nnorm);
    std::swap(texcoord, ntexcoord);
    std::swap(color, ncolor);
    std::swap(radius, nradius);
    std::swap(triangles, ntriangles);
}

///
/// Unshare shape data by duplicating all vertex data for each element,
/// giving a faceted look.
///
inline void facet_quads(std::vector<vec4i>& quads, std::vector<vec3f>& pos,
    std::vector<vec3f>& norm, std::vector<vec2f>& texcoord,
    std::vector<vec4f>& color, std::vector<float>& radius) {
    auto npos = std::vector<vec3f>();
    auto nnorm = std::vector<vec3f>();
    auto ntexcoord = std::vector<vec2f>();
    auto ncolor = std::vector<vec4f>();
    auto nradius = std::vector<float>();

    auto nquads = std::vector<vec4i>();

    for (auto q : quads) {
        nquads.push_back({(int)npos.size(), (int)npos.size() + 1,
            (int)npos.size() + 2, (int)npos.size() + 3});
        for (auto v : q) {
            if (!pos.empty()) npos.push_back(pos[v]);
            if (!norm.empty()) nnorm.push_back(norm[v]);
            if (!texcoord.empty()) ntexcoord.push_back(texcoord[v]);
            if (!color.empty()) ncolor.push_back(color[v]);
            if (!radius.empty()) nradius.push_back(radius[v]);
        }
    }

    std::swap(pos, npos);
    std::swap(norm, nnorm);
    std::swap(texcoord, ntexcoord);
    std::swap(color, ncolor);
    std::swap(radius, nradius);
    std::swap(quads, nquads);
}

// -----------------------------------------------------------------------------
// SHAPE SAMPLING
// -----------------------------------------------------------------------------

///
/// Compute a distribution for sampling points uniformly
///
inline void sample_points_cdf(int npoints, float* cdf) {
    for (auto i = 0; i < npoints; i++) cdf[i] = i + 1;
}

///
/// Compute a distribution for sampling points uniformly
///
inline std::vector<float> sample_points_cdf(int npoints) {
    auto cdf = std::vector<float>(npoints);
    sample_points_cdf((int)npoints, cdf.data());
    return cdf;
}

///
/// Pick a point
///
inline int sample_points(int npoints, float re) {
    return ym::clamp(0, npoints - 1, (int)(re * npoints));
}

///
/// Pick a point
///
inline int sample_points(int npoints, const float* cdf, float re) {
    re = clamp(re * cdf[npoints - 1], 0.0f, cdf[npoints - 1] - 0.00001f);
    return (int)(std::upper_bound(cdf, cdf + npoints, re) - cdf);
}

///
/// Pick a point
///
inline int sample_points(const std::vector<float>& cdf, float re) {
    return sample_points((int)cdf.size(), cdf.data(), re);
}

///
/// Compute a distribution for sampling lines uniformly
///
inline void sample_lines_cdf(
    int nlines, const vec2i* lines, const vec3f* pos, float* cdf) {
    for (auto i = 0; i < nlines; i++)
        cdf[i] = length(pos[lines[i].x] - pos[lines[i].y]);
    for (auto i = 1; i < nlines; i++) cdf[i] += cdf[i - 1];
}

///
/// Compute a distribution for sampling lines uniformly
///
inline std::vector<float> sample_lines_cdf(
    const std::vector<vec2i>& lines, const std::vector<vec3f>& pos) {
    auto cdf = std::vector<float>(lines.size());
    sample_lines_cdf((int)lines.size(), lines.data(), pos.data(), cdf.data());
    return cdf;
}

///
/// Pick a point on lines
///
inline std::pair<int, vec2f> sample_lines(
    int nlines, const float* cdf, float re, float ruv) {
    re = clamp(re * cdf[nlines - 1], 0.0f, cdf[nlines - 1] - 0.00001f);
    auto eid = (int)(std::upper_bound(cdf, cdf + nlines, re) - cdf);
    return {eid, {1 - ruv, ruv}};
}

///
/// Pick a point on lines
///
inline std::pair<int, vec2f> sample_lines(
    const std::vector<float>& cdf, float re, float ruv) {
    return sample_lines((int)cdf.size(), cdf.data(), re, ruv);
}

///
/// Compute a distribution for sampling triangle meshes uniformly
///
inline void sample_triangles_cdf(
    int ntriangles, const vec3i* triangles, const vec3f* pos, float* cdf) {
    for (auto i = 0; i < ntriangles; i++)
        cdf[i] = triangle_area(
            pos[triangles[i].x], pos[triangles[i].y], pos[triangles[i].z]);
    for (auto i = 1; i < ntriangles; i++) cdf[i] += cdf[i - 1];
}

///
/// Compute a distribution for sampling triangle meshes uniformly
///
inline std::vector<float> sample_triangles_cdf(
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& pos) {
    auto cdf = std::vector<float>(triangles.size());
    sample_triangles_cdf(
        (int)triangles.size(), triangles.data(), pos.data(), cdf.data());
    return cdf;
}

///
/// Pick a point on a triangle mesh
///
inline std::pair<int, vec3f> sample_triangles(
    int ntriangles, const float* cdf, float re, const vec2f& ruv) {
    re = clamp(re * cdf[ntriangles - 1], 0.0f, cdf[ntriangles - 1] - 0.00001f);
    auto eid = (int)(std::upper_bound(cdf, cdf + ntriangles, re) - cdf);
    return {
        eid, {sqrt(ruv.x) * (1 - ruv.y), 1 - sqrt(ruv.x), ruv.y * sqrt(ruv.x)}};
}

///
/// Pick a point on a triangle mesh
///
inline std::pair<int, vec3f> sample_triangles(
    const std::vector<float>& cdf, float re, const vec2f& ruv) {
    return sample_triangles((int)cdf.size(), cdf.data(), re, ruv);
}
///
/// Compute a distribution for sampling quad meshes uniformly
///
inline void sample_quads_cdf(
    int nquads, const vec4i* quads, const vec3f* pos, float* cdf) {
    for (auto i = 0; i < nquads; i++)
        cdf[i] = quad_area(
            pos[quads[i].x], pos[quads[i].y], pos[quads[i].z], pos[quads[i].w]);
    for (auto i = 1; i < nquads; i++) cdf[i] += cdf[i - 1];
}

///
/// Compute a distribution for sampling quad meshes uniformly
///
inline std::vector<float> sample_quads_cdf(
    const std::vector<vec4i>& quads, const std::vector<vec3f>& pos) {
    auto cdf = std::vector<float>(quads.size());
    sample_quads_cdf((int)quads.size(), quads.data(), pos.data(), cdf.data());
    return cdf;
}

///
/// Pick a point on a quad mesh
///
inline std::pair<int, vec4f> sample_quads(
    int nquads, const float* cdf, float re, const vec2f& ruv) {
    if (ruv.x < 0.5f) {
        auto eid = 0;
        auto euv = zero3f;
        std::tie(eid, euv) =
            sample_triangles(nquads, cdf, re, {ruv.x * 2, ruv.y});
        return {eid, {euv.x, euv.y, 0, euv.z}};
    } else {
        auto eid = 0;
        auto euv = zero3f;
        std::tie(eid, euv) =
            sample_triangles(nquads, cdf, re, {(ruv.x - 0.5f) * 2, ruv.y});
        return {eid, {0, euv.z, euv.x, euv.y}};
    }
}

///
/// Pick a point on a quad mesh
///
inline std::pair<int, vec4f> sample_quads(
    const std::vector<float>& cdf, float re, const vec2f& ruv) {
    return sample_quads((int)cdf.size(), cdf.data(), re, ruv);
}
///
/// Samples a set of points over a triangle mesh uniformly. The rng function
/// takes the point index and returns vec3f numbers uniform directibuted in
/// [0,1]^3. ù norm and texcoord are optional.
///
inline void sample_triangles_points(int ntriangles, const vec3i* triangles,
    const vec3f* pos, const vec3f* norm, const vec2f* texcoord, int npoints,
    vec3f* sampled_pos, vec3f* sampled_norm, vec2f* sampled_texcoord,
    uint64_t seed) {
    auto cdf = std::vector<float>(ntriangles);
    sample_triangles_cdf(ntriangles, triangles, pos, cdf.data());
    rng_pcg32 rng;
    init(&rng, seed, 0);
    for (auto i = 0; i < npoints; i++) {
        auto eid = 0;
        auto euv = zero3f;
        std::tie(eid, euv) = sample_triangles(
            ntriangles, cdf.data(), next1f(&rng), {next1f(&rng), next1f(&rng)});
        auto t = triangles[eid];
        if (sampled_pos)
            sampled_pos[i] =
                pos[t.x] * euv.x + pos[t.y] * euv.y + pos[t.z] * euv.z;
        if (sampled_norm)
            sampled_norm[i] = normalize(
                norm[t.x] * euv.x + norm[t.y] * euv.y + norm[t.z] * euv.z);
        if (sampled_texcoord)
            sampled_texcoord[i] = texcoord[t.x] * euv.x +
                                  texcoord[t.y] * euv.y + texcoord[t.z] * euv.z;
    }
}

///
/// Samples a set of points over a triangle mesh uniformly.
/// Wrapper to the above function.
///
inline void sample_triangles_points(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& pos, const std::vector<vec3f>& norm,
    const std::vector<vec2f>& texcoord, int npoints,
    std::vector<vec3f>& sampled_pos, std::vector<vec3f>& sampled_norm,
    std::vector<vec2f>& sampled_texcoord, uint64_t seed) {
    sampled_pos.resize(npoints);
    if (!norm.empty()) sampled_norm.resize(npoints);
    if (!texcoord.empty()) sampled_texcoord.resize(npoints);
    sample_triangles_points((int)triangles.size(), triangles.data(), pos.data(),
        norm.data(), texcoord.data(), npoints, sampled_pos.data(),
        sampled_norm.data(), sampled_texcoord.data(), seed);
}

// -----------------------------------------------------------------------------
// STANDARD SHAPES
// -----------------------------------------------------------------------------

///
/// Make a sphere. This is not watertight.
///
inline void make_uvsphere(int usteps, int vsteps, bool as_triangles,
    std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, as_triangles, triangles, quads, pos, norm,
        texcoord,
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * (1 - uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * (1 - uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) { return uv; });
}

///
/// Make a geodesic sphere.
///
inline void make_geodesicsphere(int level, std::vector<vec3i>& triangles,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm) {
    // https://stackoverflow.com/questions/17705621/algorithm-for-a-geodesic-sphere
    const float X = 0.525731112119133606f;
    const float Z = 0.850650808352039932f;
    pos = {{-X, 0.0, Z}, {X, 0.0, Z}, {-X, 0.0, -Z}, {X, 0.0, -Z}, {0.0, Z, X},
        {0.0, Z, -X}, {0.0, -Z, X}, {0.0, -Z, -X}, {Z, X, 0.0}, {-Z, X, 0.0},
        {Z, -X, 0.0}, {-Z, -X, 0.0}};
    triangles = {{0, 1, 4}, {0, 4, 9}, {9, 4, 5}, {4, 8, 5}, {4, 1, 8},
        {8, 1, 10}, {8, 10, 3}, {5, 8, 3}, {5, 3, 2}, {2, 3, 7}, {7, 3, 10},
        {7, 10, 6}, {7, 6, 11}, {11, 6, 0}, {0, 6, 1}, {6, 10, 1}, {9, 11, 0},
        {9, 2, 11}, {9, 5, 2}, {7, 11, 2}};
    norm = pos;
    std::vector<vec2f> _aux1;
    std::vector<vec4f> _aux2;
    std::vector<float> _aux3;
    for (auto l = 0; l < level - 2; l++) {
        tesselate_triangles(triangles, pos, norm, _aux1, _aux2, _aux3, false);
    }
    for (auto& p : pos) p = normalize(p);
    for (auto& n : norm) n = normalize(n);
}

///
/// Make a sphere. This is not watertight.
///
inline void make_uvhemisphere(int usteps, int vsteps, bool as_triangles,
    std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, as_triangles, triangles, quads, pos, norm,
        texcoord,
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * 0.5f * (1 - uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * 0.5f * (1 - uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) { return uv; });
}

///
/// Make an inside-out sphere. This is not watertight.
///
inline void make_uvflippedsphere(int usteps, int vsteps, bool as_triangles,
    std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, as_triangles, triangles, quads, pos, norm,
        texcoord,
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * uv.y};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * uv.y};
            return vec3f{-cos(a.x) * sin(a.y), -sin(a.x) * sin(a.y), -cos(a.y)};
        },
        [](const vec2f& uv) {
            return vec2f{uv.x, 1 - uv.y};
        });
}

///
/// Make an inside-out hemisphere. This is not watertight.
///
inline void make_uvflippedhemisphere(int usteps, int vsteps, bool as_triangles,
    std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, as_triangles, triangles, quads, pos, norm,
        texcoord,
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * (0.5f + 0.5f * uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * uv.y};
            return vec3f{-cos(a.x) * sin(a.y), -sin(a.x) * sin(a.y), -cos(a.y)};
        },
        [](const vec2f& uv) {
            return vec2f{uv.x, 1 - uv.y};
        });
}

///
/// Make a quad.
///
inline void make_uvquad(int usteps, int vsteps, bool as_triangles,
    std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, as_triangles, triangles, quads, pos, norm,
        texcoord,
        [](const vec2f& uv) {
            return vec3f{(-1 + uv.x * 2), (-1 + uv.y * 2), 0};
        },
        [](const vec2f& uv) {
            return vec3f{0, 0, 1};
        },
        [](const vec2f& uv) { return uv; });
}

///
/// Make a cube with unique vertices. This is watertight but has no
/// texture coordinates or normals.
///
inline void make_cube(std::vector<vec4i>& quads, std::vector<vec3f>& pos) {
    static auto cube_pos = std::vector<vec3f>{{-1, -1, -1}, {-1, +1, -1},
        {+1, +1, -1}, {+1, -1, -1}, {-1, -1, +1}, {-1, +1, +1}, {+1, +1, +1},
        {+1, -1, +1}};
    static auto cube_quads = std::vector<vec4i>{{0, 1, 2, 3}, {7, 6, 5, 4},
        {4, 5, 1, 0}, {6, 7, 3, 2}, {2, 1, 5, 6}, {0, 3, 7, 4}};
    static auto cube_quad_uv =
        std::vector<vec2f>{{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    pos = cube_pos;
    quads = cube_quads;
}

///
/// Make a cube with unique vertices. This is watertight but has no
/// texture coordinates or normals.
///
inline void make_cube(std::vector<vec3i>& triangles, std::vector<vec3f>& pos) {
    auto quads = std::vector<vec4i>();
    make_cube(quads, pos);
    triangles = convert_quads_to_triangles(quads);
}

///
/// Make a suzanne monkey model for testing. Note that some quads are
/// degenerate.
///
inline void make_suzanne(std::vector<vec4i>& quads, std::vector<vec3f>& pos) {
    static auto suzanne_pos = std::vector<vec3f>{{0.4375, 0.1640625, 0.765625},
        {-0.4375, 0.1640625, 0.765625}, {0.5, 0.09375, 0.6875},
        {-0.5, 0.09375, 0.6875}, {0.546875, 0.0546875, 0.578125},
        {-0.546875, 0.0546875, 0.578125}, {0.3515625, -0.0234375, 0.6171875},
        {-0.3515625, -0.0234375, 0.6171875}, {0.3515625, 0.03125, 0.71875},
        {-0.3515625, 0.03125, 0.71875}, {0.3515625, 0.1328125, 0.78125},
        {-0.3515625, 0.1328125, 0.78125}, {0.2734375, 0.1640625, 0.796875},
        {-0.2734375, 0.1640625, 0.796875}, {0.203125, 0.09375, 0.7421875},
        {-0.203125, 0.09375, 0.7421875}, {0.15625, 0.0546875, 0.6484375},
        {-0.15625, 0.0546875, 0.6484375}, {0.078125, 0.2421875, 0.65625},
        {-0.078125, 0.2421875, 0.65625}, {0.140625, 0.2421875, 0.7421875},
        {-0.140625, 0.2421875, 0.7421875}, {0.2421875, 0.2421875, 0.796875},
        {-0.2421875, 0.2421875, 0.796875}, {0.2734375, 0.328125, 0.796875},
        {-0.2734375, 0.328125, 0.796875}, {0.203125, 0.390625, 0.7421875},
        {-0.203125, 0.390625, 0.7421875}, {0.15625, 0.4375, 0.6484375},
        {-0.15625, 0.4375, 0.6484375}, {0.3515625, 0.515625, 0.6171875},
        {-0.3515625, 0.515625, 0.6171875}, {0.3515625, 0.453125, 0.71875},
        {-0.3515625, 0.453125, 0.71875}, {0.3515625, 0.359375, 0.78125},
        {-0.3515625, 0.359375, 0.78125}, {0.4375, 0.328125, 0.765625},
        {-0.4375, 0.328125, 0.765625}, {0.5, 0.390625, 0.6875},
        {-0.5, 0.390625, 0.6875}, {0.546875, 0.4375, 0.578125},
        {-0.546875, 0.4375, 0.578125}, {0.625, 0.2421875, 0.5625},
        {-0.625, 0.2421875, 0.5625}, {0.5625, 0.2421875, 0.671875},
        {-0.5625, 0.2421875, 0.671875}, {0.46875, 0.2421875, 0.7578125},
        {-0.46875, 0.2421875, 0.7578125}, {0.4765625, 0.2421875, 0.7734375},
        {-0.4765625, 0.2421875, 0.7734375}, {0.4453125, 0.3359375, 0.78125},
        {-0.4453125, 0.3359375, 0.78125}, {0.3515625, 0.375, 0.8046875},
        {-0.3515625, 0.375, 0.8046875}, {0.265625, 0.3359375, 0.8203125},
        {-0.265625, 0.3359375, 0.8203125}, {0.2265625, 0.2421875, 0.8203125},
        {-0.2265625, 0.2421875, 0.8203125}, {0.265625, 0.15625, 0.8203125},
        {-0.265625, 0.15625, 0.8203125}, {0.3515625, 0.2421875, 0.828125},
        {-0.3515625, 0.2421875, 0.828125}, {0.3515625, 0.1171875, 0.8046875},
        {-0.3515625, 0.1171875, 0.8046875}, {0.4453125, 0.15625, 0.78125},
        {-0.4453125, 0.15625, 0.78125}, {0.0, 0.4296875, 0.7421875},
        {0.0, 0.3515625, 0.8203125}, {0.0, -0.6796875, 0.734375},
        {0.0, -0.3203125, 0.78125}, {0.0, -0.1875, 0.796875},
        {0.0, -0.7734375, 0.71875}, {0.0, 0.40625, 0.6015625},
        {0.0, 0.5703125, 0.5703125}, {0.0, 0.8984375, -0.546875},
        {0.0, 0.5625, -0.8515625}, {0.0, 0.0703125, -0.828125},
        {0.0, -0.3828125, -0.3515625}, {0.203125, -0.1875, 0.5625},
        {-0.203125, -0.1875, 0.5625}, {0.3125, -0.4375, 0.5703125},
        {-0.3125, -0.4375, 0.5703125}, {0.3515625, -0.6953125, 0.5703125},
        {-0.3515625, -0.6953125, 0.5703125}, {0.3671875, -0.890625, 0.53125},
        {-0.3671875, -0.890625, 0.53125}, {0.328125, -0.9453125, 0.5234375},
        {-0.328125, -0.9453125, 0.5234375}, {0.1796875, -0.96875, 0.5546875},
        {-0.1796875, -0.96875, 0.5546875}, {0.0, -0.984375, 0.578125},
        {0.4375, -0.140625, 0.53125}, {-0.4375, -0.140625, 0.53125},
        {0.6328125, -0.0390625, 0.5390625}, {-0.6328125, -0.0390625, 0.5390625},
        {0.828125, 0.1484375, 0.4453125}, {-0.828125, 0.1484375, 0.4453125},
        {0.859375, 0.4296875, 0.59375}, {-0.859375, 0.4296875, 0.59375},
        {0.7109375, 0.484375, 0.625}, {-0.7109375, 0.484375, 0.625},
        {0.4921875, 0.6015625, 0.6875}, {-0.4921875, 0.6015625, 0.6875},
        {0.3203125, 0.7578125, 0.734375}, {-0.3203125, 0.7578125, 0.734375},
        {0.15625, 0.71875, 0.7578125}, {-0.15625, 0.71875, 0.7578125},
        {0.0625, 0.4921875, 0.75}, {-0.0625, 0.4921875, 0.75},
        {0.1640625, 0.4140625, 0.7734375}, {-0.1640625, 0.4140625, 0.7734375},
        {0.125, 0.3046875, 0.765625}, {-0.125, 0.3046875, 0.765625},
        {0.203125, 0.09375, 0.7421875}, {-0.203125, 0.09375, 0.7421875},
        {0.375, 0.015625, 0.703125}, {-0.375, 0.015625, 0.703125},
        {0.4921875, 0.0625, 0.671875}, {-0.4921875, 0.0625, 0.671875},
        {0.625, 0.1875, 0.6484375}, {-0.625, 0.1875, 0.6484375},
        {0.640625, 0.296875, 0.6484375}, {-0.640625, 0.296875, 0.6484375},
        {0.6015625, 0.375, 0.6640625}, {-0.6015625, 0.375, 0.6640625},
        {0.4296875, 0.4375, 0.71875}, {-0.4296875, 0.4375, 0.71875},
        {0.25, 0.46875, 0.7578125}, {-0.25, 0.46875, 0.7578125},
        {0.0, -0.765625, 0.734375}, {0.109375, -0.71875, 0.734375},
        {-0.109375, -0.71875, 0.734375}, {0.1171875, -0.8359375, 0.7109375},
        {-0.1171875, -0.8359375, 0.7109375}, {0.0625, -0.8828125, 0.6953125},
        {-0.0625, -0.8828125, 0.6953125}, {0.0, -0.890625, 0.6875},
        {0.0, -0.1953125, 0.75}, {0.0, -0.140625, 0.7421875},
        {0.1015625, -0.1484375, 0.7421875}, {-0.1015625, -0.1484375, 0.7421875},
        {0.125, -0.2265625, 0.75}, {-0.125, -0.2265625, 0.75},
        {0.0859375, -0.2890625, 0.7421875}, {-0.0859375, -0.2890625, 0.7421875},
        {0.3984375, -0.046875, 0.671875}, {-0.3984375, -0.046875, 0.671875},
        {0.6171875, 0.0546875, 0.625}, {-0.6171875, 0.0546875, 0.625},
        {0.7265625, 0.203125, 0.6015625}, {-0.7265625, 0.203125, 0.6015625},
        {0.7421875, 0.375, 0.65625}, {-0.7421875, 0.375, 0.65625},
        {0.6875, 0.4140625, 0.7265625}, {-0.6875, 0.4140625, 0.7265625},
        {0.4375, 0.546875, 0.796875}, {-0.4375, 0.546875, 0.796875},
        {0.3125, 0.640625, 0.8359375}, {-0.3125, 0.640625, 0.8359375},
        {0.203125, 0.6171875, 0.8515625}, {-0.203125, 0.6171875, 0.8515625},
        {0.1015625, 0.4296875, 0.84375}, {-0.1015625, 0.4296875, 0.84375},
        {0.125, -0.1015625, 0.8125}, {-0.125, -0.1015625, 0.8125},
        {0.2109375, -0.4453125, 0.7109375}, {-0.2109375, -0.4453125, 0.7109375},
        {0.25, -0.703125, 0.6875}, {-0.25, -0.703125, 0.6875},
        {0.265625, -0.8203125, 0.6640625}, {-0.265625, -0.8203125, 0.6640625},
        {0.234375, -0.9140625, 0.6328125}, {-0.234375, -0.9140625, 0.6328125},
        {0.1640625, -0.9296875, 0.6328125}, {-0.1640625, -0.9296875, 0.6328125},
        {0.0, -0.9453125, 0.640625}, {0.0, 0.046875, 0.7265625},
        {0.0, 0.2109375, 0.765625}, {0.328125, 0.4765625, 0.7421875},
        {-0.328125, 0.4765625, 0.7421875}, {0.1640625, 0.140625, 0.75},
        {-0.1640625, 0.140625, 0.75}, {0.1328125, 0.2109375, 0.7578125},
        {-0.1328125, 0.2109375, 0.7578125}, {0.1171875, -0.6875, 0.734375},
        {-0.1171875, -0.6875, 0.734375}, {0.078125, -0.4453125, 0.75},
        {-0.078125, -0.4453125, 0.75}, {0.0, -0.4453125, 0.75},
        {0.0, -0.328125, 0.7421875}, {0.09375, -0.2734375, 0.78125},
        {-0.09375, -0.2734375, 0.78125}, {0.1328125, -0.2265625, 0.796875},
        {-0.1328125, -0.2265625, 0.796875}, {0.109375, -0.1328125, 0.78125},
        {-0.109375, -0.1328125, 0.78125}, {0.0390625, -0.125, 0.78125},
        {-0.0390625, -0.125, 0.78125}, {0.0, -0.203125, 0.828125},
        {0.046875, -0.1484375, 0.8125}, {-0.046875, -0.1484375, 0.8125},
        {0.09375, -0.15625, 0.8125}, {-0.09375, -0.15625, 0.8125},
        {0.109375, -0.2265625, 0.828125}, {-0.109375, -0.2265625, 0.828125},
        {0.078125, -0.25, 0.8046875}, {-0.078125, -0.25, 0.8046875},
        {0.0, -0.2890625, 0.8046875}, {0.2578125, -0.3125, 0.5546875},
        {-0.2578125, -0.3125, 0.5546875}, {0.1640625, -0.2421875, 0.7109375},
        {-0.1640625, -0.2421875, 0.7109375}, {0.1796875, -0.3125, 0.7109375},
        {-0.1796875, -0.3125, 0.7109375}, {0.234375, -0.25, 0.5546875},
        {-0.234375, -0.25, 0.5546875}, {0.0, -0.875, 0.6875},
        {0.046875, -0.8671875, 0.6875}, {-0.046875, -0.8671875, 0.6875},
        {0.09375, -0.8203125, 0.7109375}, {-0.09375, -0.8203125, 0.7109375},
        {0.09375, -0.7421875, 0.7265625}, {-0.09375, -0.7421875, 0.7265625},
        {0.0, -0.78125, 0.65625}, {0.09375, -0.75, 0.6640625},
        {-0.09375, -0.75, 0.6640625}, {0.09375, -0.8125, 0.640625},
        {-0.09375, -0.8125, 0.640625}, {0.046875, -0.8515625, 0.6328125},
        {-0.046875, -0.8515625, 0.6328125}, {0.0, -0.859375, 0.6328125},
        {0.171875, 0.21875, 0.78125}, {-0.171875, 0.21875, 0.78125},
        {0.1875, 0.15625, 0.7734375}, {-0.1875, 0.15625, 0.7734375},
        {0.3359375, 0.4296875, 0.7578125}, {-0.3359375, 0.4296875, 0.7578125},
        {0.2734375, 0.421875, 0.7734375}, {-0.2734375, 0.421875, 0.7734375},
        {0.421875, 0.3984375, 0.7734375}, {-0.421875, 0.3984375, 0.7734375},
        {0.5625, 0.3515625, 0.6953125}, {-0.5625, 0.3515625, 0.6953125},
        {0.5859375, 0.2890625, 0.6875}, {-0.5859375, 0.2890625, 0.6875},
        {0.578125, 0.1953125, 0.6796875}, {-0.578125, 0.1953125, 0.6796875},
        {0.4765625, 0.1015625, 0.71875}, {-0.4765625, 0.1015625, 0.71875},
        {0.375, 0.0625, 0.7421875}, {-0.375, 0.0625, 0.7421875},
        {0.2265625, 0.109375, 0.78125}, {-0.2265625, 0.109375, 0.78125},
        {0.1796875, 0.296875, 0.78125}, {-0.1796875, 0.296875, 0.78125},
        {0.2109375, 0.375, 0.78125}, {-0.2109375, 0.375, 0.78125},
        {0.234375, 0.359375, 0.7578125}, {-0.234375, 0.359375, 0.7578125},
        {0.1953125, 0.296875, 0.7578125}, {-0.1953125, 0.296875, 0.7578125},
        {0.2421875, 0.125, 0.7578125}, {-0.2421875, 0.125, 0.7578125},
        {0.375, 0.0859375, 0.7265625}, {-0.375, 0.0859375, 0.7265625},
        {0.4609375, 0.1171875, 0.703125}, {-0.4609375, 0.1171875, 0.703125},
        {0.546875, 0.2109375, 0.671875}, {-0.546875, 0.2109375, 0.671875},
        {0.5546875, 0.28125, 0.671875}, {-0.5546875, 0.28125, 0.671875},
        {0.53125, 0.3359375, 0.6796875}, {-0.53125, 0.3359375, 0.6796875},
        {0.4140625, 0.390625, 0.75}, {-0.4140625, 0.390625, 0.75},
        {0.28125, 0.3984375, 0.765625}, {-0.28125, 0.3984375, 0.765625},
        {0.3359375, 0.40625, 0.75}, {-0.3359375, 0.40625, 0.75},
        {0.203125, 0.171875, 0.75}, {-0.203125, 0.171875, 0.75},
        {0.1953125, 0.2265625, 0.75}, {-0.1953125, 0.2265625, 0.75},
        {0.109375, 0.4609375, 0.609375}, {-0.109375, 0.4609375, 0.609375},
        {0.1953125, 0.6640625, 0.6171875}, {-0.1953125, 0.6640625, 0.6171875},
        {0.3359375, 0.6875, 0.59375}, {-0.3359375, 0.6875, 0.59375},
        {0.484375, 0.5546875, 0.5546875}, {-0.484375, 0.5546875, 0.5546875},
        {0.6796875, 0.453125, 0.4921875}, {-0.6796875, 0.453125, 0.4921875},
        {0.796875, 0.40625, 0.4609375}, {-0.796875, 0.40625, 0.4609375},
        {0.7734375, 0.1640625, 0.375}, {-0.7734375, 0.1640625, 0.375},
        {0.6015625, 0.0, 0.4140625}, {-0.6015625, 0.0, 0.4140625},
        {0.4375, -0.09375, 0.46875}, {-0.4375, -0.09375, 0.46875},
        {0.0, 0.8984375, 0.2890625}, {0.0, 0.984375, -0.078125},
        {0.0, -0.1953125, -0.671875}, {0.0, -0.4609375, 0.1875},
        {0.0, -0.9765625, 0.4609375}, {0.0, -0.8046875, 0.34375},
        {0.0, -0.5703125, 0.3203125}, {0.0, -0.484375, 0.28125},
        {0.8515625, 0.234375, 0.0546875}, {-0.8515625, 0.234375, 0.0546875},
        {0.859375, 0.3203125, -0.046875}, {-0.859375, 0.3203125, -0.046875},
        {0.7734375, 0.265625, -0.4375}, {-0.7734375, 0.265625, -0.4375},
        {0.4609375, 0.4375, -0.703125}, {-0.4609375, 0.4375, -0.703125},
        {0.734375, -0.046875, 0.0703125}, {-0.734375, -0.046875, 0.0703125},
        {0.59375, -0.125, -0.1640625}, {-0.59375, -0.125, -0.1640625},
        {0.640625, -0.0078125, -0.4296875}, {-0.640625, -0.0078125, -0.4296875},
        {0.3359375, 0.0546875, -0.6640625}, {-0.3359375, 0.0546875, -0.6640625},
        {0.234375, -0.3515625, 0.40625}, {-0.234375, -0.3515625, 0.40625},
        {0.1796875, -0.4140625, 0.2578125}, {-0.1796875, -0.4140625, 0.2578125},
        {0.2890625, -0.7109375, 0.3828125}, {-0.2890625, -0.7109375, 0.3828125},
        {0.25, -0.5, 0.390625}, {-0.25, -0.5, 0.390625},
        {0.328125, -0.9140625, 0.3984375}, {-0.328125, -0.9140625, 0.3984375},
        {0.140625, -0.7578125, 0.3671875}, {-0.140625, -0.7578125, 0.3671875},
        {0.125, -0.5390625, 0.359375}, {-0.125, -0.5390625, 0.359375},
        {0.1640625, -0.9453125, 0.4375}, {-0.1640625, -0.9453125, 0.4375},
        {0.21875, -0.28125, 0.4296875}, {-0.21875, -0.28125, 0.4296875},
        {0.2109375, -0.2265625, 0.46875}, {-0.2109375, -0.2265625, 0.46875},
        {0.203125, -0.171875, 0.5}, {-0.203125, -0.171875, 0.5},
        {0.2109375, -0.390625, 0.1640625}, {-0.2109375, -0.390625, 0.1640625},
        {0.296875, -0.3125, -0.265625}, {-0.296875, -0.3125, -0.265625},
        {0.34375, -0.1484375, -0.5390625}, {-0.34375, -0.1484375, -0.5390625},
        {0.453125, 0.8671875, -0.3828125}, {-0.453125, 0.8671875, -0.3828125},
        {0.453125, 0.9296875, -0.0703125}, {-0.453125, 0.9296875, -0.0703125},
        {0.453125, 0.8515625, 0.234375}, {-0.453125, 0.8515625, 0.234375},
        {0.4609375, 0.5234375, 0.4296875}, {-0.4609375, 0.5234375, 0.4296875},
        {0.7265625, 0.40625, 0.3359375}, {-0.7265625, 0.40625, 0.3359375},
        {0.6328125, 0.453125, 0.28125}, {-0.6328125, 0.453125, 0.28125},
        {0.640625, 0.703125, 0.0546875}, {-0.640625, 0.703125, 0.0546875},
        {0.796875, 0.5625, 0.125}, {-0.796875, 0.5625, 0.125},
        {0.796875, 0.6171875, -0.1171875}, {-0.796875, 0.6171875, -0.1171875},
        {0.640625, 0.75, -0.1953125}, {-0.640625, 0.75, -0.1953125},
        {0.640625, 0.6796875, -0.4453125}, {-0.640625, 0.6796875, -0.4453125},
        {0.796875, 0.5390625, -0.359375}, {-0.796875, 0.5390625, -0.359375},
        {0.6171875, 0.328125, -0.5859375}, {-0.6171875, 0.328125, -0.5859375},
        {0.484375, 0.0234375, -0.546875}, {-0.484375, 0.0234375, -0.546875},
        {0.8203125, 0.328125, -0.203125}, {-0.8203125, 0.328125, -0.203125},
        {0.40625, -0.171875, 0.1484375}, {-0.40625, -0.171875, 0.1484375},
        {0.4296875, -0.1953125, -0.2109375},
        {-0.4296875, -0.1953125, -0.2109375}, {0.890625, 0.40625, -0.234375},
        {-0.890625, 0.40625, -0.234375}, {0.7734375, -0.140625, -0.125},
        {-0.7734375, -0.140625, -0.125}, {1.0390625, -0.1015625, -0.328125},
        {-1.0390625, -0.1015625, -0.328125}, {1.28125, 0.0546875, -0.4296875},
        {-1.28125, 0.0546875, -0.4296875}, {1.3515625, 0.3203125, -0.421875},
        {-1.3515625, 0.3203125, -0.421875}, {1.234375, 0.5078125, -0.421875},
        {-1.234375, 0.5078125, -0.421875}, {1.0234375, 0.4765625, -0.3125},
        {-1.0234375, 0.4765625, -0.3125}, {1.015625, 0.4140625, -0.2890625},
        {-1.015625, 0.4140625, -0.2890625}, {1.1875, 0.4375, -0.390625},
        {-1.1875, 0.4375, -0.390625}, {1.265625, 0.2890625, -0.40625},
        {-1.265625, 0.2890625, -0.40625}, {1.2109375, 0.078125, -0.40625},
        {-1.2109375, 0.078125, -0.40625}, {1.03125, -0.0390625, -0.3046875},
        {-1.03125, -0.0390625, -0.3046875}, {0.828125, -0.0703125, -0.1328125},
        {-0.828125, -0.0703125, -0.1328125}, {0.921875, 0.359375, -0.21875},
        {-0.921875, 0.359375, -0.21875}, {0.9453125, 0.3046875, -0.2890625},
        {-0.9453125, 0.3046875, -0.2890625},
        {0.8828125, -0.0234375, -0.2109375},
        {-0.8828125, -0.0234375, -0.2109375}, {1.0390625, 0.0, -0.3671875},
        {-1.0390625, 0.0, -0.3671875}, {1.1875, 0.09375, -0.4453125},
        {-1.1875, 0.09375, -0.4453125}, {1.234375, 0.25, -0.4453125},
        {-1.234375, 0.25, -0.4453125}, {1.171875, 0.359375, -0.4375},
        {-1.171875, 0.359375, -0.4375}, {1.0234375, 0.34375, -0.359375},
        {-1.0234375, 0.34375, -0.359375}, {0.84375, 0.2890625, -0.2109375},
        {-0.84375, 0.2890625, -0.2109375}, {0.8359375, 0.171875, -0.2734375},
        {-0.8359375, 0.171875, -0.2734375}, {0.7578125, 0.09375, -0.2734375},
        {-0.7578125, 0.09375, -0.2734375}, {0.8203125, 0.0859375, -0.2734375},
        {-0.8203125, 0.0859375, -0.2734375}, {0.84375, 0.015625, -0.2734375},
        {-0.84375, 0.015625, -0.2734375}, {0.8125, -0.015625, -0.2734375},
        {-0.8125, -0.015625, -0.2734375}, {0.7265625, 0.0, -0.0703125},
        {-0.7265625, 0.0, -0.0703125}, {0.71875, -0.0234375, -0.171875},
        {-0.71875, -0.0234375, -0.171875}, {0.71875, 0.0390625, -0.1875},
        {-0.71875, 0.0390625, -0.1875}, {0.796875, 0.203125, -0.2109375},
        {-0.796875, 0.203125, -0.2109375}, {0.890625, 0.2421875, -0.265625},
        {-0.890625, 0.2421875, -0.265625}, {0.890625, 0.234375, -0.3203125},
        {-0.890625, 0.234375, -0.3203125}, {0.8125, -0.015625, -0.3203125},
        {-0.8125, -0.015625, -0.3203125}, {0.8515625, 0.015625, -0.3203125},
        {-0.8515625, 0.015625, -0.3203125}, {0.828125, 0.078125, -0.3203125},
        {-0.828125, 0.078125, -0.3203125}, {0.765625, 0.09375, -0.3203125},
        {-0.765625, 0.09375, -0.3203125}, {0.84375, 0.171875, -0.3203125},
        {-0.84375, 0.171875, -0.3203125}, {1.0390625, 0.328125, -0.4140625},
        {-1.0390625, 0.328125, -0.4140625}, {1.1875, 0.34375, -0.484375},
        {-1.1875, 0.34375, -0.484375}, {1.2578125, 0.2421875, -0.4921875},
        {-1.2578125, 0.2421875, -0.4921875}, {1.2109375, 0.0859375, -0.484375},
        {-1.2109375, 0.0859375, -0.484375}, {1.046875, 0.0, -0.421875},
        {-1.046875, 0.0, -0.421875}, {0.8828125, -0.015625, -0.265625},
        {-0.8828125, -0.015625, -0.265625}, {0.953125, 0.2890625, -0.34375},
        {-0.953125, 0.2890625, -0.34375}, {0.890625, 0.109375, -0.328125},
        {-0.890625, 0.109375, -0.328125}, {0.9375, 0.0625, -0.3359375},
        {-0.9375, 0.0625, -0.3359375}, {1.0, 0.125, -0.3671875},
        {-1.0, 0.125, -0.3671875}, {0.9609375, 0.171875, -0.3515625},
        {-0.9609375, 0.171875, -0.3515625}, {1.015625, 0.234375, -0.375},
        {-1.015625, 0.234375, -0.375}, {1.0546875, 0.1875, -0.3828125},
        {-1.0546875, 0.1875, -0.3828125}, {1.109375, 0.2109375, -0.390625},
        {-1.109375, 0.2109375, -0.390625}, {1.0859375, 0.2734375, -0.390625},
        {-1.0859375, 0.2734375, -0.390625}, {1.0234375, 0.4375, -0.484375},
        {-1.0234375, 0.4375, -0.484375}, {1.25, 0.46875, -0.546875},
        {-1.25, 0.46875, -0.546875}, {1.3671875, 0.296875, -0.5},
        {-1.3671875, 0.296875, -0.5}, {1.3125, 0.0546875, -0.53125},
        {-1.3125, 0.0546875, -0.53125}, {1.0390625, -0.0859375, -0.4921875},
        {-1.0390625, -0.0859375, -0.4921875}, {0.7890625, -0.125, -0.328125},
        {-0.7890625, -0.125, -0.328125}, {0.859375, 0.3828125, -0.3828125},
        {-0.859375, 0.3828125, -0.3828125}};
    static auto suzanne_triangles = std::vector<vec3i>{{60, 64, 48},
        {49, 65, 61}, {62, 64, 60}, {61, 65, 63}, {60, 58, 62}, {63, 59, 61},
        {60, 56, 58}, {59, 57, 61}, {60, 54, 56}, {57, 55, 61}, {60, 52, 54},
        {55, 53, 61}, {60, 50, 52}, {53, 51, 61}, {60, 48, 50}, {51, 49, 61},
        {224, 228, 226}, {227, 229, 225}, {72, 283, 73}, {73, 284, 72},
        {341, 347, 383}, {384, 348, 342}, {299, 345, 343}, {344, 346, 300},
        {323, 379, 351}, {352, 380, 324}, {441, 443, 445}, {446, 444, 442},
        {463, 491, 465}, {466, 492, 464}, {495, 497, 499}, {500, 498, 496}};
    static auto suzanne_quads = std::vector<vec4i>{{46, 0, 2, 44},
        {3, 1, 47, 45}, {44, 2, 4, 42}, {5, 3, 45, 43}, {2, 8, 6, 4},
        {7, 9, 3, 5}, {0, 10, 8, 2}, {9, 11, 1, 3}, {10, 12, 14, 8},
        {15, 13, 11, 9}, {8, 14, 16, 6}, {17, 15, 9, 7}, {14, 20, 18, 16},
        {19, 21, 15, 17}, {12, 22, 20, 14}, {21, 23, 13, 15}, {22, 24, 26, 20},
        {27, 25, 23, 21}, {20, 26, 28, 18}, {29, 27, 21, 19}, {26, 32, 30, 28},
        {31, 33, 27, 29}, {24, 34, 32, 26}, {33, 35, 25, 27}, {34, 36, 38, 32},
        {39, 37, 35, 33}, {32, 38, 40, 30}, {41, 39, 33, 31}, {38, 44, 42, 40},
        {43, 45, 39, 41}, {36, 46, 44, 38}, {45, 47, 37, 39}, {46, 36, 50, 48},
        {51, 37, 47, 49}, {36, 34, 52, 50}, {53, 35, 37, 51}, {34, 24, 54, 52},
        {55, 25, 35, 53}, {24, 22, 56, 54}, {57, 23, 25, 55}, {22, 12, 58, 56},
        {59, 13, 23, 57}, {12, 10, 62, 58}, {63, 11, 13, 59}, {10, 0, 64, 62},
        {65, 1, 11, 63}, {0, 46, 48, 64}, {49, 47, 1, 65}, {88, 173, 175, 90},
        {175, 174, 89, 90}, {86, 171, 173, 88}, {174, 172, 87, 89},
        {84, 169, 171, 86}, {172, 170, 85, 87}, {82, 167, 169, 84},
        {170, 168, 83, 85}, {80, 165, 167, 82}, {168, 166, 81, 83},
        {78, 91, 145, 163}, {146, 92, 79, 164}, {91, 93, 147, 145},
        {148, 94, 92, 146}, {93, 95, 149, 147}, {150, 96, 94, 148},
        {95, 97, 151, 149}, {152, 98, 96, 150}, {97, 99, 153, 151},
        {154, 100, 98, 152}, {99, 101, 155, 153}, {156, 102, 100, 154},
        {101, 103, 157, 155}, {158, 104, 102, 156}, {103, 105, 159, 157},
        {160, 106, 104, 158}, {105, 107, 161, 159}, {162, 108, 106, 160},
        {107, 66, 67, 161}, {67, 66, 108, 162}, {109, 127, 159, 161},
        {160, 128, 110, 162}, {127, 178, 157, 159}, {158, 179, 128, 160},
        {125, 155, 157, 178}, {158, 156, 126, 179}, {123, 153, 155, 125},
        {156, 154, 124, 126}, {121, 151, 153, 123}, {154, 152, 122, 124},
        {119, 149, 151, 121}, {152, 150, 120, 122}, {117, 147, 149, 119},
        {150, 148, 118, 120}, {115, 145, 147, 117}, {148, 146, 116, 118},
        {113, 163, 145, 115}, {146, 164, 114, 116}, {113, 180, 176, 163},
        {176, 181, 114, 164}, {109, 161, 67, 111}, {67, 162, 110, 112},
        {111, 67, 177, 182}, {177, 67, 112, 183}, {176, 180, 182, 177},
        {183, 181, 176, 177}, {134, 136, 175, 173}, {175, 136, 135, 174},
        {132, 134, 173, 171}, {174, 135, 133, 172}, {130, 132, 171, 169},
        {172, 133, 131, 170}, {165, 186, 184, 167}, {185, 187, 166, 168},
        {130, 169, 167, 184}, {168, 170, 131, 185}, {143, 189, 188, 186},
        {188, 189, 144, 187}, {184, 186, 188, 68}, {188, 187, 185, 68},
        {129, 130, 184, 68}, {185, 131, 129, 68}, {141, 192, 190, 143},
        {191, 193, 142, 144}, {139, 194, 192, 141}, {193, 195, 140, 142},
        {138, 196, 194, 139}, {195, 197, 138, 140}, {137, 70, 196, 138},
        {197, 70, 137, 138}, {189, 143, 190, 69}, {191, 144, 189, 69},
        {69, 190, 205, 207}, {206, 191, 69, 207}, {70, 198, 199, 196},
        {200, 198, 70, 197}, {196, 199, 201, 194}, {202, 200, 197, 195},
        {194, 201, 203, 192}, {204, 202, 195, 193}, {192, 203, 205, 190},
        {206, 204, 193, 191}, {198, 203, 201, 199}, {202, 204, 198, 200},
        {198, 207, 205, 203}, {206, 207, 198, 204}, {138, 139, 163, 176},
        {164, 140, 138, 176}, {139, 141, 210, 163}, {211, 142, 140, 164},
        {141, 143, 212, 210}, {213, 144, 142, 211}, {143, 186, 165, 212},
        {166, 187, 144, 213}, {80, 208, 212, 165}, {213, 209, 81, 166},
        {208, 214, 210, 212}, {211, 215, 209, 213}, {78, 163, 210, 214},
        {211, 164, 79, 215}, {130, 129, 71, 221}, {71, 129, 131, 222},
        {132, 130, 221, 219}, {222, 131, 133, 220}, {134, 132, 219, 217},
        {220, 133, 135, 218}, {136, 134, 217, 216}, {218, 135, 136, 216},
        {216, 217, 228, 230}, {229, 218, 216, 230}, {217, 219, 226, 228},
        {227, 220, 218, 229}, {219, 221, 224, 226}, {225, 222, 220, 227},
        {221, 71, 223, 224}, {223, 71, 222, 225}, {223, 230, 228, 224},
        {229, 230, 223, 225}, {182, 180, 233, 231}, {234, 181, 183, 232},
        {111, 182, 231, 253}, {232, 183, 112, 254}, {109, 111, 253, 255},
        {254, 112, 110, 256}, {180, 113, 251, 233}, {252, 114, 181, 234},
        {113, 115, 249, 251}, {250, 116, 114, 252}, {115, 117, 247, 249},
        {248, 118, 116, 250}, {117, 119, 245, 247}, {246, 120, 118, 248},
        {119, 121, 243, 245}, {244, 122, 120, 246}, {121, 123, 241, 243},
        {242, 124, 122, 244}, {123, 125, 239, 241}, {240, 126, 124, 242},
        {125, 178, 235, 239}, {236, 179, 126, 240}, {178, 127, 237, 235},
        {238, 128, 179, 236}, {127, 109, 255, 237}, {256, 110, 128, 238},
        {237, 255, 257, 275}, {258, 256, 238, 276}, {235, 237, 275, 277},
        {276, 238, 236, 278}, {239, 235, 277, 273}, {278, 236, 240, 274},
        {241, 239, 273, 271}, {274, 240, 242, 272}, {243, 241, 271, 269},
        {272, 242, 244, 270}, {245, 243, 269, 267}, {270, 244, 246, 268},
        {247, 245, 267, 265}, {268, 246, 248, 266}, {249, 247, 265, 263},
        {266, 248, 250, 264}, {251, 249, 263, 261}, {264, 250, 252, 262},
        {233, 251, 261, 279}, {262, 252, 234, 280}, {255, 253, 259, 257},
        {260, 254, 256, 258}, {253, 231, 281, 259}, {282, 232, 254, 260},
        {231, 233, 279, 281}, {280, 234, 232, 282}, {66, 107, 283, 72},
        {284, 108, 66, 72}, {107, 105, 285, 283}, {286, 106, 108, 284},
        {105, 103, 287, 285}, {288, 104, 106, 286}, {103, 101, 289, 287},
        {290, 102, 104, 288}, {101, 99, 291, 289}, {292, 100, 102, 290},
        {99, 97, 293, 291}, {294, 98, 100, 292}, {97, 95, 295, 293},
        {296, 96, 98, 294}, {95, 93, 297, 295}, {298, 94, 96, 296},
        {93, 91, 299, 297}, {300, 92, 94, 298}, {307, 308, 327, 337},
        {328, 308, 307, 338}, {306, 307, 337, 335}, {338, 307, 306, 336},
        {305, 306, 335, 339}, {336, 306, 305, 340}, {88, 90, 305, 339},
        {305, 90, 89, 340}, {86, 88, 339, 333}, {340, 89, 87, 334},
        {84, 86, 333, 329}, {334, 87, 85, 330}, {82, 84, 329, 331},
        {330, 85, 83, 332}, {329, 335, 337, 331}, {338, 336, 330, 332},
        {329, 333, 339, 335}, {340, 334, 330, 336}, {325, 331, 337, 327},
        {338, 332, 326, 328}, {80, 82, 331, 325}, {332, 83, 81, 326},
        {208, 341, 343, 214}, {344, 342, 209, 215}, {80, 325, 341, 208},
        {342, 326, 81, 209}, {78, 214, 343, 345}, {344, 215, 79, 346},
        {78, 345, 299, 91}, {300, 346, 79, 92}, {76, 323, 351, 303},
        {352, 324, 76, 303}, {303, 351, 349, 77}, {350, 352, 303, 77},
        {77, 349, 347, 304}, {348, 350, 77, 304}, {304, 347, 327, 308},
        {328, 348, 304, 308}, {325, 327, 347, 341}, {348, 328, 326, 342},
        {295, 297, 317, 309}, {318, 298, 296, 310}, {75, 315, 323, 76},
        {324, 316, 75, 76}, {301, 357, 355, 302}, {356, 358, 301, 302},
        {302, 355, 353, 74}, {354, 356, 302, 74}, {74, 353, 315, 75},
        {316, 354, 74, 75}, {291, 293, 361, 363}, {362, 294, 292, 364},
        {363, 361, 367, 365}, {368, 362, 364, 366}, {365, 367, 369, 371},
        {370, 368, 366, 372}, {371, 369, 375, 373}, {376, 370, 372, 374},
        {313, 377, 373, 375}, {374, 378, 314, 376}, {315, 353, 373, 377},
        {374, 354, 316, 378}, {353, 355, 371, 373}, {372, 356, 354, 374},
        {355, 357, 365, 371}, {366, 358, 356, 372}, {357, 359, 363, 365},
        {364, 360, 358, 366}, {289, 291, 363, 359}, {364, 292, 290, 360},
        {73, 359, 357, 301}, {358, 360, 73, 301}, {283, 285, 287, 289},
        {288, 286, 284, 290}, {283, 289, 359, 73}, {360, 290, 284, 73},
        {293, 295, 309, 361}, {310, 296, 294, 362}, {309, 311, 367, 361},
        {368, 312, 310, 362}, {311, 381, 369, 367}, {370, 382, 312, 368},
        {313, 375, 369, 381}, {370, 376, 314, 382}, {347, 349, 385, 383},
        {386, 350, 348, 384}, {317, 383, 385, 319}, {386, 384, 318, 320},
        {297, 299, 383, 317}, {384, 300, 298, 318}, {299, 343, 341, 383},
        {342, 344, 300, 384}, {313, 321, 379, 377}, {380, 322, 314, 378},
        {315, 377, 379, 323}, {380, 378, 316, 324}, {319, 385, 379, 321},
        {380, 386, 320, 322}, {349, 351, 379, 385}, {380, 352, 350, 386},
        {399, 387, 413, 401}, {414, 388, 400, 402}, {399, 401, 403, 397},
        {404, 402, 400, 398}, {397, 403, 405, 395}, {406, 404, 398, 396},
        {395, 405, 407, 393}, {408, 406, 396, 394}, {393, 407, 409, 391},
        {410, 408, 394, 392}, {391, 409, 411, 389}, {412, 410, 392, 390},
        {409, 419, 417, 411}, {418, 420, 410, 412}, {407, 421, 419, 409},
        {420, 422, 408, 410}, {405, 423, 421, 407}, {422, 424, 406, 408},
        {403, 425, 423, 405}, {424, 426, 404, 406}, {401, 427, 425, 403},
        {426, 428, 402, 404}, {401, 413, 415, 427}, {416, 414, 402, 428},
        {317, 319, 443, 441}, {444, 320, 318, 442}, {319, 389, 411, 443},
        {412, 390, 320, 444}, {309, 317, 441, 311}, {442, 318, 310, 312},
        {381, 429, 413, 387}, {414, 430, 382, 388}, {411, 417, 439, 443},
        {440, 418, 412, 444}, {437, 445, 443, 439}, {444, 446, 438, 440},
        {433, 445, 437, 435}, {438, 446, 434, 436}, {431, 447, 445, 433},
        {446, 448, 432, 434}, {429, 447, 431, 449}, {432, 448, 430, 450},
        {413, 429, 449, 415}, {450, 430, 414, 416}, {311, 447, 429, 381},
        {430, 448, 312, 382}, {311, 441, 445, 447}, {446, 442, 312, 448},
        {415, 449, 451, 475}, {452, 450, 416, 476}, {449, 431, 461, 451},
        {462, 432, 450, 452}, {431, 433, 459, 461}, {460, 434, 432, 462},
        {433, 435, 457, 459}, {458, 436, 434, 460}, {435, 437, 455, 457},
        {456, 438, 436, 458}, {437, 439, 453, 455}, {454, 440, 438, 456},
        {439, 417, 473, 453}, {474, 418, 440, 454}, {427, 415, 475, 463},
        {476, 416, 428, 464}, {425, 427, 463, 465}, {464, 428, 426, 466},
        {423, 425, 465, 467}, {466, 426, 424, 468}, {421, 423, 467, 469},
        {468, 424, 422, 470}, {419, 421, 469, 471}, {470, 422, 420, 472},
        {417, 419, 471, 473}, {472, 420, 418, 474}, {457, 455, 479, 477},
        {480, 456, 458, 478}, {477, 479, 481, 483}, {482, 480, 478, 484},
        {483, 481, 487, 485}, {488, 482, 484, 486}, {485, 487, 489, 491},
        {490, 488, 486, 492}, {463, 475, 485, 491}, {486, 476, 464, 492},
        {451, 483, 485, 475}, {486, 484, 452, 476}, {451, 461, 477, 483},
        {478, 462, 452, 484}, {457, 477, 461, 459}, {462, 478, 458, 460},
        {453, 473, 479, 455}, {480, 474, 454, 456}, {471, 481, 479, 473},
        {480, 482, 472, 474}, {469, 487, 481, 471}, {482, 488, 470, 472},
        {467, 489, 487, 469}, {488, 490, 468, 470}, {465, 491, 489, 467},
        {490, 492, 466, 468}, {391, 389, 503, 501}, {504, 390, 392, 502},
        {393, 391, 501, 499}, {502, 392, 394, 500}, {395, 393, 499, 497},
        {500, 394, 396, 498}, {397, 395, 497, 495}, {498, 396, 398, 496},
        {399, 397, 495, 493}, {496, 398, 400, 494}, {387, 399, 493, 505},
        {494, 400, 388, 506}, {493, 501, 503, 505}, {504, 502, 494, 506},
        {493, 495, 499, 501}, {500, 496, 494, 502}, {313, 381, 387, 505},
        {388, 382, 314, 506}, {313, 505, 503, 321}, {504, 506, 314, 322},
        {319, 321, 503, 389}, {504, 322, 320, 390}};
    pos = suzanne_pos;
    quads = suzanne_quads;
    quads.reserve(quads.size() + suzanne_triangles.size());
    for (auto& t : suzanne_triangles) { quads.push_back({t.x, t.y, t.z, t.z}); }
}

///
/// Make a cube with uv. This is not watertight.
///
inline void make_uvcube(int usteps, int vsteps, bool as_triangles,
    std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    frame3f frames[6] = {frame3f{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 1}},
        frame3f{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, -1}},
        frame3f{{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0, 1, 0}},
        frame3f{{1, 0, 0}, {0, 0, 1}, {0, -1, 0}, {0, -1, 0}},
        frame3f{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}, {1, 0, 0}},
        frame3f{{0, -1, 0}, {0, 0, 1}, {-1, 0, 0}, {-1, 0, 0}}};
    std::vector<vec3f> quad_pos, quad_norm;
    std::vector<vec2f> quad_texcoord;
    std::vector<vec3i> quad_triangles;
    std::vector<vec4i> quad_quads;
    make_uvquad(usteps, vsteps, as_triangles, quad_triangles, quad_quads,
        quad_pos, quad_norm, quad_texcoord);
    for (auto i = 0; i < 6; i++) {
        pos.insert(pos.end(), quad_pos.begin(), quad_pos.end());
        norm.insert(norm.end(), quad_norm.begin(), quad_norm.end());
        texcoord.insert(
            texcoord.end(), quad_texcoord.begin(), quad_texcoord.end());
        if (as_triangles) {
            triangles.insert(
                triangles.end(), quad_triangles.begin(), quad_triangles.end());
        } else {
            quads.insert(quads.end(), quad_quads.begin(), quad_quads.end());
        }
    }
    auto quad_verts = quad_pos.size();
    for (auto i = 0; i < 6; i++) {
        for (auto j = quad_verts * i; j < quad_verts * (i + 1); j++)
            pos[j] = transform_point(frames[i], pos[j]);
        for (auto j = quad_verts * i; j < quad_verts * (i + 1); j++)
            norm[j] = transform_direction(frames[i], norm[j]);
    }
    if (as_triangles) {
        auto quad_faces = quad_triangles.size(), quad_verts = quad_pos.size();
        for (auto i = 0; i < 6; i++) {
            for (auto j = quad_faces * i; j < quad_faces * (i + 1); j++) {
                triangles[j].x += quad_verts * i;
                triangles[j].y += quad_verts * i;
                triangles[j].z += quad_verts * i;
            }
        }
    } else {
        auto quad_faces = quad_quads.size(), quad_verts = quad_pos.size();
        for (auto i = 0; i < 6; i++) {
            for (auto j = quad_faces * i; j < quad_faces * (i + 1); j++) {
                quads[j].x += quad_verts * i;
                quads[j].y += quad_verts * i;
                quads[j].z += quad_verts * i;
                quads[j].w += quad_verts * i;
            }
        }
    }
}

///
/// Make a sphere from a cube. This is not watertight.
///
inline void make_uvspherecube(int usteps, int vsteps, bool as_triangles,
    std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    make_uvcube(
        usteps, vsteps, as_triangles, triangles, quads, pos, norm, texcoord);
    for (auto i = 0; i < pos.size(); i++) {
        pos[i] = normalize(pos[i]);
        norm[i] = normalize(pos[i]);
    }
}

///
/// Make a cube than stretch it towards a sphere. This is not watertight.
///
inline void make_uvspherizedcube(int usteps, int vsteps, float radius,
    bool as_triangles, std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    make_uvcube(
        usteps, vsteps, as_triangles, triangles, quads, pos, norm, texcoord);
    for (auto i = 0; i < pos.size(); i++) {
        norm[i] = normalize(pos[i]);
        pos[i] *= 1 - radius;
        pos[i] += norm[i] * radius;
    }
    compute_normals(triangles, pos, norm, true);
}

///
/// Make a flipped sphere. This is not watertight.
///
inline void make_uvflipcapsphere(int usteps, int vsteps, float radius,
    bool as_triangles, std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    make_uvsphere(
        usteps, vsteps, as_triangles, triangles, quads, pos, norm, texcoord);
    for (auto i = 0; i < pos.size(); i++) {
        if (pos[i].z > radius) {
            pos[i].z = 2 * radius - pos[i].z;
            norm[i].x = -norm[i].x;
            norm[i].y = -norm[i].y;
        } else if (pos[i].z < -radius) {
            pos[i].z = -2 * radius - pos[i].z;
            norm[i].x = -norm[i].x;
            norm[i].y = -norm[i].y;
        }
    }
}

///
/// Make a butout sphere. This is not watertight.
///
inline void make_uvcutsphere(int usteps, int vsteps, float radius,
    bool as_triangles, std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, as_triangles, triangles, quads, pos, norm,
        texcoord,
        [radius](const vec2f& uv) {
            auto p = 1 - std::acos(radius) / pif;
            auto a = vec2f{2 * pif * uv.x, pif * (1 - p * uv.y)};
            return vec3f{std::cos(a.x) * std::sin(a.y),
                std::sin(a.x) * std::sin(a.y), std::cos(a.y)};
        },
        [radius](const vec2f& uv) {
            auto p = 1 - std::acos(radius) / pif;
            auto a = vec2f{2 * pif * uv.x, pif * (1 - p * uv.y)};
            return vec3f{std::cos(a.x) * std::sin(a.y),
                std::sin(a.x) * std::sin(a.y), std::cos(a.y)};
        },
        [](const vec2f& uv) { return uv; });
}

///
/// Make a quad. This is not watertight.
///
inline void make_uvflippedcutsphere(int usteps, int vsteps, float radius,
    bool as_triangles, std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& pos, std::vector<vec3f>& norm,
    std::vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, as_triangles, triangles, quads, pos, norm,
        texcoord,
        [radius](const vec2f& uv) {
            auto p = 1 - acos(radius) / pif;
            auto a = vec2f{2 * pif * uv.x, pif * ((1 - p) + p * uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [radius](const vec2f& uv) {
            auto p = 1 - acos(radius) / pif;
            auto a = vec2f{2 * pif * uv.x, pif * ((1 - p) + p * uv.y)};
            return vec3f{-cos(a.x) * sin(a.y), -sin(a.x) * sin(a.y), -cos(a.y)};
        },
        [](const vec2f& uv) {
            return vec2f{uv.x, (1 - uv.y)};
        });
}

// -----------------------------------------------------------------------------
// RAY-PRIMITIVE INTERSECTION FUNCTIONS
// -----------------------------------------------------------------------------

///
/// Intersect a ray with a point (approximate)
///
/// Parameters:
/// - ray: ray origin and direction, parameter min, max range
/// - p: point position
/// - r: point radius
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: primitive uv ( {0,0} for points )
///
/// Returns:
/// - whether the intersection occurred
///
/// Iplementation Notes:
/// - out Parameters and only writtent o if an intersection occurs
/// - algorithm finds the closest point on the ray segment to the point and
///    test their distance with the point radius
/// - based on http://geomalgorithms.com/a02-lines.html.
///
inline bool intersect_point(
    const ray3f& ray, const vec3f& p, float r, float& ray_t) {
    // find parameter for line-point minimum distance
    auto w = p - ray.o;
    auto t = dot(w, ray.d) / dot(ray.d, ray.d);

    // exit if not within bounds
    if (t < ray.tmin || t > ray.tmax) return false;

    // test for line-point distance vs point radius
    auto rp = eval(ray, t);
    auto prp = p - rp;
    if (dot(prp, prp) > r * r) return false;

    // intersection occurred: set params and exit
    ray_t = t;

    return true;
}

///
/// Intersect a ray with a line
///
/// Parameters:
/// - ray: ray origin and direction, parameter min, max range
/// - v0, v1: line segment points
/// - r0, r1: line segment radia
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: euv.x is the line parameter at the intersection ( euv.y is zero )
///
/// Returns:
/// - whether the intersection occurred
///
/// Notes:
/// - out Parameters and only writtent o if an intersection occurs
/// - algorithm find the closest points on line and ray segment and test
///   their distance with the line radius at that location
/// - based on http://geomalgorithms.com/a05-intersect-1.html
/// - based on http://geomalgorithms.com/a07-distance.html#
///     dist3D_Segment_to_Segment
///
inline bool intersect_line(const ray3f& ray, const vec3f& v0, const vec3f& v1,
    float r0, float r1, float& ray_t, vec2f& euv) {
    // setup intersection params
    auto u = ray.d;
    auto v = v1 - v0;
    auto w = ray.o - v0;

    // compute values to solve a linear system
    auto a = dot(u, u);
    auto b = dot(u, v);
    auto c = dot(v, v);
    auto d = dot(u, w);
    auto e = dot(v, w);
    auto det = a * c - b * b;

    // check determinant and exit if lines are parallel
    // (could use EPSILONS if desired)
    if (det == 0) return false;

    // compute Parameters on both ray and segment
    auto t = (b * e - c * d) / det;
    auto s = (a * e - b * d) / det;

    // exit if not within bounds
    if (t < ray.tmin || t > ray.tmax) return false;

    // clamp segment param to segment corners
    s = clamp(s, (float)0, (float)1);

    // compute segment-segment distance on the closest points
    auto p0 = eval(ray, t);
    auto p1 = eval(ray3f{v0, v1 - v0}, s);
    auto p01 = p0 - p1;

    // check with the line radius at the same point
    auto r = r0 * (1 - s) + r1 * s;
    if (dot(p01, p01) > r * r) return false;

    // intersection occurred: set params and exit
    ray_t = t;
    euv = {1 - s, s};

    return true;
}

///
/// Intersect a ray with a triangle
///
/// Parameters:
/// - ray: ray origin and direction, parameter min, max range
/// - v0, v1, v2: triangle vertices
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: baricentric coordinates of the intersection
///
/// Returns:
/// - whether the intersection occurred
///
/// Notes:
/// - out Parameters and only writtent o if an intersection occurs
/// - algorithm based on Muller-Trombone intersection test
///
inline bool intersect_triangle(const ray3f& ray, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, float& ray_t, vec3f& euv) {
    // compute triangle edges
    auto edge1 = v1 - v0;
    auto edge2 = v2 - v0;

    // compute determinant to solve a linear system
    auto pvec = cross(ray.d, edge2);
    auto det = dot(edge1, pvec);

    // check determinant and exit if triangle and ray are parallel
    // (could use EPSILONS if desired)
    if (det == 0) return false;
    auto inv_det = 1.0f / det;

    // compute and check first bricentric coordinated
    auto tvec = ray.o - v0;
    auto u = dot(tvec, pvec) * inv_det;
    if (u < 0 || u > 1) return false;

    // compute and check second bricentric coordinated
    auto qvec = cross(tvec, edge1);
    auto v = dot(ray.d, qvec) * inv_det;
    if (v < 0 || u + v > 1) return false;

    // compute and check ray parameter
    auto t = dot(edge2, qvec) * inv_det;
    if (t < ray.tmin || t > ray.tmax) return false;

    // intersection occurred: set params and exit
    ray_t = t;
    euv = {1 - u - v, u, v};

    return true;
}

///
/// Intersect a ray with a quad represented as two triangles (0,1,3) and
/// (2,3,1), with the uv coordinates of the second triangle corrected by u =
/// 1-u' and v = 1-v' to produce a quad parametrization where u and v go from 0
/// to 1. This is equivalent to Intel's Embree. The external user does not have
/// to be concerned about the parametrization and can just use the euv as
/// specified.
///
/// Parameters:
/// - ray: ray origin and direction, parameter min, max range
/// - v0, v1, v2, v3: quad vertices
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: baricentric coordinates of the intersection
///
/// Returns:
/// - whether the intersection occurred
///
inline bool intersect_quad(const ray3f& ray, const vec3f& v0, const vec3f& v1,
    const vec3f& v2, const vec3f& v3, float& ray_t, vec4f& euv) {
    auto hit = false;
    auto tray = ray;
    if (intersect_triangle(tray, v0, v1, v3, ray_t, (vec3f&)euv)) {
        euv = {euv.x, euv.y, 0, euv.z};
        tray.tmax = ray_t;
        hit = true;
    }
    if (intersect_triangle(tray, v2, v3, v1, ray_t, (vec3f&)euv)) {
        euv = {0, 1 - euv.y, euv.y + euv.z - 1, 1 - euv.z};
        tray.tmax = ray_t;
        hit = true;
    }
    return hit;
}

///
/// Intersect a ray with a tetrahedron. Note that we consider only
/// intersection wiht the tetrahedra surface and discount intersction with
/// the interior.
///
/// Parameters:
/// - ray: ray to intersect with
/// - v0, v1, v2: triangle vertices
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: baricentric coordinates of the intersection
///
/// Returns:
/// - whether the intersection occurred
///
/// TODO: check order
/// TODO: uv
///
inline bool intersect_tetrahedron(const ray3f& ray_, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, const vec3f& v3, float& ray_t,
    vec4f& euv) {
    // check intersction for each face
    auto hit = false;
    auto ray = ray_;
    auto tuv = zero3f;
    if (intersect_triangle(ray, v0, v1, v2, ray_t, tuv)) {
        hit = true;
        ray.tmax = ray_t;
    }
    if (intersect_triangle(ray, v0, v1, v3, ray_t, tuv)) {
        hit = true;
        ray.tmax = ray_t;
    }
    if (intersect_triangle(ray, v0, v2, v3, ray_t, tuv)) {
        hit = true;
        ray.tmax = ray_t;
    }
    if (intersect_triangle(ray, v1, v2, v3, ray_t, tuv)) {
        hit = true;
        ray.tmax = ray_t;
    }

    return hit;
}

///
/// Intersect a ray with a axis-aligned bounding box
///
/// Parameters:
/// - ray: ray to intersect with
/// - bbox: bounding box min/max bounds
///
/// Returns:
/// - whether the intersection occurred
///
inline bool intersect_check_bbox(const ray3f& ray, const bbox3f& bbox) {
    // set up convenient pointers for looping over axes
    auto tmin = ray.tmin, tmax = ray.tmax;

    // for each axis, clip intersection against the bounding planes
    for (int i = 0; i < 3; i++) {
        // determine intersection ranges
        auto invd = 1.0f / ray.d[i];
        auto t0 = (bbox.min[i] - ray.o[i]) * invd;
        auto t1 = (bbox.max[i] - ray.o[i]) * invd;
        // flip based on range directions
        if (invd < 0.0f) {
            float a = t0;
            t0 = t1;
            t1 = a;
        }
        // clip intersection
        tmin = t0 > tmin ? t0 : tmin;
        tmax = t1 < tmax ? t1 : tmax;
        // if intersection is empty, exit
        if (tmin > tmax) return false;
    }

    // passed all planes, then intersection occurred
    return true;
}

///
/// Min/max used in BVH traversal. Copied here since the traversal code
/// relies on the specific behaviour wrt NaNs.
///
template <typename T>
static inline const T& _safemin(const T& a, const T& b) {
    return (a < b) ? a : b;
}
///
/// Min/max used in BVH traversal. Copied here since the traversal code
/// relies on the specific behaviour wrt NaNs.
///
template <typename T>
static inline const T& _safemax(const T& a, const T& b) {
    return (a > b) ? a : b;
}

///
/// Intersect a ray with a axis-aligned bounding box
///
/// Parameters:
/// - ray_o, ray_d: ray origin and direction
/// - ray_tmin, ray_tmax: ray parameter min, max range
/// - ray_dinv: ray inverse direction
/// - ray_dsign: ray direction sign
/// - bbox_min, bbox_max: bounding box min/max bounds
///
/// Returns:
/// - whether the intersection occurred
///
/// Implementation Notes:
/// - based on "Robust BVH Ray Traversal" by T. Ize published at
/// http://jcgt.org/published/0002/02/02/paper.pdf
///
inline bool intersect_check_bbox(const ray3f& ray, const vec3f& ray_dinv,
    const vec3i& ray_dsign, const bbox3f& bbox_) {
    auto bbox = &bbox_.min;
    auto txmin = (bbox[ray_dsign.x].x - ray.o.x) * ray_dinv.x;
    auto txmax = (bbox[1 - ray_dsign.x].x - ray.o.x) * ray_dinv.x;
    auto tymin = (bbox[ray_dsign.y].y - ray.o.y) * ray_dinv.y;
    auto tymax = (bbox[1 - ray_dsign.y].y - ray.o.y) * ray_dinv.y;
    auto tzmin = (bbox[ray_dsign.z].z - ray.o.z) * ray_dinv.z;
    auto tzmax = (bbox[1 - ray_dsign.z].z - ray.o.z) * ray_dinv.z;
    auto tmin = _safemax(tzmin, _safemax(tymin, _safemax(txmin, ray.tmin)));
    auto tmax = _safemin(tzmax, _safemin(tymax, _safemin(txmax, ray.tmax)));
    tmax *= 1.00000024f;  // for double: 1.0000000000000004
    return tmin <= tmax;
}

// -----------------------------------------------------------------------------
// POINT-PRIMITIVE DISTANCE FUNCTIONS
// -----------------------------------------------------------------------------

// TODO: documentation
inline bool overlap_point(
    const vec3f& pos, float dist_max, const vec3f& p, float r, float& dist) {
    auto d2 = distsqr(pos, p);
    if (d2 > (dist_max + r) * (dist_max + r)) return false;
    dist = sqrt(d2);
    return true;
}

// TODO: documentation
inline vec2f closestuv_line(
    const vec3f& pos, const vec3f& v0, const vec3f& v1) {
    auto ab = v1 - v0;
    auto d = dot(ab, ab);
    // Project c onto ab, computing parameterized position d(t) = a + t*(b –
    // a)
    auto u = dot(pos - v0, ab) / d;
    u = clamp(u, (float)0, (float)1);
    return {1 - u, u};
}

// TODO: documentation
inline bool overlap_line(const vec3f& pos, float dist_max, const vec3f& v0,
    const vec3f& v1, float r0, float r1, float& dist, vec2f& euv) {
    auto uv = closestuv_line(pos, v0, v1);
    // Compute projected position from the clamped t d = a + t * ab;
    auto p = lerp(v0, v1, uv.y);
    auto r = lerp(r0, r1, uv.y);
    auto d2 = distsqr(pos, p);
    // check distance
    if (d2 > (dist_max + r) * (dist_max + r)) return false;
    // done
    dist = sqrt(d2);
    euv = uv;
    return true;
}

// TODO: documentation
// this is a complicated test -> I probably prefer to use a sequence of test
// (triangle body, and 3 edges)
inline vec3f closestuv_triangle(
    const vec3f& pos, const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    auto ab = v1 - v0;
    auto ac = v2 - v0;
    auto ap = pos - v0;

    auto d1 = dot(ab, ap);
    auto d2 = dot(ac, ap);

    // corner and edge cases
    if (d1 <= 0 && d2 <= 0) return vec3f{1, 0, 0};

    auto bp = pos - v1;
    auto d3 = dot(ab, bp);
    auto d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return vec3f{0, 1, 0};

    auto vc = d1 * d4 - d3 * d2;
    if ((vc <= 0) && (d1 >= 0) && (d3 <= 0))
        return vec3f{1 - d1 / (d1 - d3), d1 / (d1 - d3), 0};

    auto cp = pos - v2;
    auto d5 = dot(ab, cp);
    auto d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return vec3f{0, 0, 1};

    auto vb = d5 * d2 - d1 * d6;
    if ((vb <= 0) && (d2 >= 0) && (d6 <= 0))
        return vec3f{1 - d2 / (d2 - d6), 0, d2 / (d2 - d6)};

    auto va = d3 * d6 - d5 * d4;
    if ((va <= 0) && (d4 - d3 >= 0) && (d5 - d6 >= 0)) {
        auto w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return vec3f{0, 1 - w, w};
    }

    // face case
    auto denom = 1 / (va + vb + vc);
    auto v = vb * denom;
    auto w = vc * denom;
    return vec3f{1 - v - w, v, w};
}

// TODO: documentation
inline bool overlap_triangle(const vec3f& pos, float dist_max, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, float r0, float r1, float r2, float& dist,
    vec3f& euv) {
    auto uv = closestuv_triangle(pos, v0, v1, v2);
    auto p = blerp(v0, v1, v2, uv);
    auto r = blerp(r0, r1, r2, uv);
    auto dd = distsqr(p, pos);
    if (dd > (dist_max + r) * (dist_max + r)) return false;
    dist = sqrt(dd);
    euv = uv;
    return true;
}

// TODO: documentation
inline bool overlap_quad(const vec3f& pos, float dist_max, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, const vec3f& v3, float r0, float r1,
    float r2, float r3, float& dist, vec4f& euv) {
    auto hit = false;
    if (overlap_triangle(
            pos, dist_max, v0, v1, v3, r0, r1, r3, dist, (vec3f&)euv)) {
        euv = {euv.x, euv.y, 0, euv.z};
        dist_max = dist;
        hit = true;
    }
    if (overlap_triangle(
            pos, dist_max, v2, v3, v1, r2, r3, r1, dist, (vec3f&)euv)) {
        dist_max = dist;
        euv = {0, 1 - euv.y, euv.y + euv.z - 1, 1 - euv.z};
        hit = true;
    }
    return hit;
}

// TODO: documentation
inline bool overlap_tetrahedron(const vec3f& pos, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, const vec3f& v3, vec4f& euv) {
    auto vol = dot(v3 - v0, cross(v3 - v1, v3 - v0));
    if (vol == 0) return false;
    auto u = dot(v3 - v0, cross(v3 - v1, v3 - v0)) / vol;
    if (u < 0 || u > 1) return false;
    auto v = dot(v3 - v0, cross(v3 - v1, v3 - v0)) / vol;
    if (v < 0 || v > 1 || u + v > 1) return false;
    auto w = dot(v3 - v0, cross(v3 - v1, v3 - v0)) / vol;
    if (w < 0 || w > 1 || u + v + w > 1) return false;
    euv = {u, v, w, 1 - u - v - w};
    return true;
}

// TODO: documentation
inline bool overlap_tetrahedron(const vec3f& pos, float dist_max,
    const vec3f& v0, const vec3f& v1, const vec3f& v2, const vec3f& v3,
    float r0, float r1, float r2, float r3, float& dist, vec4f& euv) {
    // check interior
    if (overlap_tetrahedron(pos, v0, v1, v2, v3, euv)) {
        dist = 0;
        return true;
    }

    // check faces
    auto hit = false;
    auto tuv = zero3f;
    if (overlap_triangle(pos, dist_max, v0, v1, v2, r0, r1, r2, dist, tuv)) {
        hit = true;
        dist_max = dist;
    }
    if (overlap_triangle(pos, dist_max, v0, v1, v3, r0, r1, r3, dist, tuv)) {
        hit = true;
        dist_max = dist;
    }
    if (overlap_triangle(pos, dist_max, v0, v2, v3, r0, r2, r3, dist, tuv)) {
        hit = true;
        dist_max = dist;
    }
    if (overlap_triangle(pos, dist_max, v1, v2, v3, r1, r2, r3, dist, tuv)) {
        hit = true;
        // dist_max = dist;
    }

    return hit;
}

// TODO: documentation
inline bool distance_check_bbox(
    const vec3f& pos, float dist_max, const bbox3f& bbox) {
    // computing distance
    auto dd = 0.0f;

    // For each axis count any excess distance outside box extents
    for (int i = 0; i < 3; i++) {
        auto v = pos[i];
        if (v < bbox.min[i]) dd += (bbox.min[i] - v) * (bbox.min[i] - v);
        if (v > bbox.max[i]) dd += (v - bbox.max[i]) * (v - bbox.max[i]);
    }

    // check distance
    return dd < dist_max * dist_max;
}

// TODO: doc
inline bool overlap_bbox(const bbox3f& bbox1, const bbox3f& bbox2) {
    if (bbox1.max.x < bbox2.min.x || bbox1.min.x > bbox2.max.x) return false;
    if (bbox1.max.y < bbox2.min.y || bbox1.min.y > bbox2.max.y) return false;
    if (bbox1.max.z < bbox2.min.z || bbox1.min.z > bbox2.max.z) return false;
    return true;
}

// -----------------------------------------------------------------------------
// PRIMITIVE BBOX FUNCTIONS
// -----------------------------------------------------------------------------

///
/// Point bounds
///
inline bbox3f point_bbox(const vec3f& p, float r = 0) {
    return bbox3f{p - vec3f{r, r, r}, p + vec3f{r, r, r}};
}

///
/// Line bounds
///
inline bbox3f line_bbox(
    const vec3f& v0, const vec3f& v1, float r0 = 0, float r1 = 0) {
    return make_bbox({v0 - vec3f{r0, r0, r0}, v0 + vec3f{r0, r0, r0},
        v1 - vec3f{r1, r1, r1}, v1 + vec3f{r1, r1, r1}});
}

///
/// Triangle bounds
///
inline bbox3f triangle_bbox(const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    return make_bbox({v0, v1, v2});
}

///
/// Quad bounds
///
inline bbox3f quad_bbox(
    const vec3f& v0, const vec3f& v1, const vec3f& v2, const vec3f& v3) {
    return make_bbox({v0, v1, v2, v3});
}

///
/// Tetrahedron bounds
///
inline bbox3f tetrahedron_bbox(
    const vec3f& v0, const vec3f& v1, const vec3f& v2, const vec3f& v3) {
    return make_bbox({v0, v1, v2, v3});
}

// -----------------------------------------------------------------------------
// UI UTILITIES
// -----------------------------------------------------------------------------

/// Turntable for UI navigation from a from/to/up parametrization of the
/// camera.
constexpr inline void camera_turntable(vec3f& from, vec3f& to, vec3f& up,
    const vec3f& rotate, float dolly, const vec3f& pan) {
    // rotate if necessary
    if (rotate.x || rotate.y) {
        auto z = normalize(to - from);
        auto lz = dist(to, from);
        auto phi = atan2(z.z, z.x) + rotate.x;
        auto theta = acos(z.y) + rotate.y;
        theta = clamp(theta, 0.001f, pif - 0.001f);
        auto nz = vec3f{sin(theta) * cos(phi) * lz, cos(theta) * lz,
            sin(theta) * sin(phi) * lz};
        from = to - nz;
    }

    // dolly if necessary
    if (dolly) {
        auto z = normalize(to - from);
        auto lz = max(0.001f, dist(to, from) * (1 + dolly));
        z *= lz;
        from = to - z;
    }

    // pan if necessary
    if (pan.x || pan.y) {
        auto z = normalize(to - from);
        auto x = normalize(cross(up, z));
        auto y = normalize(cross(z, x));
        auto t = vec3f{pan.x * x.x + pan.y * y.x, pan.x * x.y + pan.y * y.y,
            pan.x * x.z + pan.y * y.z};
        from += t;
        to += t;
    }
}

/// Turntable for UI navigation for a frame/distance parametrization of the
/// camera.
constexpr inline void camera_turntable(frame3f& frame, float& focus,
    const vec2f& rotate, float dolly, const vec2f& pan) {
    // rotate if necessary
    if (rotate.x || rotate.y) {
        auto phi = atan2(frame.z.z, frame.z.x) + rotate.x;
        auto theta = acos(frame.z.y) + rotate.y;
        theta = clamp(theta, 0.001f, pif - 0.001f);
        auto new_z =
            vec3f{sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi)};
        auto new_center = pos(frame) - frame.z * focus;
        auto new_o = new_center + new_z * focus;
        frame = lookat_frame3(new_o, new_center, {0, 1, 0});
        focus = dist(new_o, new_center);
    }

    // pan if necessary
    if (dolly) {
        auto c = pos(frame) - frame.z * focus;
        focus = max(focus + dolly, 0.001f);
        pos(frame) = c + frame.z * focus;
    }

    // pan if necessary
    if (pan.x || pan.y) { pos(frame) += frame.x * pan.x + frame.y * pan.y; }
}

/// FPS camera for UI navigation for a frame parametrization.
/// https://gamedev.stackexchange.com/questions/30644/how-to-keep-my-quaternion-using-fps-camera-from-tilting-and-messing-up
constexpr inline void camera_fps(
    frame3f& frame, const vec3f& transl, const vec2f& rotate) {
    auto y = vec3f{0, 1, 0};
    auto z = orthonormalize(frame.z, y);
    auto x = cross(y, z);

    frame.rot() = ym::rotation_mat3({1, 0, 0}, rotate.y) * frame.rot() *
                  ym::rotation_mat3({0, 1, 0}, rotate.x);
    frame.pos() += transl.x * x + transl.y * y + transl.z * z;
}

// -----------------------------------------------------------------------------
// IMAGE CONTAINERS
// -----------------------------------------------------------------------------

///
/// Image of a specified type
///
template <typename T>
struct image {
    /// empty image constructor
    constexpr image() : _w{0}, _h{0}, _d{} {}
    /// image constructor
    constexpr image(int w, int h, const T& v = {})
        : _w{w}, _h{h}, _d(size_t(w * h), v) {}
    /// image constructor
    constexpr image(int w, int h, const T* v)
        : _w{w}, _h{h}, _d(v, v + w * h) {}

    /// width
    constexpr int width() const { return _w; }
    /// height
    constexpr int height() const { return _h; }
    /// size
    constexpr vec2i size() const { return {_w, _h}; }
    /// check for empty
    constexpr bool empty() const { return _w == 0 || _h == 0; }
    /// check for empty
    constexpr explicit operator bool() const { return _w != 0 && _h != 0; }

    /// reallocate memory
    void resize(int w, int h, const T& v = {}) {
        _w = w;
        _h = h;
        _d.resize(_w * _h);
    }
    /// reallocate memory
    void assign(int w, int h, const T& v) {
        _w = w;
        _h = h;
        _d.assign(_w * _h, v);
    }

    /// set values
    void set(const T& v) { _d.assign(_w * _h, v); }

    /// element access
    constexpr T& operator[](const vec2i& ij) { return _d[ij.y * _w + ij.x]; }
    /// element access
    constexpr const T& operator[](const vec2i& ij) const {
        return _d[ij.y * _w + ij.x];
    }
    /// element access
    constexpr T& at(const vec2i& ij) { return _d.at(ij.y * _w + ij.x); }
    /// element access
    constexpr const T& at(const vec2i& ij) const {
        return _d.at(ij.y * _w + ij.x);
    }
    /// element access
    constexpr T& at(int i, int j) { return _d.at(j * _w + i); }
    /// element access
    constexpr const T& at(int i, int j) const { return _d.at(j * _w + i); }

    /// data access
    constexpr T* data() { return _d.data(); }
    /// data access
    constexpr const T* data() const { return _d.data(); }

   private:
    int _w, _h;
    std::vector<T> _d;
};

/// 1-dimensional float image
using image1f = image<vec<float, 1>>;
/// 2-dimensional float image
using image2f = image<vec<float, 2>>;
/// 3-dimensional float image
using image3f = image<vec<float, 3>>;
/// 4-dimensional float image
using image4f = image<vec<float, 4>>;

/// 4-dimensional byte image
using image4b = image<vec<byte, 4>>;

/// float image
using imagef = image<float>;

// -----------------------------------------------------------------------------
// IMAGE OPERATIONS
// -----------------------------------------------------------------------------

/// Approximate conversion from srgb.
inline vec3f srgb_to_linear(const vec3b& srgb) {
    return pow(byte_to_float(srgb), 2.2f);
}

/// Approximate conversion from srgb.
inline vec4f srgb_to_linear(const vec4b& srgb) {
    return {pow(byte_to_float(srgb.x), 2.2f), pow(byte_to_float(srgb.y), 2.2f),
        pow(byte_to_float(srgb.z), 2.2f), byte_to_float(srgb.w)};
}

/// Approximate conversion to srgb.
inline vec3b linear_to_srgb(const vec3f& lin) {
    return float_to_byte(pow(lin, 1 / 2.2f));
}

/// Approximate conversion to srgb.
inline vec4b linear_to_srgb(const vec4f& lin) {
    return {float_to_byte(pow(lin.x, 1 / 2.2f)),
        float_to_byte(pow(lin.y, 1 / 2.2f)),
        float_to_byte(pow(lin.z, 1 / 2.2f)), float_to_byte(lin.w)};
}

//
// Tone mapping configurations
//
enum struct tonemap_type { none = 0, srgb, gamma, filmic };

#if 1
///
/// Tone map with a fitted filmic curve.
///
/// Implementation from
/// https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
///
inline vec3f tonemap_filmic(const vec3f& hdr) {
    // rescale
    auto x = hdr * 2.05f;
    // fitted values
    float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    auto y = ((x * (a * x + b)) / (x * (c * x + d) + e));
    return pow(clamp(y, 0.0f, 1.0f), 1 / 2.2f);
}
#else
inline float tonemap_filmic(float x) {
    auto y =
        (x * (x * (x * (x * 2708.7142 + 6801.1525) + 1079.5474) + 1.1614649) -
            0.00004139375) /
        (x * (x * (x * (x * 983.38937 + 4132.0662) + 2881.6522) + 128.35911) +
            1.0);
    return (float)std::max(y, 0.0);
}
#endif

///
/// Tone mapping HDR to LDR images.
///
inline void tonemap_image(int width, int height, const vec4f* hdr, vec4b* ldr,
    tonemap_type tm, float exposure, float gamma) {
    auto scale = pow(2.0f, exposure);
    for (auto j = 0; j < height; j++) {
        for (auto i = 0; i < width; i++) {
            auto h = hdr[j * width + i];
            h.xyz() *= scale;
            switch (tm) {
                case tonemap_type::none: break;
                case tonemap_type::srgb:
                    h.xyz() = pow(h.xyz(), 1 / 2.2f);
                    break;
                case tonemap_type::gamma:
                    h.xyz() = pow(h.xyz(), 1 / gamma);
                    break;
                case tonemap_type::filmic:
                    h.xyz() = tonemap_filmic(h.xyz());
                    break;
            }
            ldr[j * width + i] = float_to_byte(h);
        }
    }
}

///
/// Tone mapping HDR to LDR images.
///
inline void tonemap_image(int width, int height, const vec3f* hdr, vec3b* ldr,
    tonemap_type tm, float exposure, float gamma) {
    auto scale = pow(2.0f, exposure);
    for (auto j = 0; j < height; j++) {
        for (auto i = 0; i < width; i++) {
            auto h = hdr[j * width + i];
            h *= scale;
            switch (tm) {
                case tonemap_type::none: break;
                case tonemap_type::srgb: h = pow(h, 1 / 2.2f); break;
                case tonemap_type::gamma: h = pow(h, 1 / gamma); break;
                case tonemap_type::filmic: h = tonemap_filmic(h); break;
            }
            ldr[j * width + i] = float_to_byte(h);
        }
    }
}

///
/// Tone mapping HDR to LDR images.
///
inline void tonemap_image(const image<vec4f>& hdr, image<vec4b>& ldr,
    tonemap_type tm, float exposure, float gamma) {
    ldr.resize(hdr.width(), hdr.height());
    return tonemap_image(
        hdr.width(), hdr.height(), hdr.data(), ldr.data(), tm, exposure, gamma);
}

///
/// Tone mapping HDR to LDR images.
///
inline image<vec4b> tonemap_image(
    const image<vec4f>& hdr, tonemap_type tm, float exposure, float gamma) {
    auto ldr = image<vec4b>(hdr.width(), hdr.height());
    tonemap_image(
        hdr.width(), hdr.height(), hdr.data(), ldr.data(), tm, exposure, gamma);
    return ldr;
}

///
/// Image over operator
///
inline void image_over(
    vec4f* img, int width, int height, int nlayers, vec4f** layers) {
    for (auto i = 0; i < width * height; i++) {
        img[i] = {0, 0, 0, 0};
        auto weight = 1.0f;
        for (auto l = 0; l < nlayers; l++) {
            img[i].x += layers[l][i].x * layers[l][i].w * weight;
            img[i].y += layers[l][i].y * layers[l][i].w * weight;
            img[i].z += layers[l][i].z * layers[l][i].w * weight;
            img[i].w += layers[l][i].w * weight;
            weight *= (1 - layers[l][i].w);
        }
        if (img[i].w) {
            img[i].x /= img[i].w;
            img[i].y /= img[i].w;
            img[i].z /= img[i].w;
        }
    }
}

///
/// Image over operator
///
inline void image_over(
    vec4b* img, int width, int height, int nlayers, vec4b** layers) {
    for (auto i = 0; i < width * height; i++) {
        auto comp = zero4f;
        auto weight = 1.0f;
        for (auto l = 0; l < nlayers && weight > 0; l++) {
            auto w = byte_to_float(layers[l][i].w);
            comp.x += byte_to_float(layers[l][i].x) * w * weight;
            comp.y += byte_to_float(layers[l][i].y) * w * weight;
            comp.z += byte_to_float(layers[l][i].z) * w * weight;
            comp.w += w * weight;
            weight *= (1 - w);
        }
        if (comp.w) {
            img[i].x = float_to_byte(comp.x / comp.w);
            img[i].y = float_to_byte(comp.y / comp.w);
            img[i].z = float_to_byte(comp.z / comp.w);
            img[i].w = float_to_byte(comp.w);
        } else {
            img[i] = {0, 0, 0, 0};
        }
    }
}

///
/// Convert HSV to RGB
///
/// Implementatkion from
/// http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
///
inline vec4b hsv_to_rgb(const vec4b& hsv) {
    vec4b rgb = {0, 0, 0, hsv.w};
    byte region, remainder, p, q, t;

    byte h = hsv.x, s = hsv.y, v = hsv.z;

    if (s == 0) {
        rgb.x = v;
        rgb.y = v;
        rgb.z = v;
        return rgb;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0:
            rgb.x = v;
            rgb.y = t;
            rgb.z = p;
            break;
        case 1:
            rgb.x = q;
            rgb.y = v;
            rgb.z = p;
            break;
        case 2:
            rgb.x = p;
            rgb.y = v;
            rgb.z = t;
            break;
        case 3:
            rgb.x = p;
            rgb.y = q;
            rgb.z = v;
            break;
        case 4:
            rgb.x = t;
            rgb.y = p;
            rgb.z = v;
            break;
        default:
            rgb.x = v;
            rgb.y = p;
            rgb.z = q;
            break;
    }

    return rgb;
}

// -----------------------------------------------------------------------------
// EXAMPLE IMAGES
// -----------------------------------------------------------------------------

///
/// Make a grid image
///
inline image<vec4b> make_grid_image(int width, int height, int tile = 64,
    const vec4b& c0 = {90, 90, 90, 255},
    const vec4b& c1 = {128, 128, 128, 255}) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < width; j++) {
        for (int i = 0; i < height; i++) {
            auto c = i % tile == 0 || i % tile == tile - 1 || j % tile == 0 ||
                     j % tile == tile - 1;
            pixels.at(i, j) = (c) ? c0 : c1;
        }
    }
    return pixels;
}

///
/// Make a checkerboard image
///
inline image<vec4b> make_checker_image(int width, int height, int tile = 64,
    const vec4b& c0 = {90, 90, 90, 255},
    const vec4b& c1 = {128, 128, 128, 255}) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto c = (i / tile + j / tile) % 2 == 0;
            pixels.at(i, j) = (c) ? c0 : c1;
        }
    }
    return pixels;
}

///
/// Make an image with bumps and dimples.
///
inline image<vec4b> make_bumpdimple_image(
    int width, int height, int tile = 64) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto c = (i / tile + j / tile) % 2 == 0;
            auto ii = i % tile - tile / 2, jj = j % tile - tile / 2;
            auto r =
                sqrt(float(ii * ii + jj * jj)) / sqrt(float(tile * tile) / 4);
            auto h = 0.5f;
            if (r < 0.5f) { h += (c) ? (0.5f - r) : -(0.5f - r); }
            auto g = float_to_byte(h);
            pixels.at(i, j) = vec4b{g, g, g, 255};
        }
    }
    return pixels;
}

///
/// Make a uv colored grid
///
inline image<vec4b> make_ramp_image(int width, int height, const vec4b& c0,
    const vec4b& c1, bool srgb = false) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto u = (float)i / (float)width;
            if (srgb) {
                pixels.at(i, j) = linear_to_srgb(
                    srgb_to_linear(c0) * (1 - u) + srgb_to_linear(c1) * u);
            } else {
                pixels.at(i, j) = float_to_byte(
                    byte_to_float(c0) * (1 - u) + byte_to_float(c1) * u);
            }
        }
    }
    return pixels;
}

///
/// Make a gamma ramp image
///
inline image<vec4b> make_gammaramp_image(int width, int height) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto u = j / float(height - 1);
            if (i < width / 3) u = pow(u, 2.2f);
            if (i > (width * 2) / 3) u = pow(u, 1 / 2.2f);
            auto c = (unsigned char)(u * 255);
            pixels.at(i, j) = {c, c, c, 255};
        }
    }
    return pixels;
}

///
/// Make a gamma ramp image
///
inline image<vec4f> make_gammaramp_imagef(int width, int height) {
    image<vec4f> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto u = j / float(height - 1);
            if (i < width / 3) u = pow(u, 2.2f);
            if (i > (width * 2) / 3) u = pow(u, 1 / 2.2f);
            pixels.at(i, j) = {u, u, u, 1};
        }
    }
    return pixels;
}

///
/// Make an image color with red/green in the [0,1] range. Helpful to visualize
/// uv texture coordinate application.
///
inline image<vec4b> make_uv_image(int width, int height) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto r = float_to_byte(i / (float)(width - 1));
            auto g = float_to_byte(j / (float)(height - 1));
            pixels.at(i, j) = vec4b{r, g, 0, 255};
        }
    }
    return pixels;
}

///
/// Make a uv colored grid
///
inline image<vec4b> make_uvgrid_image(
    int width, int height, int tile = 64, bool colored = true) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            byte ph = 32 * (i / (height / 8));
            byte pv = 128;
            byte ps = 64 + 16 * (7 - j / (height / 8));
            if (i % (tile / 2) && j % (tile / 2)) {
                if ((i / tile + j / tile) % 2)
                    pv += 16;
                else
                    pv -= 16;
            } else {
                pv = 196;
                ps = 32;
            }
            pixels.at(i, j) = (colored) ? hsv_to_rgb({ph, ps, pv, 255}) :
                                          vec4b{pv, pv, pv, 255};
        }
    }
    return pixels;
}

///
/// Make a uv recusive colored grid
///
inline image<vec4b> make_recuvgrid_image(
    int width, int height, int tile = 64, bool colored = true) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            byte ph = 32 * (i / (height / 8));
            byte pv = 128;
            byte ps = 64 + 16 * (7 - j / (height / 8));
            if (i % (tile / 2) && j % (tile / 2)) {
                if ((i / tile + j / tile) % 2)
                    pv += 16;
                else
                    pv -= 16;
                if ((i / (tile / 4) + j / (tile / 4)) % 2)
                    pv += 4;
                else
                    pv -= 4;
                if ((i / (tile / 8) + j / (tile / 8)) % 2)
                    pv += 1;
                else
                    pv -= 1;
            } else {
                pv = 196;
                ps = 32;
            }
            pixels.at(i, j) = (colored) ? hsv_to_rgb({ph, ps, pv, 255}) :
                                          vec4b{pv, pv, pv, 255};
        }
    }
    return pixels;
}

///
/// Comvert a bump map to a normal map.
///
inline image<vec4b> bump_to_normal_map(
    const image<vec4b>& img, float scale = 1) {
    image<vec4b> norm(img.width(), img.height());
    for (int j = 0; j < img.height(); j++) {
        for (int i = 0; i < img.width(); i++) {
            auto i1 = (i + 1) % img.width(), j1 = (j + 1) % img.height();
            auto p00 = img.at(i, j), p10 = img.at(i1, j), p01 = img.at(i, j1);
            auto g00 = (float(p00.x) + float(p00.y) + float(p00.z)) / (3 * 255);
            auto g01 = (float(p01.x) + float(p01.y) + float(p01.z)) / (3 * 255);
            auto g10 = (float(p10.x) + float(p10.y) + float(p10.z)) / (3 * 255);
            auto n = vec3f{scale * (g00 - g10), scale * (g00 - g01), 1.0f};
            n = normalize(n) * 0.5f + vec3f{0.5f, 0.5f, 0.5f};
            auto c =
                vec4b{byte(n.x * 255), byte(n.y * 255), byte(n.z * 255), 255};
            norm.at(i, j) = c;
        }
    }
    return norm;
}

// number of primitives to avoid splitting on
constexpr const int bvh_minprims = 4;

///
/// BVH tree node containing its bounds, indices to the BVH arrays of either
/// sorted primitives or internal nodes, whether its a leaf or an internal node,
/// and the split axis. Leaf and internal nodes are identical, except that
/// indices refer to primitives for leaf nodes or other nodes for internal
/// nodes. See bvh_tree for more details.
///
/// This is an internal data structure.
///
struct bvh_node {
    /// bounding box
    bbox3f bbox;
    /// index to the first sorted primitive/node
    uint32_t start;
    /// number of primitives/nodes
    uint16_t count;
    /// whether it is a leaf
    uint8_t isleaf;
    /// slit axis
    uint8_t axis;
};

///
/// BVH tree, stored as a node array. The tree structure is encoded using array
/// indices instead of pointers, both for speed but also to simplify code.
/// BVH nodes indices refer to either the node array, for internal nodes,
/// or a primitive array, for leaf nodes. BVH trees may contain only one type
/// of geometric primitive, like points, lines, triangle or shape other BVHs.
/// To handle multiple primitive types and transformed primitices, build
/// a two-level hierarchy with the outer BVH, the scene BVH, containing inner
/// BVHs, shape BVHs, each of which of a uniform primitive type.
///
/// This is an internal data structure.
///
struct bvh_tree {
    /// sorted array of internal nodes
    std::vector<bvh_node> nodes;
    /// sorted elements
    std::vector<int> sorted_prim;
};

//
// Struct that pack a bounding box, its associate primitive index, and other
// data for faster hierarchy build.
// This is internal only and should not be used externally.
//
struct bvh_bound_prim {
    ym::bbox3f bbox;   // bounding box
    ym::vec3f center;  // bounding box center (for faster sort)
    int pid;           // primitive id
};

//
// Comparison function for each axis
//
struct bvh_bound_prim_comp {
    int axis;
    float middle;

    bvh_bound_prim_comp(int a, float m = 0) : axis(a), middle(m) {}

    bool operator()(const bvh_bound_prim& a, const bvh_bound_prim& b) const {
        return a.center[axis] < b.center[axis];
    }

    bool operator()(const bvh_bound_prim& a) const {
        return a.center[axis] < middle;
    }
};

//
// Initializes the BVH node node that contains the primitives sorted_prims
// from start to end, by either splitting it into two other nodes,
// or initializing it as a leaf. When splitting, the heuristic heuristic is
// used and nodes added sequentially in the preallocated nodes array and
// the number of nodes nnodes is updated.
//
inline void make_bvh_node(ym::bvh_node* node, std::vector<ym::bvh_node>& nodes,
    bvh_bound_prim* sorted_prims, int start, int end, bool equalsize) {
    // compute node bounds
    node->bbox = ym::invalid_bbox3f;
    for (auto i = start; i < end; i++) node->bbox += sorted_prims[i].bbox;

    // decide whether to create a leaf
    if (end - start <= bvh_minprims) {
        // makes a leaf node
        node->isleaf = true;
        node->start = start;
        node->count = end - start;
    } else {
        // choose the split axis and position
        // init to default values
        auto axis = 0;
        auto mid = (start + end) / 2;

        // compute primintive bounds and size
        auto centroid_bbox = ym::invalid_bbox3f;
        for (auto i = start; i < end; i++)
            centroid_bbox += sorted_prims[i].center;
        auto centroid_size = ym::diagonal(centroid_bbox);

        // check if it is not possible to split
        if (centroid_size == ym::zero3f) {
            // we failed to split for some reasons
            node->isleaf = true;
            node->start = start;
            node->count = end - start;
        } else {
            // split along largest
            auto largest_axis = ym::max_element_idx(centroid_size);

            // check heuristic
            if (equalsize) {
                // split the space in the middle along the largest axis
                axis = largest_axis;
                mid = (int)(std::partition(sorted_prims + start,
                                sorted_prims + end,
                                bvh_bound_prim_comp(largest_axis,
                                    ym::center(centroid_bbox)[largest_axis])) -
                            sorted_prims);
            } else {
                // balanced tree split: find the largest axis of the bounding
                // box and split along this one right in the middle
                axis = largest_axis;
                mid = (start + end) / 2;
                std::nth_element(sorted_prims + start, sorted_prims + mid,
                    sorted_prims + end, bvh_bound_prim_comp(largest_axis));
            }

            // check correctness
            assert(axis >= 0 && mid > 0);
            assert(mid > start && mid < end);

            // makes an internal node
            node->isleaf = false;
            // perform the splits by preallocating the child nodes and recurring
            node->axis = axis;
            node->start = (int)nodes.size();
            node->count = 2;
            nodes.emplace_back();
            nodes.emplace_back();
            // build child nodes
            make_bvh_node(&nodes[node->start], nodes, sorted_prims, start, mid,
                equalsize);
            make_bvh_node(&nodes[node->start + 1], nodes, sorted_prims, mid,
                end, equalsize);
        }
    }
}

///
/// Build a BVH from a set of primitives.
///
template <typename ElemBbox>
inline bvh_tree* build_bvh(
    int nprims, bool equalsize, const ElemBbox& elem_bbox) {
    // allocate if needed
    auto bvh = new ym::bvh_tree();

    // prepare prims
    auto bound_prims = std::vector<bvh_bound_prim>(nprims);
    for (auto i = 0; i < nprims; i++) {
        bound_prims[i].pid = i;
        bound_prims[i].bbox = elem_bbox(i);
        bound_prims[i].center = ym::center(bound_prims[i].bbox);
    }

    // clear bvh
    bvh->nodes.clear();
    bvh->sorted_prim.clear();

    // allocate nodes (over-allocate now then shrink)
    bvh->nodes.reserve(nprims * 2);

    // start recursive splitting
    bvh->nodes.emplace_back();
    make_bvh_node(
        &bvh->nodes[0], bvh->nodes, bound_prims.data(), 0, nprims, equalsize);

    // shrink back
    bvh->nodes.shrink_to_fit();

    // init sorted element arrays
    // for shared memory, stored pointer to the external data
    // store the sorted primitive order for BVH walk
    bvh->sorted_prim.resize(nprims);
    for (int i = 0; i < nprims; i++) {
        bvh->sorted_prim[i] = bound_prims[i].pid;
    }

    // done
    return bvh;
}

///
/// Build a triangles BVH.
///
inline bvh_tree* build_triangles_bvh(int ntriangles, const vec3i* triangles,
    const vec3f* pos, bool equal_size = true) {
    return build_bvh(ntriangles, equal_size, [triangles, pos](int eid) {
        auto f = triangles[eid];
        return triangle_bbox(pos[f.x], pos[f.y], pos[f.z]);
    });
}

///
/// Build a triangles BVH.
///
inline bvh_tree* build_triangles_bvh(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& pos, bool equal_size = true) {
    return build_triangles_bvh(
        triangles.size(), triangles.data(), pos.data(), equal_size);
}

///
/// Build a quads BVH.
///
inline bvh_tree* build_quads_bvh(
    int nquads, const vec4i* quads, const vec3f* pos, bool equal_size = true) {
    return build_bvh(nquads, equal_size, [quads, pos](int eid) {
        auto f = quads[eid];
        return quad_bbox(pos[f.x], pos[f.y], pos[f.z], pos[f.w]);
    });
}

///
/// Build a quads BVH.
///
inline bvh_tree* build_quads_bvh(const std::vector<vec4i>& quads,
    const std::vector<vec3f>& pos, bool equal_size = true) {
    return build_quads_bvh(quads.size(), quads.data(), pos.data(), equal_size);
}

///
/// Build a lines BVH.
///
inline bvh_tree* build_lines_bvh(int nlines, const vec2i* lines,
    const vec3f* pos, const float* radius, bool equal_size = true) {
    return build_bvh(nlines, equal_size, [lines, pos, radius](int eid) {
        auto f = lines[eid];
        return line_bbox(pos[f.x], pos[f.y], radius[f.x], radius[f.y]);
    });
}

///
/// Build a lines BVH.
///
inline bvh_tree* build_lines_bvh(const std::vector<vec2i>& lines,
    const std::vector<vec3f>& pos, const std::vector<float>& radius,
    bool equal_size = true) {
    return build_lines_bvh(
        lines.size(), lines.data(), pos.data(), radius.data(), equal_size);
}

///
/// Build a points BVH.
///
inline bvh_tree* build_points_bvh(int npoints, const int* points,
    const vec3f* pos, const float* radius, bool equal_size = true) {
    return build_bvh(npoints, equal_size, [points, pos, radius](int eid) {
        auto f = points[eid];
        return point_bbox(pos[f], radius[f]);
    });
}

///
/// Build a points BVH.
///
inline bvh_tree* build_points_bvh(const std::vector<int>& points,
    const std::vector<vec3f>& pos, const std::vector<float>& radius,
    bool equal_size = true) {
    return build_points_bvh(
        points.size(), points.data(), pos.data(), radius.data(), equal_size);
}

///
/// Build a points BVH.
///
inline bvh_tree* build_points_bvh(int npoints, const vec3f* pos,
    const float* radius, bool equal_size = true) {
    return build_bvh(npoints, equal_size, [pos, radius](int eid) {
        return point_bbox(pos[eid], (radius) ? radius[eid] : 0);
    });
}

///
/// Build a points BVH.
///
inline bvh_tree* build_points_bvh(int npoints, const std::vector<vec3f>& pos,
    const std::vector<float>& radius, bool equal_size = true) {
    return build_points_bvh(npoints, pos.data(), radius.data(), equal_size);
}

///
/// Recursively recomputes the node bounds for a shape bvh
///
template <typename ElemBbox>
inline void refit_bvh(bvh_tree* bvh, int nodeid, const ElemBbox& elem_bbox) {
    // refit
    auto node = &bvh->nodes[nodeid];
    node->bbox = ym::invalid_bbox3f;
    if (node->isleaf) {
        for (auto i = 0; i < node->count; i++) {
            auto idx = bvh->sorted_prim[node->start + i];
            node->bbox += elem_bbox(idx);
        }
    } else {
        for (auto i = 0; i < node->count; i++) {
            auto idx = node->start + i;
            refit_bvh(bvh, idx, elem_bbox);
            node->bbox += bvh->nodes[idx].bbox;
        }
    }
}

///
/// Refit triangles bvh
///
inline void refit_triangles_bvh(
    bvh_tree* bvh, const vec3i* triangles, const vec3f* pos) {
    refit_bvh(bvh, 0, [triangles, pos](int eid) {
        auto f = triangles[eid];
        return triangle_bbox(pos[f.x], pos[f.y], pos[f.z]);
    });
}

///
/// Refit triangles bvh
///
inline void refit_triangles_bvh(bvh_tree* bvh,
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& pos) {
    refit_triangles_bvh(bvh, triangles.data(), pos.data());
}

///
/// Refit quads bvh
///
inline void refit_quads_bvh(
    bvh_tree* bvh, const vec4i* quads, const vec3f* pos) {
    refit_bvh(bvh, 0, [quads, pos](int eid) {
        auto f = quads[eid];
        return quad_bbox(pos[f.x], pos[f.y], pos[f.z], pos[f.w]);
    });
}

///
/// Refit quads bvh
///
inline void refit_quads_bvh(bvh_tree* bvh, const std::vector<vec4i>& quads,
    const std::vector<vec3f>& pos) {
    refit_quads_bvh(bvh, quads.data(), pos.data());
}

///
/// Refit lines bvh
///
inline void refit_lines_bvh(
    bvh_tree* bvh, const vec2i* lines, const vec3f* pos, const float* radius) {
    ym::refit_bvh(bvh, 0, [lines, pos, radius](int eid) {
        auto f = lines[eid];
        return line_bbox(pos[f.x], pos[f.y], radius[f.x], radius[f.y]);
    });
}

///
/// Refit lines bvh
///
inline void refit_lines_bvh(bvh_tree* bvh, const std::vector<vec2i>& lines,
    const std::vector<vec3f>& pos, const std::vector<float>& radius) {
    refit_lines_bvh(bvh, lines.data(), pos.data(), radius.data());
}

///
/// Refit points bvh
///
inline void refit_points_bvh(
    bvh_tree* bvh, const int* points, const vec3f* pos, const float* radius) {
    refit_bvh(bvh, 0, [points, pos, radius](int eid) {
        auto f = points[eid];
        return point_bbox(pos[f], (radius) ? radius[f] : 0);
    });
}

///
/// Refit points bvh
///
inline void refit_points_bvh(bvh_tree* bvh, const std::vector<int>& points,
    const std::vector<vec3f>& pos, const std::vector<float>& radius) {
    refit_points_bvh(bvh, points.data(), pos.data(), radius.data());
}
///
/// Refit points bvh
///
inline void refit_points_bvh(
    bvh_tree* bvh, const vec3f* pos, const float* radius) {
    refit_bvh(bvh, 0,
        [pos, radius](int eid) { return point_bbox(pos[eid], radius[eid]); });
}

///
/// Refit lines bvh
///
inline void refit_points_bvh(bvh_tree* bvh, const std::vector<vec3f>& pos,
    const std::vector<float>& radius) {
    refit_points_bvh(bvh, pos.data(), radius.data());
}

///
/// Intersect ray with a bvh.
///
template <typename Isec>
inline bool intersect_bvh(const bvh_tree* bvh, const ym::ray3f& ray_,
    bool early_exit, float& ray_t, int& eid, const Isec& intersect_elem) {
    // node stack
    int node_stack[64];
    auto node_cur = 0;
    node_stack[node_cur++] = 0;

    // shared variables
    auto hit = false;

    // copy ray to modify it
    auto ray = ray_;

    // prepare ray for fast queries
    auto ray_dinv = ym::vec3f{1, 1, 1} / ray.d;
    auto ray_dsign = ym::vec3i{(ray_dinv.x < 0) ? 1 : 0,
        (ray_dinv.y < 0) ? 1 : 0, (ray_dinv.z < 0) ? 1 : 0};
    auto ray_reverse = ym::vec<bool, 4>{
        (bool)ray_dsign.x, (bool)ray_dsign.y, (bool)ray_dsign.z, false};

    // walking stack
    while (node_cur) {
        // grab node
        auto node = bvh->nodes[node_stack[--node_cur]];

        // intersect bbox
        if (!ym::intersect_check_bbox(ray, ray_dinv, ray_dsign, node.bbox))
            continue;

        // intersect node, switching based on node type
        // for each type, iterate over the the primitive list
        if (!node.isleaf) {
            // for internal nodes, attempts to proceed along the
            // split axis from smallest to largest nodes
            if (ray_reverse[node.axis]) {
                for (auto i = 0; i < node.count; i++) {
                    auto idx = node.start + i;
                    node_stack[node_cur++] = idx;
                    assert(node_cur < 64);
                }
            } else {
                for (auto i = node.count - 1; i >= 0; i--) {
                    auto idx = node.start + i;
                    node_stack[node_cur++] = idx;
                    assert(node_cur < 64);
                }
            }
        } else {
            for (auto i = 0; i < node.count; i++) {
                auto idx = bvh->sorted_prim[node.start + i];
                if (intersect_elem(idx, ray, ray_t)) {
                    if (early_exit) return true;
                    ray.tmax = ray_t;
                    hit = true;
                    eid = idx;
                }
            }
        }
    }

    return hit;
}

///
/// Finds the closest element with a bvh.
///
template <typename OverlapElem>
inline bool overlap_bvh(const bvh_tree* bvh, const ym::vec3f& pos,
    float max_dist, bool early_exit, float& dist, int& eid,
    const OverlapElem& overlap_elem) {
    // node stack
    int node_stack[64];
    auto node_cur = 0;
    node_stack[node_cur++] = 0;

    // hit
    auto hit = false;

    // walking stack
    while (node_cur) {
        // grab node
        auto node = bvh->nodes[node_stack[--node_cur]];

        // intersect bbox
        if (!ym::distance_check_bbox(pos, max_dist, node.bbox)) continue;

        // intersect node, switching based on node type
        // for each type, iterate over the the primitive list
        if (!node.isleaf) {
            // internal node
            for (auto idx = node.start; idx < node.start + node.count; idx++) {
                node_stack[node_cur++] = idx;
                assert(node_cur < 64);
            }
        } else {
            for (auto i = 0; i < node.count; i++) {
                auto idx = bvh->sorted_prim[node.start + i];
                if (overlap_elem(idx, pos, max_dist, dist)) {
                    if (early_exit) return true;
                    max_dist = dist;
                    eid = idx;
                }
            }
        }
    }

    return hit;
}

///
/// Intersect a triangle BVH
///
inline bool intersect_triangles_bvh(const bvh_tree* bvh, const vec3i* triangles,
    const vec3f* pos, const ray3f& ray, bool early_exit, float& ray_t, int& eid,
    vec3f& euv) {
    return ym::intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&triangles, &pos, &euv](int eid, const ym::ray3f& ray, float& ray_t) {
            const auto& f = triangles[eid];
            return intersect_triangle(
                ray, pos[f.x], pos[f.y], pos[f.z], ray_t, euv);
        });
}

///
/// Intersect a triangle BVH
///
inline bool intersect_triangles_bvh(const bvh_tree* bvh,
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& pos,
    const ray3f& ray, bool early_exit, float& ray_t, int& eid, vec3f& euv) {
    return intersect_triangles_bvh(
        bvh, triangles.data(), pos.data(), ray, early_exit, ray_t, eid, euv);
}

///
/// Intersect a quad BVH
///
inline bool intersect_quads_bvh(const bvh_tree* bvh, const vec4i* quads,
    const vec3f* pos, const ray3f& ray, bool early_exit, float& ray_t, int& eid,
    vec4f& euv) {
    return ym::intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&quads, &pos, &euv](int eid, const ym::ray3f& ray, float& ray_t) {
            const auto& f = quads[eid];
            return intersect_quad(
                ray, pos[f.x], pos[f.y], pos[f.z], pos[f.w], ray_t, euv);
        });
}

///
/// Intersect a quad BVH
///
inline bool intersect_quads_bvh(const bvh_tree* bvh,
    const std::vector<vec4i>& quads, const std::vector<vec3f>& pos,
    const ray3f& ray, bool early_exit, float& ray_t, int& eid, vec4f& euv) {
    return intersect_quads_bvh(
        bvh, quads.data(), pos.data(), ray, early_exit, ray_t, eid, euv);
}

///
/// Intersect a line BVH
///
inline bool intersect_lines_bvh(const bvh_tree* bvh, const vec2i* lines,
    const vec3f* pos, const float* radius, const ray3f& ray, bool early_exit,
    float& ray_t, int& eid, vec2f& euv) {
    return ym::intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&lines, &pos, &radius, &euv](
            int eid, const ym::ray3f& ray, float& ray_t) {
            auto f = lines[eid];
            return intersect_line(
                ray, pos[f.x], pos[f.y], radius[f.x], radius[f.y], ray_t, euv);
        });
}

///
/// Intersect a line BVH
///
inline bool intersect_lines_bvh(const bvh_tree* bvh,
    const std::vector<vec2i>& lines, const std::vector<vec3f>& pos,
    const std::vector<float>& radius, const ray3f& ray, bool early_exit,
    float& ray_t, int& eid, vec2f& euv) {
    return intersect_lines_bvh(bvh, lines.data(), pos.data(), radius.data(),
        ray, early_exit, ray_t, eid, euv);
}

///
/// Intersect a point BVH
///
inline bool intersect_points_bvh(const bvh_tree* bvh, const int* points,
    const vec3f* pos, const float* radius, const ray3f& ray, bool early_exit,
    float& ray_t, int& eid) {
    return intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&points, &pos, &radius](int eid, const ym::ray3f& ray, float& ray_t) {
            auto f = points[eid];
            return ym::intersect_point(ray, pos[f], radius[f], ray_t);
        });
}

///
/// Intersect a point BVH
///
inline bool intersect_points_bvh(const bvh_tree* bvh,
    const std::vector<int>& points, const std::vector<vec3f>& pos,
    const std::vector<float>& radius, const ray3f& ray, bool early_exit,
    float& ray_t, int& eid) {
    return intersect_points_bvh(bvh, points.data(), pos.data(), radius.data(),
        ray, early_exit, ray_t, eid);
}

///
/// Intersect a point BVH
///
inline bool intersect_points_bvh(const bvh_tree* bvh, const vec3f* pos,
    const float* radius, const ray3f& ray, bool early_exit, float& ray_t,
    int& eid) {
    return intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&pos, &radius](int eid, const ym::ray3f& ray, float& ray_t) {
            return intersect_point(ray, pos[eid], radius[eid], ray_t);
        });
}

///
/// Intersect a point BVH
///
inline bool intersect_points_bvh(const bvh_tree* bvh,
    const std::vector<vec3f>& pos, const std::vector<float>& radius,
    const ray3f& ray, bool early_exit, float& ray_t, int& eid) {
    return intersect_points_bvh(
        bvh, pos.data(), radius.data(), ray, early_exit, ray_t, eid);
}

///
/// Intersect a triangle BVH
///
inline bool overlap_triangles_bvh(const bvh_tree* bvh, const vec3i* triangles,
    const vec3f* pos, const float* radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec3f& euv) {
    return ym::overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&triangles, &pos, &radius, &euv](
            int eid, const ym::vec3f& pt, float max_dist, float& dist) {
            auto f = triangles[eid];
            return overlap_triangle(pt, max_dist, pos[f.x], pos[f.y], pos[f.z],
                (radius) ? radius[f.x] : 0, (radius) ? radius[f.y] : 0,
                (radius) ? radius[f.z] : 0, dist, euv);
        });
}

///
/// Intersect a triangle BVH
///
inline bool overlap_triangles_bvh(const bvh_tree* bvh,
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& pos,
    const std::vector<float>& radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec3f& euv) {
    return overlap_triangles_bvh(bvh, triangles.data(), pos.data(),
        radius.data(), pt, max_dist, early_exit, dist, eid, euv);
}

///
/// Intersect a quad BVH
///
inline bool overlap_quads_bvh(const bvh_tree* bvh, const vec4i* quads,
    const vec3f* pos, const float* radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec4f& euv) {
    return ym::overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&quads, &pos, &radius, &euv](
            int eid, const ym::vec3f& pt, float max_dist, float& dist) {
            auto f = quads[eid];
            return overlap_quad(pt, max_dist, pos[f.x], pos[f.y], pos[f.z],
                pos[f.w], (radius) ? radius[f.x] : 0,
                (radius) ? radius[f.y] : 0, (radius) ? radius[f.z] : 0,
                (radius) ? radius[f.w] : 0, dist, euv);
        });
}

///
/// Intersect a quad BVH
///
inline bool overlap_quads_bvh(const bvh_tree* bvh,
    const std::vector<vec4i>& quads, const std::vector<vec3f>& pos,
    const std::vector<float>& radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec4f& euv) {
    return overlap_quads_bvh(bvh, quads.data(), pos.data(), radius.data(), pt,
        max_dist, early_exit, dist, eid, euv);
}

///
/// Intersect a line BVH
///
inline bool overlap_lines_bvh(const bvh_tree* bvh, const vec2i* lines,
    const vec3f* pos, const float* radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec2f& euv) {
    return ym::overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&lines, &pos, &radius, &euv](
            int eid, const ym::vec3f& pt, float max_dist, float& dist) {
            auto f = lines[eid];
            return overlap_line(pt, max_dist, pos[f.x], pos[f.y],
                (radius) ? radius[f.x] : 0, (radius) ? radius[f.y] : 0, dist,
                euv);
        });
}

///
/// Intersect a line BVH
///
inline bool overlap_lines_bvh(const bvh_tree* bvh,
    const std::vector<vec2i>& lines, const std::vector<vec3f>& pos,
    const std::vector<float>& radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec2f& euv) {
    return overlap_lines_bvh(bvh, lines.data(), pos.data(), radius.data(), pt,
        max_dist, early_exit, dist, eid, euv);
}

///
/// Intersect a point BVH
///
inline bool overlap_points_bvh(const bvh_tree* bvh, const int* points,
    const vec3f* pos, const float* radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid) {
    return overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&points, &pos, &radius](
            int eid, const ym::vec3f& pt, float max_dist, float& dist) {
            auto f = points[eid];
            return ym::overlap_point(
                pt, max_dist, pos[f], (radius) ? radius[f] : 0, dist);
        });
}

///
/// Intersect a point BVH
///
inline bool overlap_points_bvh(const bvh_tree* bvh,
    const std::vector<int>& points, const std::vector<vec3f>& pos,
    const std::vector<float>& radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid) {
    return overlap_points_bvh(bvh, points.data(), pos.data(), radius.data(), pt,
        max_dist, early_exit, dist, eid);
}

///
/// Intersect a point BVH
///
inline bool overlap_points_bvh(const bvh_tree* bvh, const vec3f* pos,
    const float* radius, const vec3f& pt, float max_dist, bool early_exit,
    float& dist, int& eid) {
    return overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&pos, &radius](
            int eid, const ym::vec3f& pt, float max_dist, float& dist) {
            return overlap_point(pt, max_dist, pos[eid], radius[eid], dist);
        });
}

///
/// Intersect a point BVH
///
inline bool overlap_points_bvh(const bvh_tree* bvh,
    const std::vector<vec3f>& pos, const std::vector<float>& radius,
    const vec3f& pt, float max_dist, bool early_exit, float& dist, int& eid) {
    return overlap_points_bvh(
        bvh, pos.data(), radius.data(), pt, max_dist, early_exit, dist, eid);
}

///
/// Finds the overlap between BVH leaf nodes.
///
template <typename OverlapElem>
void overlap_bvh_elems(const bvh_tree* bvh1, const bvh_tree* bvh2,
    bool skip_duplicates, bool skip_self, std::vector<ym::vec2i>& overlaps,
    const OverlapElem& overlap_elems) {
    // node stack
    ym::vec2i node_stack[128];
    auto node_cur = 0;
    node_stack[node_cur++] = {0, 0};

    // walking stack
    while (node_cur) {
        // grab node
        auto node_idx = node_stack[--node_cur];
        const auto node1 = bvh1->nodes[node_idx.x];
        const auto node2 = bvh2->nodes[node_idx.y];

        // intersect bbox
        if (!overlap_bbox(node1.bbox, node2.bbox)) continue;

        // check for leaves
        if (node1.isleaf && node2.isleaf) {
            // collide primitives
            for (auto i1 = node1.start; i1 < node1.start + node1.count; i1++) {
                for (auto i2 = node2.start; i2 < node2.start + node2.count;
                     i2++) {
                    auto idx1 = bvh1->sorted_prim[i1];
                    auto idx2 = bvh2->sorted_prim[i2];
                    if (skip_duplicates && idx1 > idx2) continue;
                    if (skip_self && idx1 == idx2) continue;
                    if (overlap_elems(idx1, idx2))
                        overlaps.push_back({idx1, idx2});
                }
            }
        } else {
            // descend
            if (node1.isleaf) {
                for (auto idx2 = node2.start; idx2 < node2.start + node2.count;
                     idx2++) {
                    node_stack[node_cur++] = {node_idx.x, (int)idx2};
                    assert(node_cur < 128);
                }
            } else if (node2.isleaf) {
                for (auto idx1 = node1.start; idx1 < node1.start + node1.count;
                     idx1++) {
                    node_stack[node_cur++] = {(int)idx1, node_idx.y};
                    assert(node_cur < 128);
                }
            } else {
                for (auto idx2 = node2.start; idx2 < node2.start + node2.count;
                     idx2++) {
                    for (auto idx1 = node1.start;
                         idx1 < node1.start + node1.count; idx1++) {
                        node_stack[node_cur++] = {(int)idx1, (int)idx2};
                        assert(node_cur < 128);
                    }
                }
            }
        }
    }
}

}  // namespace ym

// HACK to avoid compilation with MSVC2015 without dirtying code
#ifdef constexpr
#undef constexpr
#endif

#endif
