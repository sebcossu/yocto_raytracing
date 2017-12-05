///
/// # Yocto/Obj
///
/// Wavefront OBJ/MTL loader and writer with support for points,
/// lines, triangles and general polygons and all materials properties.
/// Contains also a few extensions to easily create demos such as per-vertex
/// color and radius, cameras, environment maps and instances.
/// Can use either a low-level OBJ representation, from this files,
/// or a high level flattened representation included in Yocto/Scn.
///
/// Both in reading and writing, OBJ has no clear convention on the orientation
/// of textures Y axis. So in many cases textures appears flipped. To handle
/// that, use the option to flip textures coordinates on either saving or
/// loading. By default texture coordinates are flipped since this seems
/// the convention found on test cases collected on the web. The value Tr
/// has similar problems, since its relation to opacity is software specific.
/// Again we let the user chose the convension and set the default to the
/// one found on the web.
///
/// In the high level interface, shapes are indexed meshes and are described
/// by arrays of vertex indices for points/lines/triangles and arrays for vertex
/// positions, normals, texcoords, color and radius. The latter two as
/// extensions. Since OBJ is a complex formats that does not match well with
/// current GPU rendering / path tracing algorithms, we adopt a simplification
/// similar to other single file libraries:
/// 1. vertex indices are unique, as in OpenGL and al standard indexed triangle
///   meshes data structures, and not OBJ triplets; YOCTO_OBJ ensures that no
///   vertex dusplication happens thought for same triplets
/// 2. we split shapes on changes to groups and materials, instead of keeping
///   per-face group/material data; this makes the data usable right away in
///   a GPU viewer; this is not a major limitation if we accept the previous
///   point that already changes shapes topology.
///
/// This library depends in yocto_math.h. Texture loading depends on
/// yocto_image. If the texture loading dependency is not desired, it can be
/// disabled by defining YOBJ_NO_IMAGE before including this file.
///
///
/// ## Usage
///
/// 1. load a obj data with `load_obj()`; can load also textues
/// 2. look at the `obj_XXX` data structures for access to individual elements
/// 3. use obj back to disk with `save_obj()`; can also save textures
/// 4. use get_shape() to get a flattened shape version that contains only
///    triangles, lines or points
///
///
/// For easier access, please consider using Yocto/Scn.
///
///
/// ## History
///
/// - v 0.32: move modified high-level interface to Yocto/Scn
/// - v 0.31: can load textures with the low level interface
/// - v 0.30: support for smoothing groups
/// - v 0.29: use reference interface for textures
/// - v 0.28: add function to split meshes into single shapes
/// - v 0.27: explicit transforms
/// - v 0.26: added interpreting of illum in scene conversions
/// - v 0.25: added convention for Tr
/// - v 0.24: remove exception from code and add explicit error handling
/// - v 0.23: texture have always 4 channels
/// - v 0.22: change variable names for compilation on gcc
/// - v 0.21: bug fixes
/// - v 0.20: use yocto_math in the interface and remove inline compilation
/// - v 0.19: add missing bounding box computation and missing data functions
/// - v 0.18: prioritize high-level interface
/// - v 0.17: name cleanup in both interface to better align with glTF
/// - v 0.16: change flattened data structure to use pointers
/// - v 0.15: added unknown properties std::map for materials
/// - v 0.14: added extension to store tetrahedral meshes
/// - v 0.13: started adding physics extension to materials
/// - v 0.12: change texture loading by flipping uvs rather than images
/// - v 0.11: use yocto_image for texture handling.
/// - v 0.10: switch to .h/.cpp pair
/// - v 0.9: bug fixes and optionally texture skipping
/// - v 0.8: high level interface uses grouping
/// - v 0.7: doxygen comments
/// - v 0.6: bug fixes
/// - v 0.5: removed options to force image formats (image library not reliable)
/// - v 0.4: [major API change] move to modern C++ interface
/// - v 0.3: new API internals and C++ interface
/// - v 0.2: removal of C interface
/// - v 0.1: C++ implementation
/// - v 0.0: initial release in C99
///
namespace yobj {}

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

#ifndef _YOBJ_H_
#define _YOBJ_H_

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "yocto_math.h"

// -----------------------------------------------------------------------------
// INTERFACE
// -----------------------------------------------------------------------------

///
/// Reading and Writing support for Wavefront OBJ.
///
namespace yobj {

///
/// Property map
///
template <typename T>
using property_map = std::unordered_map<std::string, std::vector<T>>;

///
/// Face vertex
///
struct vertex {
    /// position
    int pos;
    /// texcoord
    int texcoord;
    /// normal
    int norm;
    /// color [extension]
    int color;
    /// radius [extension]
    int radius;

    /// Constructor (copies members initializing missing ones to -1)
    vertex(int pos = -1, int texcoord = -1, int norm = -1, int color = -1,
        int radius = -1)
        : pos(pos)
        , texcoord(texcoord)
        , norm(norm)
        , color(color)
        , radius(radius) {}
};

//
// Comparison for unordred_map
//
inline bool operator==(const vertex& a, const vertex& b) {
    return a.pos == b.pos && a.texcoord == b.texcoord && a.norm == b.norm &&
           a.color == b.color && a.radius == b.radius;
}

///
/// element type
///
enum struct element_type : uint16_t {
    /// lists of points
    point = 1,
    /// polylines
    line = 2,
    /// polygon faces
    face = 3,
    /// tetrahedrons
    tetra = 4,
};

///
/// Element vertex indices
///
struct element {
    /// starting vertex index
    uint32_t start;
    /// element type
    element_type type;
    /// number of vertices
    uint16_t size;
};

///
/// Element group
///
struct group {
    // group data ---------------------------
    /// material name
    std::string matname;
    /// group name
    std::string groupname;
    /// smoothing
    bool smoothing = true;

    // element data -------------------------
    /// element vertices
    std::vector<vertex> verts;
    /// element faces
    std::vector<element> elems;
};

///
/// Obj object
///
struct object {
    // object data --------------------------
    /// object name
    std::string name;

    // element data -------------------------
    /// element groups
    std::vector<group> groups;
};

///
/// OBJ texture. Texture data is loaded only if desired.
///
struct texture {
    // whole texture data ------------------
    /// texture path
    std::string path;
    /// Width
    int width = 0;
    /// Height
    int height = 0;
    /// Number of Channels
    int ncomp = 0;
    /// Buffer data for 8-bit images
    std::vector<uint8_t> datab;
    /// Buffer data for float images
    std::vector<float> dataf;
};

///
/// OBJ material
///
struct material {
    // whole material data ------------------
    /// material name
    std::string name;
    /// MTL illum mode
    int illum = 0;

    // color information --------------------
    /// emission color
    ym::vec3f ke = {0, 0, 0};
    /// ambient color
    ym::vec3f ka = {0, 0, 0};
    /// diffuse color
    ym::vec3f kd = {0, 0, 0};
    /// specular color
    ym::vec3f ks = {0, 0, 0};
    /// reflection color
    ym::vec3f kr = {0, 0, 0};
    /// transmision color
    ym::vec3f kt = {0, 0, 0};
    /// phong exponent for ks
    float ns = 1;
    /// index of refraction
    float ior = 1;
    /// opacity
    float op = 1;

    // texture names for the above properties
    /// emission texture
    std::string ke_txt;
    /// ambient texture
    std::string ka_txt;
    /// diffuse texture
    std::string kd_txt;
    /// specular texture
    std::string ks_txt;
    /// reflection texture
    std::string kr_txt;
    /// transmission texture
    std::string kt_txt;
    /// specular exponent texture
    std::string ns_txt;
    /// opacity texture
    std::string op_txt;
    /// index of refraction
    std::string ior_txt;
    /// bump map texture (heighfield)
    std::string bump_txt;
    /// displacement map texture (heighfield)
    std::string disp_txt;
    /// normal map texture
    std::string norm_txt;

    // texture information ---------------------
    /// emission texture
    property_map<std::string> ke_txt_info = {};
    /// ambient texture
    property_map<std::string> ka_txt_info = {};
    /// diffuse texture
    property_map<std::string> kd_txt_info = {};
    /// specular texture
    property_map<std::string> ks_txt_info = {};
    /// reflection texture
    property_map<std::string> kr_txt_info = {};
    /// transmission texture
    property_map<std::string> kt_txt_info = {};
    /// specular exponent texture
    property_map<std::string> ns_txt_info = {};
    /// opacity texture
    property_map<std::string> op_txt_info = {};
    /// index of refraction
    property_map<std::string> ior_txt_info = {};
    /// bump map texture (heighfield)
    property_map<std::string> bump_txt_info = {};
    /// displacement map texture (heighfield)
    property_map<std::string> disp_txt_info = {};
    /// normal texture
    property_map<std::string> norm_txt_info = {};

    // unknown properties ---------------------
    /// unknown string props
    property_map<std::string> unknown_props;
};

///
/// Camera [extension]
///
struct camera {
    /// camera name
    std::string name;
    /// transform frame (affine matrix)
    ym::frame3f frame = ym::identity_frame3f;
    /// orthografic camera
    bool ortho = false;
    /// vertical field of view
    float yfov = 2 * std::atan(0.5f);
    /// aspect ratio
    float aspect = 16.0f / 9.0f;
    /// lens aperture
    float aperture = 0;
    /// focus distance
    float focus = 1;
};

///
/// Environment [extension]
///
struct environment {
    /// environment name
    std::string name;
    /// transform frame (affine matrix)
    ym::frame3f frame = ym::identity_frame3f;
    /// material name
    std::string matname;
};

///
/// Instance [extension]
///
struct instance {
    /// instance name
    std::string name;
    /// transform frame (affine matrix)
    ym::frame3f frame = ym::identity_frame3f;
    /// object name
    std::string objname;
};

///
/// OBJ asset
///
struct scene {
    // vertex data -------------------------
    /// vertex positions
    std::vector<ym::vec3f> pos;
    /// vertex normals
    std::vector<ym::vec3f> norm;
    /// vertex texcoord
    std::vector<ym::vec2f> texcoord;
    /// vertex color [extension]
    std::vector<ym::vec4f> color;
    /// vertex radius [extension]
    std::vector<float> radius;

    // scene objects -----------------------
    /// objects
    std::vector<object> objects;
    /// materials
    std::vector<material> materials;
    /// textures
    std::vector<texture> textures;
    /// cameras [extension]
    std::vector<camera> cameras;
    /// env maps [extension]
    std::vector<environment> environments;
    /// instances [extension]
    std::vector<instance> instances;
};

///
/// Load OBJ
///
/// - Parameters:
///     - filename: filename
///     - load_texture: whether to load textures
///     - skip_missing: whether to skip missing files
///     - flip_texcoord: whether to flip the v coordinate
///     - flip_tr: whether to flip the Tr value
/// - Return:
///     - obj (nullptr on error)
///
scene* load_obj(const std::string& filename, bool load_textures = false,
    bool skip_missing = false, bool flip_texcoord = true, bool flip_tr = true);

///
/// Save OBJ
///
/// - Parameters:
///     - filename: filename
///     - model: obj data to save
///     - save_textures: whether to save textures
///     - skip_missing: whether to skip missing files
///     - flip_texcoord: whether to flip the v coordinate
///     - flip_tr: whether to flip the Tr value
/// - Returns:
///     - whether an error occurred
///
void save_obj(const std::string& filename, const scene* model,
    bool save_textures = false, bool skip_missing = false,
    bool flip_texcoord = true, bool flip_tr = true);

///
/// Shape. May contain only one of the points/lines/triangles.
///
struct shape {
    /// name of the group that enclosed it
    std::string name = "";
    /// name of the material
    std::string matname = "";

    // shape elements -------------------------
    /// points
    std::vector<int> points;
    /// lines
    std::vector<ym::vec2i> lines;
    /// triangles
    std::vector<ym::vec3i> triangles;
    /// tetrahedrons
    std::vector<ym::vec4i> tetras;

    // vertex data ----------------------------
    /// per-vertex position (3 float)
    std::vector<ym::vec3f> pos;
    /// per-vertex normals (3 float)
    std::vector<ym::vec3f> norm;
    /// per-vertex texcoord (2 float)
    std::vector<ym::vec2f> texcoord;
    /// [extension] per-vertex color (4 float)
    std::vector<ym::vec4f> color;
    /// [extension] per-vertex radius (1 float)
    std::vector<float> radius;
};

///
/// Mesh
///
struct mesh {
    // name
    std::string name;
    /// primitives
    std::vector<shape> shapes;

    /// cleanup
    ~mesh();
};

///
/// Gets a mesh from an OBJ object.
///
mesh* get_mesh(const scene* model, const object& oobj, bool facet_non_smooth);

}  // namespace yobj

#endif
