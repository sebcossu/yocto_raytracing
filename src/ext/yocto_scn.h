///
/// # Yocto/Scn
///
/// Simple scene model used by other parts of Yocto. Support conversion to/from
/// Wavefront OBJ and Khronos glTF. Support ray intersection and closest point
/// queries.
///
/// The geometry model is comprised of a set of shapes, which are indexed
/// collections of points, lines, triangles and quads. Each shape may contain
/// only one element type. Shapes are organized into a scene by creating shape
/// instances, each its own transform. Materials are specified like in glTF and
/// include emission, base-metallic and diffuse-specular parametrization,
/// normal, occlusion and displacement mapping. Finally, the scene containes
/// caemras and environement maps. Quad support in shapes is experimental and
/// mostly supported for loading and saving.
///
/// For low-level access to OBJ/glTF formats, you are best accssing the formats
/// directly with Yocto/Obj and Yocto/glTF. This library provides _simplified_
/// high-level access to each format which is sufficient for most applications
/// and tuned for quick creating viewers, renderers and simulators.
///
/// Ray-intersection and closet-point routines supporting points,
/// lines and triangles accelerated by a two-level bounding volume
/// hierarchy (BVH). Quad support is experimental.
///
/// This library depends on yocto_math.h. For loading and saving, the library
/// depends on yocto_gltf.{h,cpp} and yocto_obj.{h,cpp}. These latter
/// dependencies can be disabled by setting YSCN_NO_GLTF or YSCN_NO_OBJ.
/// The library depends also on yocto_image.{h,cpp} for texture loading and
/// saving. You can disable this dependency with YSCN_NO_IMAGE.
///
///
/// ## Usage for loading and saving
///
/// 1. load a scene with `load_scene()` and save it with `save_scene()`.
/// 2. add missing data with `add_elements()`
/// 3. use `compute_bounds()` to compute element bounds
/// 4. can merge scene together with `merge_into()`
///
/// Most of the functions listed above have their own configuration parameters
/// for quick accessing.
///
///
/// ## Usage for ray intersection and closest point queries
///
/// 1. build the bvh with `build_bvh()`
/// 2. perform ray-interseciton tests with `intersect_ray()`
///     - use early_exit=false if you want to know the closest hit point
///     - use early_exit=false if you only need to know whether there is a hit
///     - for points and lines, a radius is required
///     - for triangles, the radius is ignored
/// 2. perform point overlap tests with `overlap_point()` to check whether
///    a point overlaps with an element within a maximum distance
///     - use early_exit as above
///     - for all primitives, a radius is used if defined, but should
///       be very small compared to the size of the primitive since the radius
///       overlap is approximate
/// 3. perform instance overlap queries with `overlap_instance_bounds()`
/// 4. use `refit_bvh()` to recompute the bvh bounds if transforms or vertices
///    are changed (you should rebuild the bvh for large changes)
///
///
/// ## History
///
/// - v 0.0: initial release
///
namespace yscn {}

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

#ifndef _YSCN_H_
#define _YSCN_H_

#include "yocto_math.h"

#include <string>
#include <vector>

///
/// Scene representation for conversion to/from OBJ and glTF.
///
namespace yscn {

///
/// Property map
///
template <typename T>
using property_map = std::unordered_map<std::string, std::vector<T>>;

///
/// Scene Texture
///
struct texture {
    /// name
    std::string name;
    /// path
    std::string path;
    /// if loaded, ldr image
    ym::image4b ldr;
    /// if loaded, hdr image
    ym::image4f hdr;

    /// get texture width
    int width() const {
        if (ldr) return ldr.width();
        if (hdr) return hdr.width();
        return 0;
    }
    /// get texture height
    int height() const {
        if (ldr) return ldr.height();
        if (hdr) return hdr.height();
        return 0;
    }
};

///
/// Texture wrap mode
///
enum struct texture_wrap {
    /// repeat
    repeat = 1,
    /// clamp
    clamp = 2,
    /// mirror
    mirror = 3,
};

///
/// Texture filter mode
///
enum struct texture_filter {
    /// linear
    linear = 1,
    /// nearest
    nearest = 2,
    /// linear mipmap linear
    linear_mipmap_linear = 3,
    /// nearest mipmap nearest
    nearest_mipmap_nearest = 4,
    /// linear mipmap nearest
    linear_mipmap_nearest = 5,
    /// nearest mipmap linear
    nearest_mipmap_linear = 6,
};

///
/// Scene Texture Additional Information
///
struct texture_info {
    /// texture pointer
    texture* txt = nullptr;
    /// wrap mode for s coordinate
    texture_wrap wrap_s = texture_wrap::repeat;
    /// wrap mdoe for t coordinate
    texture_wrap wrap_t = texture_wrap::repeat;
    /// filter mode
    texture_filter filter_mag = texture_filter::linear;
    /// filter mode
    texture_filter filter_min = texture_filter::linear_mipmap_linear;
    /// texture strength (occlusion and normal)
    float scale = 1;
    /// unknown string props
    property_map<std::string> unknown_props;

    /// check whether the texture if present
    operator bool() const { return (bool)txt; }
};

///
/// Material type
///
enum struct material_type {
    /// Microfacet material type (OBJ)
    specular_roughness = 0,
    /// Base and metallic material (metallic-roughness in glTF)
    metallic_roughness = 1,
    /// Diffuse and specular material (specular-glossness in glTF)
    specular_glossiness = 2,
};

///
/// Scene Material
///
struct material {
    // whole material data -------------------
    /// material name
    std::string name;
    /// double-sided rendering
    bool double_sided = false;
    /// material type
    material_type mtype = material_type::specular_roughness;

    // color information ---------------------
    /// emission color
    ym::vec3f ke = {0, 0, 0};
    /// diffuse color
    ym::vec3f kd = {0, 0, 0};
    /// specular color
    ym::vec3f ks = {0, 0, 0};
    /// reflection color
    ym::vec3f kr = {0, 0, 0};
    /// base color
    ym::vec3f kb = {0, 0, 0};
    /// metallic factor
    float km = 0;
    /// transmission color
    ym::vec3f kt = {0, 0, 0};
    /// roughness
    float rs = 0.0001;
    /// opacity
    float op = 1;

    // textures -------------------------------
    /// emission texture
    texture_info ke_txt = {};
    /// diffuse texture
    texture_info kd_txt = {};
    /// specular texture
    texture_info ks_txt = {};
    /// reflection texture
    texture_info kr_txt = {};
    /// base texture
    texture_info kb_txt = {};
    /// metallic texture
    texture_info km_txt = {};
    /// transmission texture
    texture_info kt_txt = {};
    /// roughness texture
    texture_info rs_txt = {};
    /// bump map texture (heighfield)
    texture_info bump_txt = {};
    /// displacement map texture (heighfield)
    texture_info disp_txt = {};
    /// normal texture
    texture_info norm_txt = {};
    /// occlusion texture
    texture_info occ_txt = {};

    // unknown properties ---------------------
    /// unknown string props
    property_map<std::string> unknown_props;
};

///
/// Shape data represented as an indexed array.
/// May contain only one of the points/lines/triangles.
///
struct shape {
    /// shape name
    std::string name = "";
    /// path (used for saving in glTF)
    std::string path = "";
    /// shape material
    material* mat = nullptr;

    // shape elements -------------------------
    /// points
    std::vector<int> points;
    /// lines
    std::vector<ym::vec2i> lines;
    /// triangles
    std::vector<ym::vec3i> triangles;
    /// quads
    std::vector<ym::vec4i> quads;

    // vertex data ----------------------------
    /// per-vertex position (3 float)
    std::vector<ym::vec3f> pos;
    /// per-vertex normals (3 float)
    std::vector<ym::vec3f> norm;
    /// per-vertex texcoord (2 float)
    std::vector<ym::vec2f> texcoord;
    /// per-vertex second texcoord (2 float)
    std::vector<ym::vec2f> texcoord1;
    /// per-vertex color (4 float)
    std::vector<ym::vec4f> color;
    /// per-vertex radius (1 float)
    std::vector<float> radius;
    /// per-vertex tangent space (4 float)
    std::vector<ym::vec4f> tangsp;

    // computed data --------------------------
    /// element CDF for sampling
    std::vector<float> elem_cdf;
    /// BVH
    ym::bvh_tree* bvh = nullptr;
    /// bounding box (needs to be updated explicitly)
    ym::bbox3f bbox = ym::invalid_bbox3f;

    // clean
    ~shape() {
        if (bvh) delete bvh;
    }
};

///
/// Shape instance.
///
struct instance {
    // name
    std::string name;
    /// transform frame
    ym::frame3f frame = ym::identity_frame3f;
    /// shape instance
    shape* shp = nullptr;

    // computed data --------------------------
    /// bounding box (needs to be updated explicitly)
    ym::bbox3f bbox = ym::invalid_bbox3f;

    /// instance transform as matrix
    ym::mat4f xform() const { return ym::to_mat(frame); }
};

///
/// Scene Camera
///
struct camera {
    /// name
    std::string name;
    /// transform frame
    ym::frame3f frame = ym::identity_frame3f;
    /// ortho cam
    bool ortho = false;
    /// vertical field of view
    float yfov = 2;
    /// aspect ratio
    float aspect = 16.0f / 9.0f;
    /// focus distance
    float focus = 1;
    /// lens aperture
    float aperture = 0;
    /// near plane distance
    float near = 0.01f;
    /// far plane distance
    float far = 10000;
};

///
/// Envinonment map
///
struct environment {
    /// name
    std::string name;
    /// transform frame
    ym::frame3f frame = ym::identity_frame3f;
    /// index of material in material array
    material* mat = nullptr;
};

///
/// Light, either an instance or an environment.
/// This is only used internally to avoid looping over all objects every time.
///
struct light {
    /// instance
    instance* ist = nullptr;
    /// environment
    environment* env = nullptr;
};

///
/// Scene
///
struct scene {
    /// shape array
    std::vector<shape*> shapes;
    /// instance array
    std::vector<instance*> instances;
    /// material array
    std::vector<material*> materials;
    /// texture array
    std::vector<texture*> textures;
    /// camera array
    std::vector<camera*> cameras;
    /// environment array
    std::vector<environment*> environments;

    /// light array
    std::vector<light*> lights;

    // computed data --------------------------
    /// BVH
    ym::bvh_tree* bvh = nullptr;
    /// bounding box (needs to be updated explicitly)
    ym::bbox3f bbox = ym::invalid_bbox3f;

    /// cleanup
    ~scene();
};

///
/// Loading options
///
struct load_options {
    /// Whether to load textures
    bool load_textures = true;
    /// Skip missing files without giving and error
    bool skip_missing = true;
    /// Whether to flip the v coordinate in OBJ
    bool obj_flip_texcoord = true;
    /// Duplicate vertices if smoothing off in OBJ
    bool obj_facet_non_smooth = false;
    /// Whether to flip tr in OBJ
    bool obj_flip_tr = true;
    /// whether to preserve quads
    bool preserve_quads = false;
};

///
/// Loads a scene. For now OBJ or glTF are supported.
/// Throws an exception if an error occurs.
///
scene* load_scene(const std::string& filename, const load_options& opts = {});

///
/// Save options
///
struct save_options {
    /// Whether to save textures
    bool save_textures = true;
    /// Skip missing files without giving and error
    bool skip_missing = true;
    /// Whether to flip the v coordinate in OBJ
    bool obj_flip_texcoord = true;
    /// Whether to flip tr in OBJ
    bool obj_flip_tr = true;
    /// Whether to use separate buffers in gltf
    bool gltf_separate_buffers = false;
};

///
/// Saves a scene. For now OBJ and glTF are supported.
/// Throws an exception if an error occurs.
///
void save_scene(
    const std::string& filename, const scene* scn, const save_options& opts);

///
/// Add elements options
///
struct add_elements_options {
    /// Add missing normal
    bool smooth_normals = true;
    /// Add missing radius for points and lines (<=0 for no adding)
    float pointline_radius = 0;
    /// Add missing trangent space
    bool tangent_space = true;
    /// texture data
    bool texture_data = true;
    /// Add instances
    bool shape_instances = true;
    /// Add default camera
    bool default_camera = true;
    /// Add default names
    bool default_names = true;
    /// Add default paths
    bool default_paths = true;

    /// initialize to no element
    static add_elements_options none() {
        auto opts = add_elements_options();
        memset(&opts, 0, sizeof(opts));
        return opts;
    }
};

///
/// Add elements
///
void add_elements(scene* scn, const add_elements_options& opts = {});

///
/// Merge scene into one another. Note that the objects are _moved_ from
/// merge_from to merged_into, so merge_from will be empty after this function.
///
void merge_into(scene* merge_into, scene* merge_from);

///
/// Computes a scene bounding box
///
ym::bbox3f compute_bounds(const scene* scn);

///
/// Computes an instance bounding box
///
ym::bbox3f compute_bounds(const instance* ist);

///
/// Computes a shape bounding box
///
ym::bbox3f compute_bounds(const shape* shp);

///
/// Flatten scene instances into separate meshes.
///
void flatten_instances(scene* scn);

///
/// Initialize the lights
///
void update_lights(scene* scn, bool point_only);

///
/// Print scene information
///
void print_info(const scene* scn);

///
/// Build a shape BVH
///
void build_bvh(shape* shp, bool equalsize = true);

///
/// Build a scene BVH
///
void build_bvh(scene* scn, bool equalsize = true, bool do_shapes = true);

///
/// Refits a scene BVH
///
void refit_bvh(shape* shp);

///
/// Refits a scene BVH
///
void refit_bvh(scene* scn, bool do_shapes = true);

///
/// Intersect the shape with a ray. Find any interstion if early_exit,
/// otherwise find first intersection.
///
/// - Parameters:
///     - scn: scene to intersect
///     - ray: ray to be intersected
///     - early_exit: whether to stop at the first found hit
///     - ray_t: ray distance at intersection
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
///
bool intersect_ray(const shape* shp, const ym::ray3f& ray, bool early_exit,
    float& ray_t, int& eid, ym::vec4f& euv);

///
/// Intersect the instance with a ray. Find any interstion if early_exit,
/// otherwise find first intersection.
///
/// - Parameters:
///     - scn: scene to intersect
///     - ray: ray to be intersected
///     - early_exit: whether to stop at the first found hit
///     - ray_t: ray distance at intersection
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
///
bool intersect_ray(const instance* ist, const ym::ray3f& ray, bool early_exit,
    float& ray_t, int& eid, ym::vec4f& euv);

///
/// Intersect the scene with a ray. Find any interstion if early_exit,
/// otherwise find first intersection.
///
/// - Parameters:
///     - scn: scene to intersect
///     - ray: ray to be intersected
///     - early_exit: whether to stop at the first found hit
///     - ray_t: ray distance at intersection
///     - iid: instance index
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
///
bool intersect_ray(const scene* scn, const ym::ray3f& ray, bool early_exit,
    float& ray_t, int& iid, int& eid, ym::vec4f& euv);

///
/// Surface point.
///
struct intersection_point {
    /// distance of the hit along the ray or from the point
    float dist = 0;
    /// instance index
    int iid = -1;
    /// shape element index
    int eid = -1;
    /// shape barycentric coordinates
    ym::vec4f euv = ym::zero4f;

    /// check if intersection is valid
    operator bool() const { return eid >= 0; }
};

///
/// Intersect the scene with a ray. Find any interstion if early_exit,
/// otherwise find first intersection.
///
/// - Parameters:
///     - scn: scene to intersect
///     - ray: ray to be intersected
///     - early_exit: whether to stop at the first found hit
/// - Returns:
///     - intersection record
///
inline intersection_point intersect_ray(
    const scene* scn, const ym::ray3f& ray, bool early_exit) {
    auto isec = intersection_point();
    if (!intersect_ray(
            scn, ray, early_exit, isec.dist, isec.iid, isec.eid, isec.euv))
        return {};
    return isec;
}

///
/// Finds the closest element that overlaps a point within a given distance.
///
/// - Parameters:
///     - scn: scene to intersect
///     - pos: point position
///     - max_dist: maximu valid distance
///     - early_exit: whether to stop at the first found hit
///     - dist: distance at intersection
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
///
bool overlap_point(const shape* shp, const ym::vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& eid, ym::vec4f& euv);

///
/// Finds the closest element that overlaps a point within a given distance.
///
/// - Parameters:
///     - scn: scene to intersect
///     - pos: point position
///     - max_dist: maximu valid distance
///     - early_exit: whether to stop at the first found hit
///     - dist: distance at intersection
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
///
bool overlap_point(const instance* ist, const ym::vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& eid, ym::vec4f& euv);

///
/// Finds the closest element that overlaps a point within a given distance.
///
/// - Parameters:
///     - scn: scene to intersect
///     - pos: point position
///     - max_dist: maximu valid distance
///     - early_exit: whether to stop at the first found hit
///     - dist: distance at intersection
///     - iid: instance index
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
///
bool overlap_point(const scene* scn, const ym::vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& iid, int& eid, ym::vec4f& euv);

///
/// Find the list of overlaps between instance bounds.
///
void overlap_instance_bounds(const scene* scn1, const scene* scn2,
    bool skip_duplicates, bool skip_self, std::vector<ym::vec2i>& overlaps);

}  // namespace yscn

#endif
