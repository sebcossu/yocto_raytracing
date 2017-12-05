#ifndef _SCENE_H_
#define _SCENE_H_

#include <vector>
#include "image.h"
#include "math.h"

// LITC begin bvh_node
struct bvh_node {
    bbox3f bbox;     // bounding box
    uint32_t start;  // index to the first sorted primitive/node
    uint16_t count;  // number of primitives/nodes
    uint8_t isleaf;  // whether it is a leaf
    uint8_t axis;    // split axis
};
// LITC end bvh_node

// LITC begin bvh_tree
struct bvh_tree {
    std::vector<bvh_node> nodes;
    std::vector<int> leaf_prims;
};
// LITC end bvh_tree

// LITC begin shape
struct shape {
    std::string name;

    std::vector<vec3f> pos;       // vertex position
    std::vector<vec3f> norm;      // vertex normal
    std::vector<vec2f> texcoord;  // vertex texture coordinates
    std::vector<vec3f> col;       // vertex color
    std::vector<float> radius;    // vertex radius
    std::vector<vec4f> tangsp;    // vertex tangent space

    std::vector<int> points;        // points indices
    std::vector<vec2i> lines;       // line indices
    std::vector<vec3i> triangles;   // triangle indices
    std::vector<vec4i> tetrahedra;  // tetrahedra indices

    std::vector<vec3f> lpos;      // simulation: previous vertex position
    std::vector<vec3f> vel;       // simulation: vertex velocity
    std::vector<float> inv_mass;  // simulation: inverse of vertex mass

    bvh_tree* bvh = nullptr;  // geometry queries acceleration

    ~shape() {
        if (bvh) delete bvh;
    }
};
// LITC end shape

// LITC begin texture
struct texture {
    std::string filename;
    image4f hdr;
    image4b ldr;
};
// LITC end texture

// LITC begin material
struct material {
    std::string name;

    vec3f ke = {0, 0, 0};           // emission color
    vec3f kd = {0.5, 0.5, 0.5};     // diffuse color
    vec3f ks = {0.04, 0.04, 0.04};  // specular color
    float rs = 0;                   // specular roughness
    vec3f kr = {0, 0, 0};           // reflection color

    texture* ke_txt = nullptr;  // emission texture
    texture* kd_txt = nullptr;  // diffuse texture
    texture* ks_txt = nullptr;  // specular texture
    texture* rs_txt = nullptr;  // roughness texture
    texture* kr_txt = nullptr;  // reflection texture

    texture* norm_txt = nullptr;  // normal map
    texture* disp_txt = nullptr;  // displacement map
    float disp_height = 0.0f;     // displacement height

    bool simulated = false;      // simulation: on/off
    bool deformable = false;     // simulation: deformable or rigid object
    bool pin = false;            // simulation: pin object
    float density = 0.0f;        // simulation: density
    float inv_stiffness = 0.0f;  // simulation: inverse stiffness
};
// LITC end material

// LITC begin animation
struct animation {
    float delta_t = 1.0f / 60.0f;
    std::vector<frame3f> frame_keyframes;
    std::vector<std::vector<vec3f>> pos_keyframes;
    std::vector<std::vector<vec3f>> norm_keyframes;
};
// LITC end animation

// LITC begin instance
struct instance {
    std::string name;

    frame3f frame = identity_frame3f;  // coordinate frame
    material* mat = nullptr;           // material
    shape* shp = nullptr;              // shape

    vec3f lvel, avel;   // simulation: linear and angular velocity
    float inv_mass;     // simulation: body inverse mass
    mat3f inv_inertia;  // simulation: body inverse inertia

    animation* anim = nullptr;  // animation data
};
// LITC end instance

// LITC begin camera
struct camera {
    std::string name;

    frame3f frame = identity_frame3f;
    float fovy = 1;
    float aspect = 16.0f / 9.0f;
    float aperture = 0;
    float focus = 1;
};
// LITC end camera

// LITC begin environment
struct environment {
    std::string name;
    frame3f frame = identity_frame3f;
    vec3f ke = {0, 0, 0};       // emission color
    texture* ke_txt = nullptr;  // emission texture
};
// LITC end environment

// LITC begin scene
struct scene {
    std::vector<camera*> cameras;
    std::vector<shape*> shapes;
    std::vector<texture*> textures;
    std::vector<material*> materials;
    std::vector<instance*> instances;
    std::vector<environment*> environments;

    bvh_tree* bvh = nullptr;

    ~scene() {
        for (auto v : cameras) delete v;
        for (auto v : shapes) delete v;
        for (auto v : textures) delete v;
        for (auto v : materials) delete v;
        for (auto v : instances) delete v;
        for (auto v : environments) delete v;
        if (bvh) delete bvh;
    }
};
// LITC end scene

// LITC begin eval_pos_shape
inline vec3f eval_pos(const shape* shp, int ei, const vec4f& ew) {
    if (!shp->points.empty()) {
        return shp->pos[shp->points[ei]];
    } else if (!shp->lines.empty()) {
        return shp->pos[shp->lines[ei].x] * ew.x +
               shp->pos[shp->lines[ei].y] * ew.y;
    } else if (!shp->triangles.empty()) {
        return shp->pos[shp->triangles[ei].x] * ew.x +
               shp->pos[shp->triangles[ei].y] * ew.y +
               shp->pos[shp->triangles[ei].z] * ew.z;
    } else {
        return {};
    }
}
// LITC end eval_pos_shape

// LITC begin eval_norm_shape
inline vec3f eval_norm(const shape* shp, int ei, const vec4f& ew) {
    if (!shp->points.empty()) {
        return shp->norm[shp->points[ei]];
    } else if (!shp->lines.empty()) {
        return normalize(shp->norm[shp->lines[ei].x] * ew.x +
                         shp->norm[shp->lines[ei].y] * ew.y);
    } else if (!shp->triangles.empty()) {
        return normalize(shp->norm[shp->triangles[ei].x] * ew.x +
                         shp->norm[shp->triangles[ei].y] * ew.y +
                         shp->norm[shp->triangles[ei].z] * ew.z);
    } else {
        return {};
    }
}
// LITC end eval_norm_shape

// LITC begin eval_texcoord_shape
inline vec2f eval_texcoord(const shape* shp, int ei, const vec4f& ew) {
    if (!shp->points.empty()) {
        return shp->texcoord[shp->points[ei]];
    } else if (!shp->lines.empty()) {
        return shp->texcoord[shp->lines[ei].x] * ew.x +
               shp->texcoord[shp->lines[ei].y] * ew.y;
    } else if (!shp->triangles.empty()) {
        return shp->texcoord[shp->triangles[ei].x] * ew.x +
               shp->texcoord[shp->triangles[ei].y] * ew.y +
               shp->texcoord[shp->triangles[ei].z] * ew.z;
    } else {
        return {};
    }
}
// LITC end eval_texcoord_shape

// LITC begin eval_pos_instance
inline vec3f eval_pos(const instance* ist, int ei, const vec4f& ew) {
    return transform_point(ist->frame, eval_pos(ist->shp, ei, ew));
}
// LITC end eval_pos_instance

// LITC begin eval_norm_instance
inline vec3f eval_norm(const instance* ist, int ei, const vec4f& ew) {
    return transform_direction(ist->frame, eval_norm(ist->shp, ei, ew));
}
// LITC end eval_norm_instance

// LITC begin smooth_normals_interface
void compute_smooth_normals(shape* shp);
// LITC end smooth_normals_interface

scene* load_scene(const std::string& filename);

struct intersection3f {
    instance* ist = nullptr;
    int ei = -1;
    vec4f ew = {0, 0, 0, 0};
    float dist = 0;

    bool hit() const { return ei >= 0; }
};

intersection3f intersect_first(const scene* scn, const ray3f& ray);
bool intersect_any(const scene* scn, const ray3f& ray);
void build_bvh(scene* scn, bool equal_num);

#endif
