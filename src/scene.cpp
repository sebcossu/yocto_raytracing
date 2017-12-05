
#include "scene.h"

#define YSCN_NO_IMAGE
#include "ext/yocto_scn.h"

#include <map>
#include <set>

// LITC begin smooth_normals
void compute_smooth_normals(shape* shp) {
    shp->norm.resize(shp->pos.size());
    for (auto& norm : shp->norm) norm = {0, 0, 0};

    for (auto l : shp->lines) {
        auto n = line_tangent(shp->pos[l.x], shp->pos[l.y]);
        auto w = line_length(shp->pos[l.x], shp->pos[l.y]);
        shp->norm[l.x] += n * w;
        shp->norm[l.y] += n * w;
    }

    for (auto t : shp->triangles) {
        auto n = triangle_normal(shp->pos[t.x], shp->pos[t.y], shp->pos[t.z]);
        auto w = triangle_area(shp->pos[t.x], shp->pos[t.y], shp->pos[t.z]);
        shp->norm[t.x] += n * w;
        shp->norm[t.y] += n * w;
        shp->norm[t.z] += n * w;
    }

    for (auto& norm : shp->norm) norm = normalize(norm);
}
// LITC end smooth_normals

// LITC begin animfuncs
void update_animation(instance* ist, float time) {
    auto anim = ist->anim;
    if (!anim) return;
    auto idx = int(time / anim->delta_t) % anim->frame_keyframes.size();
    ist->frame = anim->frame_keyframes[idx];
    if (!anim->pos_keyframes.empty()) ist->shp->pos = anim->pos_keyframes[idx];
    if (!anim->norm_keyframes.empty())
        ist->shp->norm = anim->norm_keyframes[idx];
}

void add_keyframe(instance* ist) {
    ist->anim->frame_keyframes.push_back(ist->frame);
    ist->anim->pos_keyframes.push_back(ist->shp->pos);
    ist->anim->norm_keyframes.push_back(ist->shp->norm);
}
// LITC end animfuncs

// triangle tangent and bitangent from uv (not othornormalized with themselfves
// not the normal)
// see http://www.terathon.com/code/tangent.html and
// https://gist.github.com/aras-p/2843984
// LITC begin compute_tangent_space
std::pair<vec3f, vec3f> triangle_tangents_fromuv(const vec3f& v0,
    const vec3f& v1, const vec3f& v2, const vec2f& uv0, const vec2f& uv1,
    const vec2f& uv2) {
    // normal points up from texture space
    auto p = v1 - v0;
    auto q = v2 - v0;
    auto s = vec2f{uv1.x - uv0.x, uv2.x - uv0.x};
    auto t = vec2f{uv1.y - uv0.y, uv2.y - uv0.y};
    auto div = s.x * t.y - s.y * t.x;

    if (div > 0) {
        auto tu = vec3f{t.y * p.x - t.x * q.x, t.y * p.y - t.x * q.y,
                      t.y * p.z - t.x * q.z} /
                  div;
        auto tv = vec3f{s.x * q.x - s.y * p.x, s.x * q.y - s.y * p.y,
                      s.x * q.z - s.y * p.z} /
                  div;
        return {tu, tv};
    } else {
        return {{1, 0, 0}, {0, 1, 0}};
    }
}

void compute_tangent_space(shape* shp) {
    auto tangu = std::vector<vec3f>(shp->pos.size(), {0, 0, 0});
    auto tangv = std::vector<vec3f>(shp->pos.size(), {0, 0, 0});

    for (auto t : shp->triangles) {
        auto tutv = triangle_tangents_fromuv(shp->pos[t.x], shp->pos[t.y],
            shp->pos[t.z], shp->texcoord[t.x], shp->texcoord[t.y],
            shp->texcoord[t.z]);
        auto w = triangle_area(shp->pos[t.x], shp->pos[t.y], shp->pos[t.z]);
        tangu[t.x] += tutv.first * w;
        tangu[t.y] += tutv.first * w;
        tangu[t.z] += tutv.first * w;
        tangv[t.x] += tutv.second * w;
        tangv[t.y] += tutv.second * w;
        tangv[t.z] += tutv.second * w;
    }

    shp->tangsp.resize(shp->pos.size());
    for (auto i = 0; i < tangu.size(); i++) {
        tangu[i] = orthonormalize(tangu[i], shp->norm[i]);
        auto s =
            (dot(cross(shp->norm[i], tangu[i]), tangv[i]) < 0) ? -1.0f : 1.0f;
        shp->tangsp[i] = {tangu[i].x, tangu[i].y, tangu[i].z, s};
    }
}
// LITC end compute_tangent_space

// LITC begin load
inline frame3f to_frame(const ym::frame3f& m) {
    return {{m.x.x, m.x.y, m.x.z}, {m.y.x, m.y.y, m.y.z}, {m.z.x, m.z.y, m.z.z},
        {m.o.x, m.o.y, m.o.z}};
}

scene* load_scene(const std::string& filename) {
    auto yscn_ = (yscn::scene*)nullptr;
    try {
        auto load_opts = yscn::load_options();
        load_opts.load_textures = false;
        yscn_ = yscn::load_scene(filename, load_opts);
    } catch (std::exception& e) {
        printf("could not load scene\n");
        exit(1);
    }

    auto add_opts = yscn::add_elements_options::none();
    add_opts.shape_instances = true;
    add_opts.default_names = true;
    add_opts.default_camera = true;
    add_opts.pointline_radius = 0.001f;
    add_opts.texture_data = false;
    yscn::add_elements(yscn_, add_opts);

    auto scn = new scene();

    for (auto ycam : yscn_->cameras) {
        auto cam = new camera();
        cam->name = ycam->name;
        cam->frame = to_frame(ycam->frame);
        cam->fovy = ycam->yfov;
        cam->aspect = ycam->aspect;
        cam->focus = ycam->focus;
        cam->aperture = ycam->aperture;
        scn->cameras.push_back(cam);
    }

    auto dirname = std::string();
    if (filename.rfind("/") != filename.npos) {
        dirname = filename.substr(0, filename.rfind("/") + 1);
    }
    auto textures = std::map<yscn::texture*, texture*>{{nullptr, nullptr}};
    for (auto ytxt : yscn_->textures) {
        auto txt = new texture();
        txt->filename = ytxt->path;
        if (txt->filename.substr(txt->filename.length() - 4) == ".hdr") {
            txt->hdr = load_image4f(dirname + ytxt->path);
        } else {
            txt->ldr = load_image4b(dirname + ytxt->path);
        }
        scn->textures.push_back(txt);
        textures[ytxt] = txt;
    }

    auto materials = std::map<yscn::material*, material*>();
    for (auto ymat : yscn_->materials) {
        auto mat = new material();
        mat->name = ymat->name;
        mat->ke = {ymat->ke[0], ymat->ke[1], ymat->ke[2]};
        mat->kd = {ymat->kd[0], ymat->kd[1], ymat->kd[2]};
        mat->ks = {ymat->ks[0], ymat->ks[1], ymat->ks[2]};
        mat->rs = ymat->rs;
        mat->kr = {ymat->kr[0], ymat->kr[1], ymat->kr[2]};
        mat->ke_txt = textures.at(ymat->ke_txt.txt);
        mat->kd_txt = textures.at(ymat->kd_txt.txt);
        mat->ks_txt = textures.at(ymat->ks_txt.txt);
        mat->kr_txt = textures.at(ymat->kr_txt.txt);
        mat->norm_txt = textures.at(ymat->norm_txt.txt);
        mat->disp_txt = textures.at(ymat->disp_txt.txt);
        mat->disp_height = 0.01f;
        scn->materials.push_back(mat);
        materials[ymat] = mat;
    }

    auto shapes = std::map<yscn::shape*, shape*>();
    for (auto yshp : yscn_->shapes) {
        auto shp = new shape();
        shp->name = yshp->name;
        shp->pos = *(std::vector<vec3f>*)&yshp->pos;
        shp->norm = *(std::vector<vec3f>*)&yshp->norm;
        shp->texcoord = *(std::vector<vec2f>*)&yshp->texcoord;
        shp->radius = yshp->radius;
        shp->points = yshp->points;
        shp->lines = *(std::vector<vec2i>*)&yshp->lines;
        shp->triangles = *(std::vector<vec3i>*)&yshp->triangles;
        scn->shapes.push_back(shp);
        shapes[yshp] = shp;
    }

    for (auto yist : yscn_->instances) {
        auto yshp = yist->shp;
        auto ist = new instance();
        ist->name = yist->name;
        ist->frame = to_frame(yist->frame);
        ist->mat = materials[yshp->mat];
        ist->shp = shapes[yshp];
        scn->instances.push_back(ist);
    }

    for (auto yenv : yscn_->environments) {
        auto ymat = yenv->mat;
        auto env = new environment();
        env->name = yenv->name;
        env->frame = to_frame(yenv->frame);
        env->ke = {ymat->ke[0], ymat->ke[1], ymat->ke[2]};
        env->ke_txt = textures.at(ymat->ke_txt.txt);
        scn->environments.push_back(env);
    }

    for (auto ist : scn->instances) {
        auto shp = ist->shp;
        if (shp->norm.empty()) compute_smooth_normals(shp);
        if (shp->tangsp.empty() && ist->mat->norm_txt)
            compute_tangent_space(shp);
    }

    return scn;
}
// LITC end load

// LITC begin intersect_triangle
bool intersect_triangle(const ray3f& ray, const vec3f& v0, const vec3f& v1,
    const vec3f& v2, float& dist, vec4f& ew) {
    // compute triangle edges
    auto e1 = v1 - v0;
    auto e2 = v2 - v0;

    // compute denominator
    auto r = cross(ray.d, e2);
    auto den = dot(r, e1);

    // check denominator and exit if triangle and ray are parallel
    // (could use EPSILONS if desired)
    if (den == 0) return false;
    float inv_den = 1.0f / den;

    // compute and check first barycentric coordinate
    auto c = ray.o - v0;
    auto w1 = dot(r, c) * inv_den;
    if (w1 < 0 || w1 > 1) return false;

    // compute and check second barycentric coordinate
    auto s = cross(c, e1);
    auto w2 = dot(s, ray.d) * inv_den;
    if (w2 < 0.0 || w1 + w2 > 1.0) return false;

    // compute and check ray parameter
    auto t = dot(s, e2) * inv_den;
    if (t < ray.tmin || t > ray.tmax) return false;

    // intersection occurred: set params and exit
    dist = t;
    ew = {1 - w1 - w2, w1, w2, 0};

    return true;
}
// LITC end intersect_triangle

// LITC begin intersect_point
inline bool intersect_point(
    const ray3f& ray, const vec3f& p, float r, float& dist, vec4f& ew) {
    auto w = p - ray.o;
    auto t = dot(w, ray.d) / dot(ray.d, ray.d);
    if (t < ray.tmin || t > ray.tmax) return false;

    auto rp = eval_ray(ray, t);
    auto prp = p - rp;
    if (dot(prp, prp) > r * r) return false;

    dist = t;
    ew = {1, 0, 0, 0};

    return true;
}
// LITC end intersect_point

// LITC begin intersect_line
inline bool intersect_line(const ray3f& ray, const vec3f& v0, const vec3f& v1,
    float r0, float r1, float& dist, vec4f& ew) {
    auto u = ray.d, v = v1 - v0, w = ray.o - v0;

    auto a = dot(u, u), b = dot(u, v), c = dot(v, v), d = dot(u, w),
         e = dot(v, w);
    auto det = a * c - b * b;
    if (det == 0) return false;

    auto t = (b * e - c * d) / det, s = (a * e - b * d) / det;
    if (t < ray.tmin || t > ray.tmax) return false;

    s = clamp(s, (float)0, (float)1);
    auto p0 = eval_ray(ray, t), p1 = eval_ray(ray3f{v0, v1 - v0}, s);
    auto p01 = p0 - p1;
    auto r = r0 * (1 - s) + r1 * s;
    if (dot(p01, p01) > r * r) return false;

    dist = t;
    ew = {1 - s, s, 0, 0};

    return true;
}
// LITC end intersect_line

// LITC begin intersect_ray_shape
bool intersect_ray(
    const shape* shp, const ray3f& ray, float& dist, int& iei, vec4f& iew) {
    auto hit = false;
    auto tray = ray;
    for (auto ei = 0; ei < shp->triangles.size(); ei++) {
        auto e = shp->triangles[ei];
        if (!intersect_triangle(
                tray, shp->pos[e.x], shp->pos[e.y], shp->pos[e.z], dist, iew))
            continue;
        iei = ei;
        tray.tmax = dist;
        hit = true;
    }
    for (auto ei = 0; ei < shp->lines.size(); ei++) {
        auto e = shp->lines[ei];
        if (!intersect_line(tray, shp->pos[e.x], shp->pos[e.y],
                shp->radius[e.x], shp->radius[e.y], dist, iew))
            continue;
        iei = ei;
        tray.tmax = dist;
        hit = true;
    }
    for (auto ei = 0; ei < shp->points.size(); ei++) {
        auto e = shp->points[ei];
        if (!intersect_point(tray, shp->pos[e], shp->radius[e], dist, iew))
            continue;
        iei = ei;
        tray.tmax = dist;
        hit = true;
    }
    return hit;
}
// LITC end intersect_ray_shape

// LITC begin intersect_ray_scene
bool intersect_ray(const scene* scn, const ray3f& ray, float& dist,
    instance*& iist, int& iei, vec4f& iew) {
    auto hit = false;
    auto tray = ray;
    for (auto ist : scn->instances) {
        auto lray = transform_ray_inverse(ist->frame, tray);
        if (!intersect_ray(ist->shp, lray, dist, iei, iew)) continue;
        tray.tmax = dist;
        iist = ist;
        hit = true;
    }
    return hit;
}
// LITC end intersect_ray_scene

// LITC begin intersect_ray_easy
intersection3f intersect_scene(const scene* scn, const ray3f& ray) {
    intersection3f isec;
    if (!intersect_ray(scn, ray, isec.dist, isec.ist, isec.ei, isec.ew))
        return {};
    return isec;
}
// LITC end intersect_ray_easy

// LITC begin intersect_bbox
inline bool intersect_check_bbox(const ray3f& ray, const bbox3f& bbox) {
    auto invd = vec3f{1.0f / ray.d.x, 1.0f / ray.d.y, 1.0f / ray.d.z};
    auto t0 = (bbox.min - ray.o) * invd;
    auto t1 = (bbox.max - ray.o) * invd;
    if (invd.x < 0) std::swap(t0.x, t1.x);
    if (invd.y < 0) std::swap(t0.y, t1.y);
    if (invd.z < 0) std::swap(t0.z, t1.z);
    auto tmin = max(t0.z, max(t0.y, max(t0.x, ray.tmin)));
    auto tmax = min(t1.z, min(t1.y, min(t1.x, ray.tmax)));
    tmax *= 1.00000024f;
    return tmin <= tmax;
}
// LITC end intersect_bbox

// LITC begin intersect_bvh_shape
bool intersect_bvh(const shape* shp, const ray3f& ray, bool any, float& dist,
    int& ei, vec4f& ew) {
    int node_stack[64];
    auto node_cur = 0;
    node_stack[node_cur++] = 0;

    auto bvh = shp->bvh;

    auto tray = ray;
    auto hit = false;

    while (node_cur) {
        auto node = bvh->nodes[node_stack[--node_cur]];
        if (!intersect_check_bbox(tray, node.bbox)) continue;

        if (!node.isleaf) {
            for (auto i = node.start; i < node.start + node.count; i++) {
                node_stack[node_cur++] = i;
            }
        } else if (!shp->triangles.empty()) {
            for (auto i = node.start; i < node.start + node.count; i++) {
                auto e = shp->triangles[bvh->leaf_prims[i]];
                if (!intersect_triangle(tray, shp->pos[e.x], shp->pos[e.y],
                        shp->pos[e.z], dist, ew))
                    continue;
                hit = true;
                tray.tmax = dist;
                ei = bvh->leaf_prims[i];
                if (any) return true;
            }
        } else if (!shp->lines.empty()) {
            for (auto i = node.start; i < node.start + node.count; i++) {
                auto e = shp->lines[bvh->leaf_prims[i]];
                if (!intersect_line(tray, shp->pos[e.x], shp->pos[e.y],
                        shp->radius[e.x], shp->radius[e.y], dist, ew))
                    continue;
                hit = true;
                tray.tmax = dist;
                ei = bvh->leaf_prims[i];
                if (any) return true;
            }
        } else if (!shp->points.empty()) {
            for (auto i = node.start; i < node.start + node.count; i++) {
                auto e = shp->points[bvh->leaf_prims[i]];
                if (!intersect_point(
                        tray, shp->pos[e], shp->radius[e], dist, ew))
                    continue;
                hit = true;
                tray.tmax = dist;
                ei = bvh->leaf_prims[i];
                if (any) return true;
            }
        }
    }

    return hit;
}
// LITC end intersect_bvh_shape

// LITC begin intersect_bvh_scene
bool intersect_bvh(const scene* scn, const ray3f& ray, bool any, float& dist,
    instance*& ist, int& ei, vec4f& ew) {
    int node_stack[64];
    auto node_cur = 0;
    node_stack[node_cur++] = 0;

    auto bvh = scn->bvh;

    auto tray = ray;
    auto hit = false;

    while (node_cur) {
        auto node = bvh->nodes[node_stack[--node_cur]];
        if (!intersect_check_bbox(tray, node.bbox)) continue;

        if (!node.isleaf) {
            for (auto i = node.start; i < node.start + node.count; i++) {
                node_stack[node_cur++] = i;
            }
        } else {
            for (auto i = node.start; i < node.start + node.count; i++) {
                auto is = scn->instances[bvh->leaf_prims[i]];
                auto lray = transform_ray_inverse(is->frame, tray);
                if (!intersect_bvh(is->shp, lray, any, dist, ei, ew)) continue;
                tray.tmax = dist;
                ist = is;
                hit = true;
                if (any) return hit;
            }
        }
    }

    return hit;
}
// LITC end intersect_bvh_scene

// LITC begin intersect_bvh_easy
intersection3f intersect_first(const scene* scn, const ray3f& ray) {
    intersection3f isec;
    if (!intersect_bvh(scn, ray, false, isec.dist, isec.ist, isec.ei, isec.ew))
        return {};
    return isec;
}
bool intersect_any(const scene* scn, const ray3f& ray) {
    intersection3f isec;
    return (bool)intersect_bvh(
        scn, ray, true, isec.dist, isec.ist, isec.ei, isec.ew);
}
// LITC end intersect_bvh_easy

// LITC begin intersect_bvh_other_easy
intersection3f intersect_first(const shape* shp, const ray3f& ray) {
    intersection3f isec;
    if (!intersect_bvh(shp, ray, false, isec.dist, isec.ei, isec.ew)) return {};
    return isec;
}
bool intersect_any(const shape* shp, const ray3f& ray) {
    intersection3f isec;
    return (bool)intersect_bvh(shp, ray, true, isec.dist, isec.ei, isec.ew);
}
// LITC end intersect_bvh_other_easy

// LITC begin bound_prim
struct bound_prim {
    bbox3f bbox;   // bounding box
    vec3f center;  // center
    int pid;       // primitive id
};
// LITC end bound_prim

// LITC begin build_bvh_decl
bvh_tree* build_bvh(std::vector<bound_prim>& bound_prims, bool equal_num);
// LITC end build_bvh_decl

// LITC begin build_bvh_shape
bbox3f expand_bbox(const bbox3f& bbox, const vec3f& p, float r) {
    return expand_bbox(bbox, {p - vec3f{r, r, r}, p + vec3f{r, r, r}});
}

void build_bvh(shape* shp, bool equal_num) {
    auto bound_prims = std::vector<bound_prim>();
    for (auto ei = 0; ei < shp->points.size(); ei++) {
        auto e = shp->points[ei];
        auto bbox = invalid_bbox3f;
        bbox = expand_bbox(bbox, shp->pos[e], shp->radius[e]);
        bound_prims.push_back({bbox, (bbox.min + bbox.max) / 2.0f, ei});
    }
    for (auto ei = 0; ei < shp->lines.size(); ei++) {
        auto e = shp->lines[ei];
        auto bbox = invalid_bbox3f;
        bbox = expand_bbox(bbox, shp->pos[e.x], shp->radius[e.x]);
        bbox = expand_bbox(bbox, shp->pos[e.y], shp->radius[e.y]);
        bound_prims.push_back({bbox, (bbox.min + bbox.max) / 2.0f, ei});
    }
    for (auto ei = 0; ei < shp->triangles.size(); ei++) {
        auto e = shp->triangles[ei];
        auto bbox = invalid_bbox3f;
        bbox = expand_bbox(bbox, shp->pos[e.x], 0);
        bbox = expand_bbox(bbox, shp->pos[e.y], 0);
        bbox = expand_bbox(bbox, shp->pos[e.z], 0);
        bound_prims.push_back({bbox, (bbox.min + bbox.max) / 2.0f, ei});
    }

    shp->bvh = build_bvh(bound_prims, equal_num);
}
// LITC end build_bvh_shape

// LITC begin build_bvh_scene
void build_bvh(scene* scn, bool equal_num) {
    for (auto shp : scn->shapes) { build_bvh(shp, equal_num); }

    auto bound_prims = std::vector<bound_prim>();
    for (auto ii = 0; ii < scn->instances.size(); ii++) {
        auto ist = scn->instances[ii];
        auto bbox = bbox_to_world(ist->frame, ist->shp->bvh->nodes[0].bbox);
        bound_prims.push_back({bbox, (bbox.min + bbox.max) / 2.0f, ii});
    }

    scn->bvh = build_bvh(bound_prims, equal_num);
}
// LITC end build_bvh_scene

// LITC begin make_node
bool split_prims(std::vector<bound_prim>& sorted_prim, int start, int end,
    bool equalnum, int& axis, int& mid);

void make_node(bvh_tree* bvh, int nid, std::vector<bound_prim>& leaf_prims,
    int start, int end, bool equal_num) {
    auto node = &bvh->nodes[nid];

    node->bbox = invalid_bbox3f;
    for (auto i = start; i < end; i++) {
        node->bbox = expand_bbox(node->bbox, leaf_prims[i].bbox);
    }

    auto split = false;
    auto axis = -1, mid = -1;
    if (end - start > 4) {
        split = split_prims(leaf_prims, start, end, equal_num, axis, mid);
    }

    if (!split) {
        node->isleaf = true;
        node->start = start;
        node->count = end - start;
    } else {
        assert(mid > start && mid < end);
        node->isleaf = false;
        node->axis = axis;
        auto first = (int)bvh->nodes.size();
        node->start = first;
        node->count = 2;
        bvh->nodes.push_back({});
        bvh->nodes.push_back({});
        make_node(bvh, first, leaf_prims, start, mid, equal_num);
        make_node(bvh, first + 1, leaf_prims, mid, end, equal_num);
    }
}
// LITC end make_node

// LITC begin split_prims
bool split_prims(std::vector<bound_prim>& sorted_prim, int start, int end,
    bool equalnum, int& axis, int& mid) {
    auto centroid_bbox = invalid_bbox3f;
    for (auto i = start; i < end; i++)
        centroid_bbox = expand_bbox(centroid_bbox, sorted_prim[i].center);
    auto size = centroid_bbox.max - centroid_bbox.min;

    if (size == vec3f{0, 0, 0}) return false;

    if (size.x >= size.y && size.x >= size.z)
        axis = 0;
    else if (size.y >= size.x && size.y >= size.z)
        axis = 1;
    else
        axis = 2;

    if (equalnum) {
        mid = (start + end) / 2;
        std::nth_element(sorted_prim.begin() + start, sorted_prim.begin() + mid,
            sorted_prim.begin() + end, [axis](auto a, auto b) {
                return (&a.center.x)[axis] < (&b.center.x)[axis];
            });
    } else {
        auto half = (centroid_bbox.min + centroid_bbox.max) / 2;
        mid = (int)(std::partition(sorted_prim.begin() + start,
                        sorted_prim.begin() + end,
                        [axis, half](auto a) {
                            return (&a.center.x)[axis] < (&half.x)[axis];
                        }) -
                    sorted_prim.begin());
    }
    return true;
}
// LITC end split_prims

// LITC begin build_bvh
bvh_tree* build_bvh(std::vector<bound_prim>& bound_prims, bool equal_num) {
    auto bvh = new bvh_tree();

    bvh->nodes.reserve(bound_prims.size() * 2);
    bvh->nodes.push_back({});
    make_node(bvh, 0, bound_prims, 0, bound_prims.size(), equal_num);
    bvh->nodes.shrink_to_fit();

    bvh->leaf_prims.resize(bound_prims.size());
    for (int i = 0; i < bound_prims.size(); i++) {
        bvh->leaf_prims[i] = bound_prims[i].pid;
    }

    return bvh;
}
// LITC end build_bvh
