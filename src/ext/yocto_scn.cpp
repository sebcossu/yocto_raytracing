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

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR YOCTO_SCENE
// -----------------------------------------------------------------------------

#include "yocto_scn.h"

#ifndef YSCN_NO_IMAGE
#include "yocto_img.h"
#else
#define YOBJ_NO_IMAGE
#define YGLTF_NO_IMAGE
#endif

#ifndef YSCN_NO_OBJ
#include "yocto_obj.h"
#endif

#ifndef YSCN_NO_GLTF
#include "yocto_gltf.h"
#endif

namespace yscn {
//
// Cleanup memory
//
scene::~scene() {
    for (auto v : shapes)
        if (v) delete v;
    for (auto v : instances)
        if (v) delete v;
    for (auto v : materials)
        if (v) delete v;
    for (auto v : textures)
        if (v) delete v;
    for (auto v : cameras)
        if (v) delete v;
    for (auto v : environments)
        if (v) delete v;
    for (auto light : lights)
        if (light) delete light;
    if (bvh) delete bvh;
}

//
// Get extension (including '.').
//
inline std::string get_extension(const std::string& filename) {
    auto pos = filename.rfind('.');
    if (pos == std::string::npos) return "";
    return filename.substr(pos);
}

//
// Get directory name (including '/').
//
inline std::string get_dirname(const std::string& filename) {
    auto pos = filename.rfind('/');
    if (pos == std::string::npos) pos = filename.rfind('\\');
    if (pos == std::string::npos) return "";
    return filename.substr(0, pos + 1);
}

//
// Get file basename.
//
inline std::string get_basename(const std::string& filename) {
    auto dirname = get_dirname(filename);
    auto extension = get_extension(filename);
    return filename.substr(
        dirname.size(), filename.size() - dirname.size() - extension.size());
}

//
// Check if a string starts with a prefix
//
static inline bool startsiwith(
    const std::string& str, const std::string& prefix) {
    if (str.length() < prefix.length()) return false;
    return str.substr(0, prefix.length()) == prefix;
}

//
// Loads textures for an scene.
//
void load_obj_textures(
    scene* scn, const std::string& dirname, const load_options& opts) {
#ifndef YSCN_NO_IMAGE
    for (auto txt : scn->textures) {
        auto filename = dirname + txt->path;
        for (auto& c : filename)
            if (c == '\\') c = '/';
        if (yimg::is_hdr_filename(filename)) {
            txt->hdr = yimg::load_image4f(filename);
        } else {
            txt->ldr = yimg::load_image4b(filename);
        }
        if (!txt->hdr && !txt->ldr && !opts.skip_missing)
            throw std::runtime_error("cannot load image " + filename);
    }
#endif
}

//
// Loads textures for an scene.
//
bool save_obj_textures(
    const scene* scn, const std::string& dirname, const save_options& opts) {
#ifndef YSCN_NO_IMAGE
    for (auto txt : scn->textures) {
        if (!txt->ldr && !txt->hdr) continue;
        auto filename = dirname + txt->path;
        for (auto& c : filename)
            if (c == '\\') c = '/';
        auto ok = true;
        if (txt->ldr) { ok = yimg::save_image4b(filename, txt->ldr); }
        if (txt->hdr) { ok = yimg::save_image4f(filename, txt->hdr); }
        if (!ok && !opts.skip_missing)
            throw new std::runtime_error("cannot save image " + filename);
    }
#endif
    return true;
}

//
// Flattens an scene
//
scene* obj_to_scene(const yobj::scene* obj, const load_options& opts) {
    // clear scene
    auto scn = new scene();

    struct obj_vertex_hash {
        std::hash<int> Th;
        size_t operator()(const yobj::vertex& vv) const {
            auto v = (const int*)&vv;
            size_t h = 0;
            for (auto i = 0; i < sizeof(yobj::vertex) / sizeof(int); i++) {
                // embads hash_combine below
                h ^= (Th(v[i]) + 0x9e3779b9 + (h << 6) + (h >> 2));
            }
            return h;
        }
    };

    // convert textures
    auto tmap = std::unordered_map<std::string, texture*>{{"", nullptr}};
    for (auto& otxt : obj->textures) {
        auto txt = new texture();
        txt->name = otxt.path;
        txt->path = otxt.path;
        if (!otxt.datab.empty()) {
            txt->ldr = ym::image4b(otxt.width, otxt.height);
            for (auto j = 0; j < otxt.height; j++) {
                for (auto i = 0; i < otxt.width; i++) {
                    auto v =
                        otxt.datab.data() + (otxt.width * j + i) * otxt.ncomp;
                    switch (otxt.ncomp) {
                        case 1:
                            txt->ldr.at(i, j) = {v[0], v[0], v[0], 255};
                            break;
                        case 2: txt->ldr.at(i, j) = {v[0], v[1], 0, 255}; break;
                        case 3:
                            txt->ldr.at(i, j) = {v[0], v[1], v[2], 255};
                            break;
                        case 4:
                            txt->ldr.at(i, j) = {v[0], v[1], v[2], v[3]};
                            break;
                        default: assert(false); break;
                    }
                }
            }
        } else if (!otxt.dataf.empty()) {
            txt->hdr = ym::image4f(otxt.width, otxt.height);
            for (auto j = 0; j < otxt.height; j++) {
                for (auto i = 0; i < otxt.width; i++) {
                    auto v =
                        otxt.dataf.data() + (otxt.width * j + i) * otxt.ncomp;
                    switch (otxt.ncomp) {
                        case 1:
                            txt->hdr.at(i, j) = {v[0], v[0], v[0], 1};
                            break;
                        case 2: txt->hdr.at(i, j) = {v[0], v[1], 0, 1}; break;
                        case 3:
                            txt->hdr.at(i, j) = {v[0], v[1], v[2], 1};
                            break;
                        case 4:
                            txt->hdr.at(i, j) = {v[0], v[1], v[2], v[3]};
                            break;
                        default: assert(false); break;
                    }
                }
            }
        }
        scn->textures.push_back(txt);
        tmap[txt->path] = txt;
    }

    auto add_texture = [&tmap](const std::string& path,
                           const yobj::property_map<std::string>& props) {
        auto info = texture_info();
        if (path == "") return info;
        info.txt = tmap.at(path);
        if (props.empty()) return info;
        if (props.find("-clamp") != props.end() &&
            !props.at("-clamp").empty()) {
            auto clamp =
                props.at("-clamp")[0] == "on" || props.at("-clamp")[0] == "1";
            auto wrap = (clamp) ? texture_wrap::clamp : texture_wrap::repeat;
            info.wrap_s = wrap;
            info.wrap_t = wrap;
        }
        if (props.find("-bm") != props.end() && !props.at("-bm").empty()) {
            info.scale = std::atof(props.at("-bm")[0].c_str());
        }
        info.unknown_props = props;
        return info;
    };

    // convert materials and build textures
    auto mmap = std::unordered_map<std::string, material*>{{"", nullptr}};
    for (auto& omat : obj->materials) {
        auto mat = new material();
        mat->name = omat.name;
        mat->mtype = material_type::specular_roughness;
        mat->ke = omat.ke;
        mat->kd = omat.kd;
        mat->ks = omat.ks;
        mat->kr = omat.kr;
        mat->kt = omat.kt;
        mat->rs = ym::pow(2 / (omat.ns + 2), 1 / 4.0f);
        mat->op = omat.op;
        mat->ke_txt = add_texture(omat.ke_txt, omat.ke_txt_info);
        mat->kd_txt = add_texture(omat.kd_txt, omat.kd_txt_info);
        mat->ks_txt = add_texture(omat.ks_txt, omat.ks_txt_info);
        mat->kr_txt = add_texture(omat.kr_txt, omat.kr_txt_info);
        mat->kt_txt = add_texture(omat.kt_txt, omat.kt_txt_info);
        mat->rs_txt = add_texture(omat.ns_txt, omat.ns_txt_info);
        mat->norm_txt = add_texture(omat.norm_txt, omat.norm_txt_info);
        mat->bump_txt = add_texture(omat.bump_txt, omat.bump_txt_info);
        mat->disp_txt = add_texture(omat.disp_txt, omat.disp_txt_info);
        mat->unknown_props = omat.unknown_props;
        switch (omat.illum) {
            case 0:  // Color on and Ambient off
            case 1:  // Color on and Ambient on
            case 2:  // Highlight on
            case 3:  // Reflection on and Ray trace on
                mat->op = 1;
                mat->kt = {0, 0, 0};
                break;
            case 4:  // Transparency: Glass on
                // Reflection: Ray trace on
                break;
            case 5:  // Reflection: Fresnel on and Ray trace on
                mat->op = 1;
                mat->kt = {0, 0, 0};
                break;
            case 6:  // Transparency: Refraction on
                     // Reflection: Fresnel off and Ray trace on
            case 7:  // Transparency: Refraction on
                // Reflection: Fresnel on and Ray trace on
                break;
            case 8:  // Reflection on and Ray trace off
                mat->op = 1;
                mat->kt = {0, 0, 0};
                break;
            case 9:  // Transparency: Glass on
                // Reflection: Ray trace off
                break;
        }
        scn->materials.push_back(mat);
        mmap[mat->name] = mat;
    }

    // convert meshes
    auto omap = std::unordered_map<std::string, std::vector<shape*>>{{"", {}}};
    for (auto& omsh : obj->objects) {
        omap[omsh.name] = {};
        for (auto& oshp : omsh.groups) {
            if (oshp.verts.empty()) continue;
            if (oshp.elems.empty()) continue;

            auto shp = new shape();
            shp->name = omsh.name + oshp.groupname;
            shp->mat = mmap[oshp.matname];

            // insert all vertices
            std::unordered_map<yobj::vertex, int, obj_vertex_hash> vert_map;
            std::vector<int> vert_ids;
            // vert_map.clear();
            // vert_ids.clear();
            for (auto& vert : oshp.verts) {
                if (vert_map.find(vert) == vert_map.end()) {
                    vert_map[vert] = (int)vert_map.size();
                }
                vert_ids.push_back(vert_map.at(vert));
            }

            // check whether to preserve quads
            auto as_quads = false;
            if (opts.preserve_quads) {
                auto m = 10000, M = -1;
                for (auto& elem : oshp.elems) {
                    if (elem.type != yobj::element_type::face)
                        m = 2;
                    else {
                        m = std::min(m, (int)elem.size);
                        M = std::max(M, (int)elem.size);
                    }
                }
                if (m >= 3 && M == 4) as_quads = true;
            }

            // convert elements
            for (auto& elem : oshp.elems) {
                switch (elem.type) {
                    case yobj::element_type::point: {
                        for (auto i = elem.start; i < elem.start + elem.size;
                             i++) {
                            shp->points.push_back(vert_ids[i]);
                        }
                    } break;
                    case yobj::element_type::line: {
                        for (auto i = elem.start;
                             i < elem.start + elem.size - 1; i++) {
                            shp->lines.push_back(
                                {vert_ids[i], vert_ids[i + 1]});
                        }
                    } break;
                    case yobj::element_type::face: {
                        if (as_quads) {
                            shp->quads.push_back({vert_ids[elem.start + 0],
                                vert_ids[elem.start + 1],
                                vert_ids[elem.start + 2],
                                vert_ids[elem.start +
                                         ((elem.size == 3) ? 2 : 3)]});
                        } else if (elem.size == 3) {
                            shp->triangles.push_back({vert_ids[elem.start + 0],
                                vert_ids[elem.start + 1],
                                vert_ids[elem.start + 2]});
                        } else {
                            for (auto i = elem.start + 2;
                                 i < elem.start + elem.size; i++) {
                                shp->triangles.push_back({vert_ids[elem.start],
                                    vert_ids[i - 1], vert_ids[i]});
                            }
                        }
                    } break;
                    default: { assert(false); }
                }
            }

            // check for errors
            // copy vertex data
            auto v = oshp.verts[0];
            if (v.pos >= 0) shp->pos.resize(vert_map.size());
            if (v.texcoord >= 0) shp->texcoord.resize(vert_map.size());
            if (v.norm >= 0) shp->norm.resize(vert_map.size());
            if (v.color >= 0) shp->color.resize(vert_map.size());
            if (v.radius >= 0) shp->radius.resize(vert_map.size());
            for (auto& kv : vert_map) {
                if (v.pos >= 0 && kv.first.pos >= 0) {
                    shp->pos[kv.second] = obj->pos[kv.first.pos];
                }
                if (v.texcoord >= 0 && kv.first.texcoord >= 0) {
                    shp->texcoord[kv.second] = obj->texcoord[kv.first.texcoord];
                }
                if (v.norm >= 0 && kv.first.norm >= 0) {
                    shp->norm[kv.second] = obj->norm[kv.first.norm];
                }
                if (v.color >= 0 && kv.first.color >= 0) {
                    shp->color[kv.second] = obj->color[kv.first.color];
                }
                if (v.radius >= 0 && kv.first.radius >= 0) {
                    shp->radius[kv.second] = obj->radius[kv.first.radius];
                }
            }

            // fix smoothing
            if (!oshp.smoothing && opts.obj_facet_non_smooth) {
                auto faceted = new shape();
                faceted->name = shp->name;
                auto pidx = std::vector<int>();
                for (auto point : shp->points) {
                    faceted->points.push_back((int)pidx.size());
                    pidx.push_back(point);
                }
                for (auto line : shp->lines) {
                    faceted->lines.push_back(
                        {(int)pidx.size() + 0, (int)pidx.size() + 1});
                    pidx.push_back(line.x);
                    pidx.push_back(line.y);
                }
                for (auto triangle : shp->triangles) {
                    faceted->triangles.push_back({(int)pidx.size() + 0,
                        (int)pidx.size() + 1, (int)pidx.size() + 2});
                    pidx.push_back(triangle.x);
                    pidx.push_back(triangle.y);
                    pidx.push_back(triangle.z);
                }
                for (auto idx : pidx) {
                    if (!shp->pos.empty())
                        faceted->pos.push_back(shp->pos[idx]);
                    if (!shp->norm.empty())
                        faceted->norm.push_back(shp->norm[idx]);
                    if (!shp->texcoord.empty())
                        faceted->texcoord.push_back(shp->texcoord[idx]);
                    if (!shp->color.empty())
                        faceted->color.push_back(shp->color[idx]);
                    if (!shp->radius.empty())
                        faceted->radius.push_back(shp->radius[idx]);
                }
                delete shp;
                shp = faceted;
            }
            scn->shapes.push_back(shp);
            omap[omsh.name].push_back(shp);
        }
    }

    // convert cameras
    for (auto& ocam : obj->cameras) {
        auto cam = new camera();
        cam->name = ocam.name;
        cam->ortho = ocam.ortho;
        cam->yfov = ocam.yfov;
        cam->aspect = ocam.aspect;
        cam->aperture = ocam.aperture;
        cam->focus = ocam.focus;
        cam->frame = ocam.frame;
        scn->cameras.push_back(cam);
    }

    // convert envs
    for (auto& oenv : obj->environments) {
        auto env = new environment();
        env->name = oenv.name;
        env->mat = nullptr;
        for (auto mat : scn->materials) {
            if (mat->name == oenv.matname) { env->mat = mat; }
        }
        env->frame = oenv.frame;
        scn->environments.push_back(env);
    }

    // convert instances
    for (auto& oist : obj->instances) {
        for (auto shp : omap[oist.objname]) {
            auto ist = new instance();
            ist->name = oist.name;
            ist->shp = shp;
            ist->frame = oist.frame;
            scn->instances.push_back(ist);
        }
    }

    // done
    return scn;
}

//
// Load an obj scene
//
scene* load_obj_scene(const std::string& filename, const load_options& opts) {
    auto oscn = std::unique_ptr<yobj::scene>(
        yobj::load_obj(filename, opts.load_textures, opts.skip_missing,
            opts.obj_flip_texcoord, opts.obj_flip_tr));
    auto scn = std::unique_ptr<scene>(obj_to_scene(oscn.get(), opts));
    return scn.release();
}

//
// Save an scene
//
yobj::scene* scene_to_obj(const scene* scn) {
    auto obj = new yobj::scene();

    auto texture_path = [](const texture_info& info) {
        if (!info.txt) return std::string("");
        return info.txt->path;
    };

    auto texture_props = [](const texture_info& info) {
        if (!info.txt) return property_map<std::string>();
        auto props = info.unknown_props;
        if (info.wrap_s == texture_wrap::clamp ||
            info.wrap_t == texture_wrap::clamp) {
            props["-clamp"] = {"on"};
        }
        if (info.scale != 1) { props["-bm"] = {std::to_string(info.scale)}; }
        return props;
    };

    // convert materials
    for (auto mat : scn->materials) {
        obj->materials.emplace_back();
        auto omat = &obj->materials.back();
        omat->name = mat->name;
        omat->ke = mat->ke;
        omat->ke_txt = texture_path(mat->ke_txt);
        omat->ke_txt_info = texture_props(mat->ke_txt);
        switch (mat->mtype) {
            case material_type::specular_roughness: {
                omat->kd = mat->kd;
                omat->ks = mat->ks;
                omat->kr = mat->kr;
                omat->kt = mat->kt;
                omat->ns = (mat->rs) ? 2 / ym::pow(mat->rs, 4.0f) - 2 : 1e6;
                omat->op = mat->op;
                omat->kd_txt = texture_path(mat->kd_txt);
                omat->ks_txt = texture_path(mat->ks_txt);
                omat->kr_txt = texture_path(mat->kr_txt);
                omat->kt_txt = texture_path(mat->kt_txt);
                omat->kd_txt_info = texture_props(mat->kd_txt);
                omat->ks_txt_info = texture_props(mat->ks_txt);
                omat->kt_txt_info = texture_props(mat->kt_txt);
                omat->ns_txt_info = texture_props(mat->rs_txt);
            } break;
            case material_type::metallic_roughness: {
                if (mat->rs == 1 && mat->km == 0) {
                    omat->kd = mat->kb;
                    omat->ks = {0, 0, 0};
                    omat->ns = 1;
                } else {
                    omat->kd = mat->kb * (1 - 0.04f) * (1 - mat->km);
                    omat->ks = mat->kb * mat->km +
                               ym::vec3f{0.04f, 0.04f, 0.04f} * (1 - mat->km);
                    omat->ns = (mat->rs) ? 2 / ym::pow(mat->rs, 4.0f) - 2 : 1e6;
                }
                omat->op = mat->op;
                if (mat->km < 0.5f) {
                    omat->kd_txt = texture_path(mat->kb_txt);
                    omat->kd_txt_info = texture_props(mat->kb_txt);
                } else {
                    omat->ks_txt = texture_path(mat->kb_txt);
                    omat->ks_txt_info = texture_props(mat->kb_txt);
                }
            } break;
            case material_type::specular_glossiness: {
                omat->kd = mat->kd;
                omat->ks = mat->ks;
                omat->ns = (mat->rs) ? 2 / ym::pow(1 - mat->rs, 4.0f) - 2 : 1e6;
                omat->op = mat->op;
                omat->kd_txt = texture_path(mat->kd_txt);
                omat->ks_txt = texture_path(mat->ks_txt);
                omat->kd_txt_info = texture_props(mat->kd_txt);
                omat->ks_txt_info = texture_props(mat->ks_txt);
            } break;
        }
        omat->bump_txt = texture_path(mat->bump_txt);
        omat->disp_txt = texture_path(mat->disp_txt);
        omat->norm_txt = texture_path(mat->norm_txt);
        omat->bump_txt_info = texture_props(mat->bump_txt);
        omat->disp_txt_info = texture_props(mat->disp_txt);
        omat->norm_txt_info = texture_props(mat->norm_txt);
        omat->unknown_props = mat->unknown_props;
        if (mat->op < 1 || mat->kt != ym::zero3f) {
            omat->illum = 4;
        } else {
            omat->illum = 2;
        }
    }

    // convert shapes
    for (auto shp : scn->shapes) {
        auto offset = yobj::vertex{(int)obj->pos.size(),
            (int)obj->texcoord.size(), (int)obj->norm.size(),
            (int)obj->color.size(), (int)obj->radius.size()};
        for (auto& v : shp->pos) obj->pos.push_back(v);
        for (auto& v : shp->norm) obj->norm.push_back(v);
        for (auto& v : shp->texcoord) obj->texcoord.push_back(v);
        for (auto& v : shp->color) obj->color.push_back(v);
        for (auto& v : shp->radius) obj->radius.push_back(v);
        obj->objects.emplace_back();
        auto object = &obj->objects.back();
        object->name = shp->name;
        object->groups.emplace_back();
        auto group = &object->groups.back();
        group->matname = (shp->mat) ? shp->mat->name : "";
        for (auto point : shp->points) {
            group->elems.push_back(
                {(uint32_t)group->verts.size(), yobj::element_type::point, 1});
            group->verts.push_back(
                {(shp->pos.empty()) ? -1 : offset.pos + point,
                    (shp->texcoord.empty()) ? -1 : offset.texcoord + point,
                    (shp->norm.empty()) ? -1 : offset.norm + point,
                    (shp->color.empty()) ? -1 : offset.color + point,
                    (shp->radius.empty()) ? -1 : offset.radius + point});
        }
        for (auto line : shp->lines) {
            group->elems.push_back(
                {(uint32_t)group->verts.size(), yobj::element_type::line, 2});
            for (auto vid : line) {
                group->verts.push_back(
                    {(shp->pos.empty()) ? -1 : offset.pos + vid,
                        (shp->texcoord.empty()) ? -1 : offset.texcoord + vid,
                        (shp->norm.empty()) ? -1 : offset.norm + vid,
                        (shp->color.empty()) ? -1 : offset.color + vid,
                        (shp->radius.empty()) ? -1 : offset.radius + vid});
            }
        }
        for (auto triangle : shp->triangles) {
            group->elems.push_back(
                {(uint32_t)group->verts.size(), yobj::element_type::face, 3});
            for (auto vid : triangle) {
                group->verts.push_back(
                    {(shp->pos.empty()) ? -1 : offset.pos + vid,
                        (shp->texcoord.empty()) ? -1 : offset.texcoord + vid,
                        (shp->norm.empty()) ? -1 : offset.norm + vid,
                        (shp->color.empty()) ? -1 : offset.color + vid,
                        (shp->radius.empty()) ? -1 : offset.radius + vid});
            }
        }
        for (auto quad : shp->quads) {
            group->elems.push_back(
                {(uint32_t)group->verts.size(), yobj::element_type::face, 4});
            for (auto vid : quad) {
                group->verts.push_back(
                    {(shp->pos.empty()) ? -1 : offset.pos + vid,
                        (shp->texcoord.empty()) ? -1 : offset.texcoord + vid,
                        (shp->norm.empty()) ? -1 : offset.norm + vid,
                        (shp->color.empty()) ? -1 : offset.color + vid,
                        (shp->radius.empty()) ? -1 : offset.radius + vid});
            }
        }
    }

    // convert cameras
    for (auto cam : scn->cameras) {
        obj->cameras.emplace_back();
        auto ocam = &obj->cameras.back();
        ocam->name = cam->name;
        ocam->ortho = cam->ortho;
        ocam->yfov = cam->yfov;
        ocam->aspect = cam->aspect;
        ocam->focus = cam->focus;
        ocam->aperture = cam->aperture;
        ocam->frame = cam->frame;
    }

    // convert envs
    for (auto env : scn->environments) {
        obj->environments.emplace_back();
        auto oenv = &obj->environments.back();
        oenv->name = env->name;
        oenv->matname = (env->mat) ? env->mat->name : "";
        oenv->frame = env->frame;
    }

    // convert instances
    for (auto ist : scn->instances) {
        obj->instances.emplace_back();
        auto oist = &obj->instances.back();
        oist->name = ist->name;
        oist->objname = (ist->shp) ? ist->shp->name : "<undefined>";
        oist->frame = ist->frame;
    }

    return obj;
}

//
// Save an obj scene
//
void save_obj_scene(
    const std::string& filename, const scene* scn, const save_options& opts) {
    auto oscn = std::unique_ptr<yobj::scene>(scene_to_obj(scn));
    save_obj(filename, oscn.get(), opts.save_textures, opts.skip_missing,
        opts.obj_flip_texcoord, opts.obj_flip_tr);
}

//
// Instance gltf cameras and meshes
//
void gltf_node_to_instances(scene* scn, const std::vector<camera>& cameras,
    const std::vector<std::vector<shape*>>& meshes, const ygltf::glTF* gltf,
    ygltf::glTFid<ygltf::glTFNode> nid, const ym::mat4f& xf) {
    auto nde = gltf->get(nid);
    auto xform = xf * ygltf::node_transform(nde);
    if (nde->camera) {
        auto cam = new camera(cameras[(int)nde->camera]);
        cam->frame = ym::to_frame(xform);
        scn->cameras.push_back(cam);
    }
    if (nde->mesh) {
        for (auto shp : meshes[(int)nde->mesh]) {
            auto ist = new instance();
            ist->name = nde->name;
            ist->frame = ym::to_frame(xform);
            ist->shp = shp;
            scn->instances.push_back(ist);
        }
    }
    for (auto cid : nde->children)
        gltf_node_to_instances(scn, cameras, meshes, gltf, cid, xform);
}

//
// Flattens a gltf file into a flattened asset.
//
scene* gltf_to_scene(const ygltf::glTF* gltf) {
    // clear asset
    auto scn = new scene();

    // convert images
    for (auto gtxt : gltf->images) {
        auto txt = new texture();
        txt->name = gtxt->name;
        txt->path = (startsiwith(gtxt->uri, "data:")) ? std::string("inlines") :
                                                        gtxt->uri;
        if (!gtxt->data.datab.empty()) {
            txt->ldr = ym::image4b(gtxt->data.width, gtxt->data.height);
            for (auto j = 0; j < gtxt->data.height; j++) {
                for (auto i = 0; i < gtxt->data.width; i++) {
                    auto v = gtxt->data.datab.data() +
                             (gtxt->data.width * j + i) * gtxt->data.ncomp;
                    switch (gtxt->data.ncomp) {
                        case 1:
                            txt->ldr.at(i, j) = {v[0], v[0], v[0], 255};
                            break;
                        case 2: txt->ldr.at(i, j) = {v[0], v[1], 0, 255}; break;
                        case 3:
                            txt->ldr.at(i, j) = {v[0], v[1], v[2], 255};
                            break;
                        case 4:
                            txt->ldr.at(i, j) = {v[0], v[1], v[2], v[3]};
                            break;
                        default: assert(false); break;
                    }
                }
            }
        } else if (!gtxt->data.dataf.empty()) {
            txt->hdr = ym::image4f(gtxt->data.width, gtxt->data.height);
            for (auto j = 0; j < gtxt->data.height; j++) {
                for (auto i = 0; i < gtxt->data.width; i++) {
                    auto v = gtxt->data.dataf.data() +
                             (gtxt->data.width * j + i) * gtxt->data.ncomp;
                    switch (gtxt->data.ncomp) {
                        case 1:
                            txt->hdr.at(i, j) = {v[0], v[0], v[0], 1};
                            break;
                        case 2: txt->hdr.at(i, j) = {v[0], v[1], 0, 1}; break;
                        case 3:
                            txt->hdr.at(i, j) = {v[0], v[1], v[2], 1};
                            break;
                        case 4:
                            txt->hdr.at(i, j) = {v[0], v[1], v[2], v[3]};
                            break;
                        default: assert(false); break;
                    }
                }
            }
        }
        scn->textures.push_back(txt);
    }

    // maps for translation
    static const auto filter_min_map =
        std::unordered_map<ygltf::glTFSamplerMinFilter, texture_filter>{
            {ygltf::glTFSamplerMinFilter::NotSet,
                texture_filter::linear_mipmap_linear},
            {ygltf::glTFSamplerMinFilter::Linear, texture_filter::linear},
            {ygltf::glTFSamplerMinFilter::Nearest, texture_filter::nearest},
            {ygltf::glTFSamplerMinFilter::LinearMipmapLinear,
                texture_filter::linear_mipmap_linear},
            {ygltf::glTFSamplerMinFilter::LinearMipmapNearest,
                texture_filter::linear_mipmap_nearest},
            {ygltf::glTFSamplerMinFilter::NearestMipmapLinear,
                texture_filter::nearest_mipmap_linear},
            {ygltf::glTFSamplerMinFilter::NearestMipmapNearest,
                texture_filter::nearest_mipmap_nearest},
        };
    static const auto filter_mag_map =
        std::unordered_map<ygltf::glTFSamplerMagFilter, texture_filter>{
            {ygltf::glTFSamplerMagFilter::NotSet, texture_filter::linear},
            {ygltf::glTFSamplerMagFilter::Linear, texture_filter::linear},
            {ygltf::glTFSamplerMagFilter::Nearest, texture_filter::nearest},
        };
    static const auto wrap_s_map =
        std::unordered_map<ygltf::glTFSamplerWrapS, texture_wrap>{
            {ygltf::glTFSamplerWrapS::NotSet, texture_wrap::repeat},
            {ygltf::glTFSamplerWrapS::Repeat, texture_wrap::repeat},
            {ygltf::glTFSamplerWrapS::ClampToEdge, texture_wrap::clamp},
            {ygltf::glTFSamplerWrapS::MirroredRepeat, texture_wrap::mirror},
        };
    static const auto wrap_t_map =
        std::unordered_map<ygltf::glTFSamplerWrapT, texture_wrap>{
            {ygltf::glTFSamplerWrapT::NotSet, texture_wrap::repeat},
            {ygltf::glTFSamplerWrapT::Repeat, texture_wrap::repeat},
            {ygltf::glTFSamplerWrapT::ClampToEdge, texture_wrap::clamp},
            {ygltf::glTFSamplerWrapT::MirroredRepeat, texture_wrap::mirror},
        };

    // add a texture
    auto add_texture = [gltf, scn](ygltf::glTFTextureInfo* ginfo,
                           bool normal = false, bool occlusion = false) {
        auto info = texture_info();
        if (!ginfo) return info;
        auto gtxt = gltf->get(ginfo->index);
        if (!gtxt || !gtxt->source) return info;
        auto txt = scn->textures.at((int)gtxt->source);
        if (!txt) return info;
        info.txt = scn->textures.at((int)gtxt->source);
        auto gsmp = gltf->get(gtxt->sampler);
        if (gsmp) {
            info.filter_mag = filter_mag_map.at(gsmp->magFilter);
            info.filter_min = filter_min_map.at(gsmp->minFilter);
            info.wrap_s = wrap_s_map.at(gsmp->wrapS);
            info.wrap_t = wrap_t_map.at(gsmp->wrapT);
        }
        if (normal) {
            auto ninfo = (ygltf::glTFMaterialNormalTextureInfo*)ginfo;
            info.scale = ninfo->scale;
        }
        if (occlusion) {
            auto ninfo = (ygltf::glTFMaterialOcclusionTextureInfo*)ginfo;
            info.scale = ninfo->strength;
        }
        return info;
    };

    // convert materials
    for (auto gmat : gltf->materials) {
        auto mat = new material();
        mat->name = gmat->name;
        mat->ke = gmat->emissiveFactor;
        mat->ke_txt = add_texture(gmat->emissiveTexture);
        if (gmat->pbrMetallicRoughness) {
            mat->mtype = material_type::metallic_roughness;
            auto gmr = gmat->pbrMetallicRoughness;
            mat->kb = {gmr->baseColorFactor[0], gmr->baseColorFactor[1],
                gmr->baseColorFactor[2]};
            mat->op = gmr->baseColorFactor[3];
            mat->km = gmr->metallicFactor;
            mat->rs = gmr->roughnessFactor;
            mat->kb_txt = add_texture(gmr->baseColorTexture);
            mat->km_txt = add_texture(gmr->metallicRoughnessTexture);
        }
        if (gmat->pbrSpecularGlossiness) {
            mat->mtype = material_type::specular_glossiness;
            auto gsg = gmat->pbrSpecularGlossiness;
            mat->kd = {gsg->diffuseFactor[0], gsg->diffuseFactor[1],
                gsg->diffuseFactor[2]};
            mat->op = gsg->diffuseFactor[3];
            mat->ks = gsg->specularFactor;
            mat->rs = gsg->glossinessFactor;
            mat->kd_txt = add_texture(gsg->diffuseTexture);
            mat->ks_txt = add_texture(gsg->specularGlossinessTexture);
        }
        mat->norm_txt = add_texture(gmat->normalTexture, true, false);
        mat->occ_txt = add_texture(gmat->occlusionTexture, false, true);
        mat->double_sided = gmat->doubleSided;
        scn->materials.push_back(mat);
    }

    // convert meshes
    auto meshes = std::vector<std::vector<shape*>>();
    for (auto gmesh : gltf->meshes) {
        meshes.push_back({});
        // primitives
        for (auto gprim : gmesh->primitives) {
            auto shp = new shape();
            if (gprim->material) {
                shp->mat = scn->materials[(int)gprim->material];
            }
            // vertex data
            for (auto gattr : gprim->attributes) {
                auto semantic = gattr.first;
                auto vals = ygltf::accessor_view(gltf, gltf->get(gattr.second));
                if (semantic == "POSITION") {
                    shp->pos.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->pos.push_back(vals.getv<3>(i));
                } else if (semantic == "NORMAL") {
                    shp->norm.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->norm.push_back(vals.getv<3>(i));
                } else if (semantic == "TEXCOORD" || semantic == "TEXCOORD_0") {
                    shp->texcoord.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->texcoord.push_back(vals.getv<2>(i));
                } else if (semantic == "TEXCOORD_1") {
                    shp->texcoord1.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->texcoord1.push_back(vals.getv<2>(i));
                } else if (semantic == "COLOR" || semantic == "COLOR_0") {
                    shp->color.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->color.push_back(vals.getv<4>(i, {0, 0, 0, 1}));
                } else if (semantic == "TANGENT") {
                    shp->tangsp.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->tangsp.push_back(vals.getv<4>(i));
                } else if (semantic == "RADIUS") {
                    shp->radius.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->radius.push_back(vals.get(i, 0));
                } else {
                    // ignore
                }
            }
            // indices
            if (!gprim->indices) {
                switch (gprim->mode) {
                    case ygltf::glTFMeshPrimitiveMode::Triangles: {
                        shp->triangles.reserve(shp->pos.size() / 3);
                        for (auto i = 0; i < shp->pos.size() / 3; i++) {
                            shp->triangles.push_back(
                                {i * 3 + 0, i * 3 + 1, i * 3 + 2});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::TriangleFan: {
                        shp->triangles.reserve(shp->pos.size() - 2);
                        for (auto i = 2; i < shp->pos.size(); i++) {
                            shp->triangles.push_back({0, i - 1, i});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::TriangleStrip: {
                        shp->triangles.reserve(shp->pos.size() - 2);
                        for (auto i = 2; i < shp->pos.size(); i++) {
                            shp->triangles.push_back({i - 2, i - 1, i});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::Lines: {
                        shp->lines.reserve(shp->pos.size() / 2);
                        for (auto i = 0; i < shp->pos.size() / 2; i++) {
                            shp->lines.push_back({i * 2 + 0, i * 2 + 1});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::LineLoop: {
                        shp->lines.reserve(shp->pos.size());
                        for (auto i = 1; i < shp->pos.size(); i++) {
                            shp->lines.push_back({i - 1, i});
                        }
                        shp->lines.back() = {(int)shp->pos.size() - 1, 0};
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::LineStrip: {
                        shp->lines.reserve(shp->pos.size() - 1);
                        for (auto i = 1; i < shp->pos.size(); i++) {
                            shp->lines.push_back({i - 1, i});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::NotSet:
                    case ygltf::glTFMeshPrimitiveMode::Points: {
                        shp->points.reserve(shp->pos.size());
                        for (auto i = 0; i < shp->pos.size(); i++) {
                            shp->points.push_back(i);
                        }
                    } break;
                }
            } else {
                auto indices =
                    ygltf::accessor_view(gltf, gltf->get(gprim->indices));
                switch (gprim->mode) {
                    case ygltf::glTFMeshPrimitiveMode::Triangles: {
                        shp->triangles.reserve(indices.size());
                        for (auto i = 0; i < indices.size() / 3; i++) {
                            shp->triangles.push_back({indices.geti(i * 3 + 0),
                                indices.geti(i * 3 + 1),
                                indices.geti(i * 3 + 2)});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::TriangleFan: {
                        shp->triangles.reserve(indices.size() - 2);
                        for (auto i = 2; i < indices.size(); i++) {
                            shp->triangles.push_back({indices.geti(0),
                                indices.geti(i - 1), indices.geti(i)});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::TriangleStrip: {
                        shp->triangles.reserve(indices.size() - 2);
                        for (auto i = 2; i < indices.size(); i++) {
                            shp->triangles.push_back({indices.geti(i - 2),
                                indices.geti(i - 1), indices.geti(i)});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::Lines: {
                        shp->lines.reserve(indices.size() / 2);
                        for (auto i = 0; i < indices.size() / 2; i++) {
                            shp->lines.push_back({indices.geti(i * 2 + 0),
                                indices.geti(i * 2 + 1)});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::LineLoop: {
                        shp->lines.reserve(indices.size());
                        for (auto i = 1; i < indices.size(); i++) {
                            shp->lines.push_back(
                                {indices.geti(i - 1), indices.geti(i)});
                        }
                        shp->lines.back() = {
                            indices.geti(indices.size() - 1), indices.geti(0)};
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::LineStrip: {
                        shp->lines.reserve(indices.size() - 1);
                        for (auto i = 1; i < indices.size(); i++) {
                            shp->lines.push_back(
                                {indices.geti(i - 1), indices.geti(i)});
                        }
                    } break;
                    case ygltf::glTFMeshPrimitiveMode::NotSet:
                    case ygltf::glTFMeshPrimitiveMode::Points: {
                        shp->points.reserve(indices.size());
                        for (auto i = 0; i < indices.size(); i++) {
                            shp->points.push_back(indices.geti(i));
                        }
                    } break;
                }
            }
            scn->shapes.push_back(shp);
            meshes.back().push_back(shp);
        }
    }

    // convert cameras
    auto cameras = std::vector<camera>();
    for (auto gcam : gltf->cameras) {
        cameras.push_back({});
        auto cam = &cameras.back();
        cam->name = gcam->name;
        cam->ortho = gcam->type == ygltf::glTFCameraType::Orthographic;
        if (cam->ortho) {
            auto ortho = gcam->orthographic;
            cam->yfov = ortho->ymag;
            cam->aspect = ortho->xmag / ortho->ymag;
            cam->near = ortho->znear;
            cam->far = ortho->zfar;
        } else {
            auto persp = gcam->perspective;
            cam->yfov = persp->yfov;
            cam->aspect = persp->aspectRatio;
            if (!cam->aspect) cam->aspect = 16.0f / 9.0f;
            cam->near = persp->znear;
            cam->far = persp->zfar;
        }
    }

    // instance meshes and cameras
    if (gltf->scene) {
        for (auto nid : gltf->get(gltf->scene)->nodes) {
            gltf_node_to_instances(
                scn, cameras, meshes, gltf, nid, ym::identity_mat4f);
        }
    } else if (!gltf->nodes.empty()) {
        // set up node children and root nodes
        auto is_root = std::vector<bool>(gltf->nodes.size(), true);
        for (auto nid = 0; nid < gltf->nodes.size(); nid++) {
            for (auto cid :
                gltf->get(ygltf::glTFid<ygltf::glTFNode>(nid))->children)
                is_root[(int)cid] = false;
        }
        for (auto nid = 0; nid < gltf->nodes.size(); nid++) {
            if (!is_root[nid]) continue;
            gltf_node_to_instances(scn, cameras, meshes, gltf,
                ygltf::glTFid<ygltf::glTFNode>(nid), ym::identity_mat4f);
        }
    }

    return scn;
}

//
// Load an gltf scene
//
scene* load_gltf_scene(const std::string& filename, const load_options& opts) {
    auto gscn = std::unique_ptr<ygltf::glTF>(ygltf::load_gltf(
        filename, true, opts.load_textures, opts.skip_missing));
    auto scn = std::unique_ptr<scene>(gltf_to_scene(gscn.get()));
    if (!scn) {
        throw std::runtime_error("could not convert gltf scene");
        return nullptr;
    }
    return scn.release();
}

//
// helper
//
template <typename T>
static inline int index(const std::vector<T*>& vec, T* val) {
    auto pos = std::find(vec.begin(), vec.end(), val);
    if (pos == vec.end()) return -1;
    return (int)(pos - vec.begin());
}

//
// Unflattnes gltf
//
ygltf::glTF* scene_to_gltf(
    const scene* scn, const std::string& buffer_uri, bool separate_buffers) {
    auto gltf = std::unique_ptr<ygltf::glTF>(new ygltf::glTF());

    // add asset info
    gltf->asset = new ygltf::glTFAsset();
    gltf->asset->generator = "Yocto/gltf";
    gltf->asset->version = "2.0";

    // convert cameras
    for (auto cam : scn->cameras) {
        auto gcam = new ygltf::glTFCamera();
        gcam->name = cam->name;
        gcam->type = (cam->ortho) ? ygltf::glTFCameraType::Orthographic :
                                    ygltf::glTFCameraType::Perspective;
        if (cam->ortho) {
            auto ortho = new ygltf::glTFCameraOrthographic();
            ortho->ymag = cam->yfov;
            ortho->xmag = cam->aspect * cam->yfov;
            ortho->znear = cam->near;
            ortho->znear = cam->far;
            gcam->orthographic = ortho;
        } else {
            auto persp = new ygltf::glTFCameraPerspective();
            persp->yfov = cam->yfov;
            persp->aspectRatio = cam->aspect;
            persp->znear = cam->near;
            persp->zfar = cam->far;
            gcam->perspective = persp;
        }
        gltf->cameras.push_back(gcam);
    }

    // convert images
    for (auto txt : scn->textures) {
        auto gimg = new ygltf::glTFImage();
        gimg->uri = txt->path;
        if (txt->hdr) {
            gimg->data.width = txt->hdr.width();
            gimg->data.height = txt->hdr.height();
            gimg->data.ncomp = 4;
            gimg->data.dataf.assign((uint8_t*)txt->hdr.data(),
                (uint8_t*)txt->hdr.data() +
                    txt->hdr.width() * txt->hdr.height() * 4);
        }
        if (txt->ldr) {
            gimg->data.width = txt->ldr.width();
            gimg->data.height = txt->ldr.height();
            gimg->data.ncomp = 4;
            gimg->data.datab.assign((uint8_t*)txt->ldr.data(),
                (uint8_t*)txt->ldr.data() +
                    txt->ldr.width() * txt->ldr.height() * 4);
        }
        gltf->images.push_back(gimg);
    }

    // conversion maps
    static const auto wrap_s_map =
        std::map<texture_wrap, ygltf::glTFSamplerWrapS>{
            {texture_wrap::repeat, ygltf::glTFSamplerWrapS::Repeat},
            {texture_wrap::clamp, ygltf::glTFSamplerWrapS::ClampToEdge},
            {texture_wrap::mirror, ygltf::glTFSamplerWrapS::MirroredRepeat},
        };
    static const auto wrap_t_map =
        std::map<texture_wrap, ygltf::glTFSamplerWrapT>{
            {texture_wrap::repeat, ygltf::glTFSamplerWrapT::Repeat},
            {texture_wrap::clamp, ygltf::glTFSamplerWrapT::ClampToEdge},
            {texture_wrap::mirror, ygltf::glTFSamplerWrapT::MirroredRepeat},
        };
    static const auto texture_min_map =
        std::map<texture_filter, ygltf::glTFSamplerMinFilter>{
            {texture_filter::linear, ygltf::glTFSamplerMinFilter::Linear},
            {texture_filter::nearest, ygltf::glTFSamplerMinFilter::Nearest},
            {texture_filter::linear_mipmap_linear,
                ygltf::glTFSamplerMinFilter::LinearMipmapLinear},
            {texture_filter::linear_mipmap_nearest,
                ygltf::glTFSamplerMinFilter::LinearMipmapNearest},
            {texture_filter::nearest_mipmap_linear,
                ygltf::glTFSamplerMinFilter::NearestMipmapNearest},
            {texture_filter::nearest_mipmap_nearest,
                ygltf::glTFSamplerMinFilter::NearestMipmapNearest},
        };
    static const auto texture_mag_map =
        std::map<texture_filter, ygltf::glTFSamplerMagFilter>{
            {texture_filter::linear, ygltf::glTFSamplerMagFilter::Linear},
            {texture_filter::nearest, ygltf::glTFSamplerMagFilter::Nearest},
        };

    // add a texture and sampler
    auto add_texture = [&gltf, scn](const texture_info& info, bool norm = false,
                           bool occ = false) {
        if (!info.txt) return (ygltf::glTFTextureInfo*)nullptr;
        auto gtxt = new ygltf::glTFTexture();
        gtxt->name = info.txt->name;
        gtxt->source =
            ygltf::glTFid<ygltf::glTFImage>(index(scn->textures, info.txt));

        /// check if it is default
        auto is_default =
            info.wrap_s == texture_wrap::repeat &&
            info.wrap_t == texture_wrap::repeat &&
            info.filter_mag == texture_filter::linear &&
            info.filter_min == texture_filter::linear_mipmap_linear;

        if (!is_default) {
            auto gsmp = new ygltf::glTFSampler();
            gsmp->wrapS = wrap_s_map.at(info.wrap_s);
            gsmp->wrapT = wrap_t_map.at(info.wrap_t);
            gsmp->minFilter = texture_min_map.at(info.filter_min);
            gsmp->magFilter = texture_mag_map.at(info.filter_mag);
            gtxt->sampler =
                ygltf::glTFid<ygltf::glTFSampler>((int)gltf->samplers.size());
            gltf->samplers.push_back(gsmp);
        }
        gltf->textures.push_back(gtxt);
        if (norm) {
            auto ginfo = new ygltf::glTFMaterialNormalTextureInfo();
            ginfo->index = ygltf::glTFid<ygltf::glTFTexture>{
                (int)gltf->textures.size() - 1};
            ginfo->scale = info.scale;
            return (ygltf::glTFTextureInfo*)ginfo;
        } else if (occ) {
            auto ginfo = new ygltf::glTFMaterialOcclusionTextureInfo();
            ginfo->index = ygltf::glTFid<ygltf::glTFTexture>{
                (int)gltf->textures.size() - 1};
            ginfo->strength = info.scale;
            return (ygltf::glTFTextureInfo*)ginfo;
        } else {
            auto ginfo = new ygltf::glTFTextureInfo();
            ginfo->index = ygltf::glTFid<ygltf::glTFTexture>{
                (int)gltf->textures.size() - 1};
            return ginfo;
        }
    };

    // convert materials
    for (auto mat : scn->materials) {
        auto gmat = new ygltf::glTFMaterial();
        gmat->name = mat->name;
        gmat->emissiveFactor = mat->ke;
        gmat->emissiveTexture = add_texture(mat->ke_txt);
        switch (mat->mtype) {
            case material_type::specular_roughness: {
                gmat->pbrSpecularGlossiness =
                    new ygltf::glTFMaterialPbrSpecularGlossiness();
                auto gsg = gmat->pbrSpecularGlossiness;
                gsg->diffuseFactor = {
                    mat->kd[0], mat->kd[1], mat->kd[2], mat->op};
                gsg->specularFactor = mat->ks;
                gsg->glossinessFactor = mat->rs;
                gsg->diffuseTexture = add_texture(mat->kd_txt);
                gsg->specularGlossinessTexture = add_texture(mat->ks_txt);

            } break;
            case material_type::metallic_roughness: {
                gmat->pbrMetallicRoughness =
                    new ygltf::glTFMaterialPbrMetallicRoughness();
                auto gmr = gmat->pbrMetallicRoughness;
                gmr->baseColorFactor = {
                    mat->kb[0], mat->kb[1], mat->kb[2], mat->op};
                gmr->metallicFactor = mat->km;
                gmr->roughnessFactor = mat->rs;
                gmr->baseColorTexture = add_texture(mat->kb_txt);

                gmr->metallicRoughnessTexture = add_texture(mat->km_txt);

            } break;
            case material_type::specular_glossiness: {
                gmat->pbrSpecularGlossiness =
                    new ygltf::glTFMaterialPbrSpecularGlossiness();
                auto gsg = gmat->pbrSpecularGlossiness;
                gsg->diffuseFactor = {
                    mat->kd[0], mat->kd[1], mat->kd[2], mat->op};
                gsg->specularFactor = mat->ks;
                gsg->glossinessFactor = mat->rs;
                gsg->diffuseTexture = add_texture(mat->kd_txt);
                gsg->specularGlossinessTexture = add_texture(mat->ks_txt);
            } break;
        }
        gmat->normalTexture =
            (ygltf::glTFMaterialNormalTextureInfo*)add_texture(
                mat->norm_txt, true, false);
        gmat->occlusionTexture =
            (ygltf::glTFMaterialOcclusionTextureInfo*)add_texture(
                mat->occ_txt, false, true);
        gmat->doubleSided = mat->double_sided;
        gltf->materials.push_back(gmat);
    }

    // add buffer
    auto add_buffer = [&gltf](const std::string& buffer_uri) {
        auto gbuffer = new ygltf::glTFBuffer();
        gltf->buffers.push_back(gbuffer);
        gbuffer->uri = buffer_uri;
        return gbuffer;
    };

    // init buffers
    auto gbuffer_global = add_buffer(buffer_uri);

    // add an optional buffer
    auto add_opt_buffer = [&gbuffer_global, buffer_uri, &add_buffer,
                              separate_buffers](const std::string& uri) {
        if (separate_buffers && uri != "") {
            return add_buffer(uri);
        } else {
            if (!gbuffer_global) gbuffer_global = add_buffer(buffer_uri);
            return gbuffer_global;
        }
    };

    // attribute handling
    auto add_accessor = [&gltf](ygltf::glTFBuffer* gbuffer,
                            const std::string& name,
                            ygltf::glTFAccessorType type,
                            ygltf::glTFAccessorComponentType ctype, int count,
                            int csize, const void* data, bool save_min_max) {
        gltf->bufferViews.push_back(new ygltf::glTFBufferView());
        auto bufferView = gltf->bufferViews.back();
        bufferView->buffer =
            ygltf::glTFid<ygltf::glTFBuffer>(index(gltf->buffers, gbuffer));
        bufferView->byteOffset = (int)gbuffer->data.size();
        bufferView->byteStride = 0;
        bufferView->byteLength = count * csize;
        gbuffer->data.resize(gbuffer->data.size() + bufferView->byteLength);
        gbuffer->byteLength += bufferView->byteLength;
        auto ptr = gbuffer->data.data() + gbuffer->data.size() -
                   bufferView->byteLength;
        bufferView->target = ygltf::glTFBufferViewTarget::ArrayBuffer;
        memcpy(ptr, data, bufferView->byteLength);
        gltf->accessors.push_back(new ygltf::glTFAccessor());
        auto accessor = gltf->accessors.back();
        accessor->bufferView = ygltf::glTFid<ygltf::glTFBufferView>(
            (int)gltf->bufferViews.size() - 1);
        accessor->byteOffset = 0;
        accessor->componentType = ctype;
        accessor->count = count;
        accessor->type = type;
        if (save_min_max && count &&
            ctype == ygltf::glTFAccessorComponentType::Float) {
            switch (type) {
                case ygltf::glTFAccessorType::Scalar: {
                    auto bbox = ym::make_bbox(count, (ym::vec1f*)data);
                    accessor->min = {bbox.min.x};
                    accessor->max = {bbox.max.x};
                } break;
                case ygltf::glTFAccessorType::Vec2: {
                    auto bbox = ym::make_bbox(count, (ym::vec2f*)data);
                    accessor->min = {bbox.min.x, bbox.min.y};
                    accessor->max = {bbox.max.x, bbox.max.y};
                } break;
                case ygltf::glTFAccessorType::Vec3: {
                    auto bbox = ym::make_bbox(count, (ym::vec3f*)data);
                    accessor->min = {bbox.min.x, bbox.min.y, bbox.min.z};
                    accessor->max = {bbox.max.x, bbox.max.y, bbox.max.z};
                } break;
                case ygltf::glTFAccessorType::Vec4: {
                    auto bbox = ym::make_bbox(count, (ym::vec4f*)data);
                    accessor->min = {
                        bbox.min.x, bbox.min.y, bbox.min.z, bbox.min.w};
                    accessor->max = {
                        bbox.max.x, bbox.max.y, bbox.max.z, bbox.max.w};
                } break;
                default: break;
            }
        }
        return ygltf::glTFid<ygltf::glTFAccessor>(
            (int)gltf->accessors.size() - 1);
    };

    // convert meshes
    for (auto shp : scn->shapes) {
        auto gbuffer = add_opt_buffer(shp->path);
        auto gmesh = new ygltf::glTFMesh();
        gmesh->name = shp->name;
        auto gprim = new ygltf::glTFMeshPrimitive();
        gprim->material =
            ygltf::glTFid<ygltf::glTFMaterial>(index(scn->materials, shp->mat));
        if (!shp->pos.empty())
            gprim->attributes["POSITION"] = add_accessor(gbuffer,
                shp->name + "_pos", ygltf::glTFAccessorType::Vec3,
                ygltf::glTFAccessorComponentType::Float, (int)shp->pos.size(),
                sizeof(ym::vec3f), shp->pos.data(), true);
        if (!shp->norm.empty())
            gprim->attributes["NORMAL"] = add_accessor(gbuffer,
                shp->name + "_norm", ygltf::glTFAccessorType::Vec3,
                ygltf::glTFAccessorComponentType::Float, (int)shp->norm.size(),
                sizeof(ym::vec3f), shp->norm.data(), false);
        if (!shp->texcoord.empty())
            gprim->attributes["TEXCOORD_0"] = add_accessor(gbuffer,
                shp->name + "_texcoord", ygltf::glTFAccessorType::Vec2,
                ygltf::glTFAccessorComponentType::Float,
                (int)shp->texcoord.size(), sizeof(ym::vec2f),
                shp->texcoord.data(), false);
        if (!shp->texcoord1.empty())
            gprim->attributes["TEXCOORD_1"] = add_accessor(gbuffer,
                shp->name + "_texcoord1", ygltf::glTFAccessorType::Vec2,
                ygltf::glTFAccessorComponentType::Float,
                (int)shp->texcoord1.size(), sizeof(ym::vec2f),
                shp->texcoord1.data(), false);
        if (!shp->color.empty())
            gprim->attributes["COLOR_0"] = add_accessor(gbuffer,
                shp->name + "_color", ygltf::glTFAccessorType::Vec4,
                ygltf::glTFAccessorComponentType::Float, (int)shp->color.size(),
                sizeof(ym::vec4f), shp->color.data(), false);
        if (!shp->radius.empty())
            gprim->attributes["RADIUS"] = add_accessor(gbuffer,
                shp->name + "_radius", ygltf::glTFAccessorType::Scalar,
                ygltf::glTFAccessorComponentType::Float,
                (int)shp->radius.size(), sizeof(float), shp->radius.data(),
                false);
        // auto elem_as_uint = shp->pos.size() >
        // std::numeric_limits<unsigned short>::max();
        if (!shp->points.empty()) {
            gprim->indices = add_accessor(gbuffer, shp->name + "_points",
                ygltf::glTFAccessorType::Scalar,
                ygltf::glTFAccessorComponentType::UnsignedInt,
                (int)shp->points.size(), sizeof(int), (int*)shp->points.data(),
                false);
            gprim->mode = ygltf::glTFMeshPrimitiveMode::Points;
        } else if (!shp->lines.empty()) {
            gprim->indices = add_accessor(gbuffer, shp->name + "_lines",
                ygltf::glTFAccessorType::Scalar,
                ygltf::glTFAccessorComponentType::UnsignedInt,
                (int)shp->lines.size() * 2, sizeof(int),
                (int*)shp->lines.data(), false);
            gprim->mode = ygltf::glTFMeshPrimitiveMode::Lines;
        } else if (!shp->triangles.empty()) {
            gprim->indices = add_accessor(gbuffer, shp->name + "_triangles",
                ygltf::glTFAccessorType::Scalar,
                ygltf::glTFAccessorComponentType::UnsignedInt,
                (int)shp->triangles.size() * 3, sizeof(int),
                (int*)shp->triangles.data(), false);
            gprim->mode = ygltf::glTFMeshPrimitiveMode::Triangles;
        } else if (!shp->quads.empty()) {
            auto triangles = ym::convert_quads_to_triangles(shp->quads);
            gprim->indices = add_accessor(gbuffer, shp->name + "_quads",
                ygltf::glTFAccessorType::Scalar,
                ygltf::glTFAccessorComponentType::UnsignedInt,
                (int)triangles.size() * 3, sizeof(int), (int*)triangles.data(),
                false);
            gprim->mode = ygltf::glTFMeshPrimitiveMode::Triangles;
        } else {
            assert(false);
        }
        gmesh->primitives.push_back(gprim);
        gltf->meshes.push_back(gmesh);
    }

    // instances
    for (auto ist : scn->instances) {
        auto gnode = new ygltf::glTFNode();
        gnode->name = ist->name;
        gnode->mesh =
            ygltf::glTFid<ygltf::glTFMesh>(index(scn->shapes, ist->shp));
        gnode->matrix = ym::to_mat(ist->frame);
        gltf->nodes.push_back(gnode);
    }

    // cameras
    for (auto cam : scn->cameras) {
        auto gnode = new ygltf::glTFNode();
        gnode->name = cam->name;
        gnode->camera =
            ygltf::glTFid<ygltf::glTFCamera>(index(scn->cameras, cam));
        gnode->matrix = ym::to_mat(cam->frame);
        gltf->nodes.push_back(gnode);
    }

    // scenes
    if (!gltf->nodes.empty()) {
        auto gscene = new ygltf::glTFScene();
        gscene->name = "scene";
        for (auto i = 0; i < gltf->nodes.size(); i++) {
            gscene->nodes.push_back(ygltf::glTFid<ygltf::glTFNode>(i));
        }
        gltf->scenes.push_back(gscene);
        gltf->scene = ygltf::glTFid<ygltf::glTFScene>(0);
    }

    // done
    return gltf.release();
}

//
// Load a scene
//
scene* load_scene(const std::string& filename, const load_options& opts) {
    auto ext = get_extension(filename);
    if (ext == ".obj" || ext == ".OBJ") return load_obj_scene(filename, opts);
    if (ext == ".gltf" || ext == ".GLTF")
        return load_gltf_scene(filename, opts);
    throw std::runtime_error("unsupported extension " + ext);
    return nullptr;
}

//
// Save a gltf scene
//
void save_gltf_scene(
    const std::string& filename, const scene* scn, const save_options& opts) {
    auto buffer_uri = get_basename(filename) + ".bin";
    auto gscn = std::unique_ptr<ygltf::glTF>(
        scene_to_gltf(scn, buffer_uri, opts.gltf_separate_buffers));
    save_gltf(filename, gscn.get(), true, opts.save_textures);
}

//
// Save a scene
//
void save_scene(
    const std::string& filename, const scene* scn, const save_options& opts) {
    auto ext = get_extension(filename);
    if (ext == ".obj" || ext == ".OBJ")
        return save_obj_scene(filename, scn, opts);
    if (ext == ".gltf" || ext == ".GLTF")
        return save_gltf_scene(filename, scn, opts);
    throw std::runtime_error("unsupported extension " + ext);
}

//
// Add missing values and elements
//
void add_elements(scene* scn, const add_elements_options& opts) {
    if (opts.smooth_normals) {
        for (auto shp : scn->shapes) {
            if (!shp->norm.empty()) continue;
            shp->norm.resize(shp->pos.size());
            if (!shp->points.empty()) {
                shp->norm.assign(shp->pos.size(), {0, 0, 1});
            } else if (!shp->lines.empty()) {
                ym::compute_tangents(shp->lines, shp->pos, shp->norm);
            } else if (!shp->triangles.empty()) {
                ym::compute_normals(shp->triangles, shp->pos, shp->norm);
            } else if (!shp->quads.empty()) {
                ym::compute_normals(shp->quads, shp->pos, shp->norm);
            }
        }
    }

    if (opts.tangent_space) {
        for (auto shp : scn->shapes) {
            if (!shp->tangsp.empty() || shp->triangles.empty() ||
                shp->texcoord.empty() || (shp->mat))
                continue;
            shp->tangsp.resize(shp->pos.size());
            ym::compute_tangent_frame(shp->triangles, shp->pos, shp->norm,
                shp->texcoord, shp->tangsp);
        }
    }

    if (opts.pointline_radius > 0) {
        for (auto shp : scn->shapes) {
            if ((shp->points.empty() && shp->lines.empty()) ||
                !shp->radius.empty())
                continue;
            shp->radius.resize(shp->pos.size(), opts.pointline_radius);
        }
    }

    if (opts.texture_data) {
        for (auto txt : scn->textures) {
            if (!txt->hdr && !txt->ldr) {
                printf("unable to load texture %s\n", txt->path.c_str());
                txt->ldr = ym::image4b(1, 1, {255, 255, 255, 255});
            }
        }
    }

    if (opts.shape_instances) {
        if (!scn->instances.empty()) return;
        for (auto shp : scn->shapes) {
            auto ist = new instance();
            ist->name = shp->name;
            ist->shp = shp;
            scn->instances.push_back(ist);
        }
    }

    if (opts.default_names || opts.default_paths) {
        auto cid = 0;
        for (auto cam : scn->cameras) {
            if (cam->name.empty())
                cam->name = "unnamed_camera_" + std::to_string(cid);
            cid++;
        }

        auto tid = 0;
        for (auto txt : scn->textures) {
            if (txt->name.empty())
                txt->name = "unnamed_texture_" + std::to_string(tid);
            tid++;
        }

        auto mid = 0;
        for (auto mat : scn->materials) {
            if (mat->name.empty())
                mat->name = "unnamed_material_" + std::to_string(mid);
            mid++;
        }

        auto sid = 0;
        for (auto shp : scn->shapes) {
            if (shp->name.empty())
                shp->name = "unnamed_shape_" + std::to_string(sid);
            sid++;
        }

        auto iid = 0;
        for (auto ist : scn->instances) {
            if (ist->name.empty())
                ist->name = "unnamed_instance_" + std::to_string(iid);
            iid++;
        }

        auto eid = 0;
        for (auto env : scn->environments) {
            if (env->name.empty())
                env->name = "unnamed_environment_" + std::to_string(eid);
            eid++;
        }
    }

    if (opts.default_paths) {
        for (auto txt : scn->textures) {
            if (txt->path != "") continue;
            txt->path = txt->name + ".png";
        }
        for (auto shp : scn->shapes) {
            if (shp->path != "") continue;
            shp->path = shp->name + ".bin";
        }
    }

    if (opts.default_camera && scn->cameras.empty()) {
        auto bbox = compute_bounds(scn);
        auto center = ym::center(bbox);
        auto bbox_size = ym::diagonal(bbox);
        auto bbox_msize =
            ym::max(bbox_size[0], ym::max(bbox_size[1], bbox_size[2]));
        // set up camera
        auto cam = new camera();
        cam->name = "default_camera";
        auto camera_dir = ym::vec3f{1, 0.4f, 1};
        auto from = camera_dir * bbox_msize + center;
        auto to = center;
        auto up = ym::vec3f{0, 1, 0};
        cam->frame = ym::lookat_frame3(from, to, up);
        cam->ortho = false;
        cam->aspect = 16.0f / 9.0f;
        cam->yfov = 2 * atanf(0.5f);
        cam->aperture = 0;
        cam->focus = ym::length(to - from);
        scn->cameras.push_back(cam);
    }
}

//
// Merge scene into one another
//
void merge_into(scene* merge_into, scene* merge_from) {
    merge_into->cameras.insert(merge_from->cameras.begin(),
        merge_from->cameras.end(), merge_into->cameras.end());
    merge_from->cameras.clear();
    merge_into->textures.insert(merge_from->textures.begin(),
        merge_from->textures.end(), merge_into->textures.end());
    merge_from->textures.clear();
    merge_into->materials.insert(merge_from->materials.begin(),
        merge_from->materials.end(), merge_into->materials.end());
    merge_from->materials.clear();
    merge_into->shapes.insert(merge_from->shapes.begin(),
        merge_from->shapes.end(), merge_into->shapes.end());
    merge_from->shapes.clear();
    merge_into->instances.insert(merge_from->instances.begin(),
        merge_from->instances.end(), merge_into->instances.end());
    merge_from->instances.clear();
    merge_into->environments.insert(merge_from->environments.begin(),
        merge_from->environments.end(), merge_into->environments.end());
    merge_from->environments.clear();
}

//
// Computes a shape bounding box
//
ym::bbox3f compute_bounds(const shape* shp) {
    auto bbox = ym::invalid_bbox3f;
    for (auto p : shp->pos) bbox += ym::vec3f(p);
    return bbox;
}

//
// Computes an instance bounding box
//
ym::bbox3f compute_bounds(const instance* ist) {
    return ym::transform_bbox(ist->frame, compute_bounds(ist->shp));
}

//
// Computes a scene bounding box
//
ym::bbox3f compute_bounds(const scene* scn) {
    auto bbox_shapes = std::map<shape*, ym::bbox3f>();
    for (auto shp : scn->shapes) { bbox_shapes[shp] = compute_bounds(shp); }
    auto bbox = ym::invalid_bbox3f;
    if (!scn->instances.empty()) {
        for (auto ist : scn->instances) {
            bbox += ym::transform_bbox(ist->frame, bbox_shapes[ist->shp]);
        }
    } else {
        for (auto shp : scn->shapes) { bbox += bbox_shapes[shp]; }
    }
    return bbox;
}

//
// Flatten scene instances into separate meshes.
//
void flatten_instances(scene* scn) {
    if (scn->instances.empty()) return;
    auto shapes = scn->shapes;
    scn->shapes.clear();
    auto instances = scn->instances;
    scn->instances.clear();
    for (auto ist : instances) {
        if (!ist->shp) continue;
        auto xf = ist->xform();
        auto nshp = new shape(*ist->shp);
        for (auto& p : nshp->pos) p = transform_point(xf, p);
        for (auto& n : nshp->norm) n = transform_direction(xf, n);
        scn->shapes.push_back(nshp);
    }
    for (auto e : shapes) delete e;
    for (auto e : instances) delete e;
}

//
// Initialize the lights
//
void update_lights(scene* scn, bool point_only) {
    for (auto lgt : scn->lights) delete lgt;
    scn->lights.clear();

    for (auto ist : scn->instances) {
        if (ist->shp->mat->ke == ym::zero3f) continue;
        if (point_only && ist->shp->points.empty()) continue;
        auto lgt = new light();
        lgt->ist = ist;
        if (point_only) continue;
        auto shp = ist->shp;
        if (shp->elem_cdf.empty()) {
            if (!shp->points.empty()) {
                shp->elem_cdf = ym::sample_points_cdf(shp->points.size());
            } else if (!ist->shp->lines.empty()) {
                shp->elem_cdf = ym::sample_lines_cdf(shp->lines, shp->pos);
            } else if (!shp->triangles.empty()) {
                shp->elem_cdf =
                    ym::sample_triangles_cdf(shp->triangles, shp->pos);
            }
        }
        scn->lights.push_back(lgt);
    }

    for (auto env : scn->environments) {
        if (point_only) continue;
        if (env->mat->ke == ym::zero3f) continue;
        auto lgt = new light();
        lgt->env = env;
        scn->lights.push_back(lgt);
    }
}

//
// Build a shape BVH. Public function whose interface is described above.
//
void build_bvh(shape* shp, bool equalsize) {
    if (!shp->points.empty()) {
        shp->bvh =
            ym::build_points_bvh(shp->points, shp->pos, shp->radius, equalsize);
    } else if (!shp->lines.empty()) {
        shp->bvh =
            ym::build_lines_bvh(shp->lines, shp->pos, shp->radius, equalsize);
    } else if (!shp->triangles.empty()) {
        shp->bvh = ym::build_triangles_bvh(shp->triangles, shp->pos, equalsize);
    } else if (!shp->quads.empty()) {
        shp->bvh = ym::build_quads_bvh(shp->quads, shp->pos, equalsize);
    } else {
        shp->bvh = ym::build_points_bvh(
            shp->pos.size(), shp->pos, shp->radius, equalsize);
    }
    shp->bbox = shp->bvh->nodes[0].bbox;
}

//
// Build a scene BVH. Public function whose interface is described above.
//
void build_bvh(scene* scn, bool equalsize, bool do_shapes) {
    // do shapes
    if (do_shapes) {
        for (auto shp : scn->shapes) build_bvh(shp, equalsize);
    }

    // update instance bbox
    for (auto ist : scn->instances)
        ist->bbox = ym::transform_bbox(ist->frame, ist->shp->bbox);

    // tree bvh
    scn->bvh = ym::build_bvh((int)scn->instances.size(), equalsize,
        [scn](int eid) { return scn->instances[eid]->bbox; });
}

//
// Refits a scene BVH. Public function whose interface is described above.
//
void refit_bvh(shape* shp) {
    if (!shp->points.empty()) {
        ym::refit_points_bvh(shp->bvh, shp->points, shp->pos, shp->radius);
    } else if (!shp->lines.empty()) {
        ym::refit_lines_bvh(shp->bvh, shp->lines, shp->pos, shp->radius);
    } else if (!shp->triangles.empty()) {
        ym::refit_triangles_bvh(shp->bvh, shp->triangles, shp->pos);
    } else if (!shp->quads.empty()) {
        ym::refit_quads_bvh(shp->bvh, shp->quads, shp->pos);
    } else {
        ym::refit_points_bvh(shp->bvh, shp->pos, shp->radius);
    }
    shp->bbox = shp->bvh->nodes[0].bbox;
}

//
// Refits a scene BVH. Public function whose interface is described above.
//
void refit_bvh(scene* scn, bool do_shapes) {
    if (do_shapes) {
        for (auto shp : scn->shapes) refit_bvh(shp);
    }

    // update instance bbox
    for (auto ist : scn->instances)
        ist->bbox = ym::transform_bbox(ist->frame, ist->shp->bbox);

    // recompute bvh bounds
    refit_bvh(
        scn->bvh, 0, [scn](int eid) { return scn->instances[eid]->bbox; });
}

// -----------------------------------------------------------------------------
// BVH INTERSECTION FUNCTIONS
// -----------------------------------------------------------------------------

//
// Shape intersection
//
bool intersect_ray(const shape* shp, const ym::ray3f& ray, bool early_exit,
    float& ray_t, int& eid, ym::vec4f& euv) {
    // switch over shape type
    if (!shp->triangles.empty()) {
        if (ym::intersect_triangles_bvh(shp->bvh, shp->triangles, shp->pos, ray,
                early_exit, ray_t, eid, (ym::vec3f&)euv)) {
            euv = {euv.x, euv.y, euv.z, 0};
            return true;
        }
    } else if (!shp->quads.empty()) {
        if (ym::intersect_quads_bvh(shp->bvh, shp->quads, shp->pos, ray,
                early_exit, ray_t, eid, euv)) {
            return true;
        }
    } else if (!shp->lines.empty()) {
        if (ym::intersect_lines_bvh(shp->bvh, shp->lines, shp->pos, shp->radius,
                ray, early_exit, ray_t, eid, (ym::vec2f&)euv)) {
            euv = {euv.x, euv.y, 0, 0};
            return true;
        }
    } else if (!shp->points.empty()) {
        if (ym::intersect_points_bvh(shp->bvh, shp->points, shp->pos,
                shp->radius, ray, early_exit, ray_t, eid)) {
            euv = {1, 0, 0, 0};
            return true;
        }
    } else {
        if (ym::intersect_points_bvh(
                shp->bvh, shp->pos, shp->radius, ray, early_exit, ray_t, eid)) {
            euv = {1, 0, 0, 0};
        }
        return true;
    }

    return false;
}

//
// Instance intersection
//
bool intersect_ray(const instance* ist, const ym::ray3f& ray, bool early_exit,
    float& ray_t, int& eid, ym::vec4f& euv) {
    return intersect_ray(ist->shp, ym::transform_ray_inverse(ist->frame, ray),
        early_exit, ray_t, eid, euv);
}

//
// Scene intersection
//
bool intersect_ray(const scene* scn, const ym::ray3f& ray, bool early_exit,
    float& ray_t, int& iid, int& eid, ym::vec4f& euv) {
    return ym::intersect_bvh(scn->bvh, ray, early_exit, ray_t, iid,
        [&eid, &euv, early_exit, scn](
            int iid, const ym::ray3f& ray, float& ray_t) {
            return intersect_ray(
                scn->instances[iid], ray, early_exit, ray_t, eid, euv);
        });
}

//
// Shape intersection
//
bool overlap_point(const shape* shp, const ym::vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& eid, ym::vec4f& euv) {
    // switch over shape type
    if (!shp->triangles.empty()) {
        if (ym::overlap_triangles_bvh(shp->bvh, shp->triangles, shp->pos,
                shp->radius, pos, max_dist, early_exit, dist, eid,
                (ym::vec3f&)euv)) {
            euv = {euv.x, euv.y, euv.z, 0};
            return true;
        }
    } else if (!shp->quads.empty()) {
        if (ym::overlap_quads_bvh(shp->bvh, shp->quads, shp->pos, shp->radius,
                pos, max_dist, early_exit, dist, eid, euv)) {
            return true;
        }
    } else if (!shp->lines.empty()) {
        if (ym::overlap_lines_bvh(shp->bvh, shp->lines, shp->pos, shp->radius,
                pos, max_dist, early_exit, dist, eid, (ym::vec2f&)euv)) {
            euv = {euv.x, euv.y, 0, 0};
            return true;
        }
    } else if (!shp->points.empty()) {
        if (ym::overlap_points_bvh(shp->bvh, shp->points, shp->pos, shp->radius,
                pos, max_dist, early_exit, dist, eid)) {
            euv = {1, 0, 0, 0};
            return true;
        }
    } else {
        if (ym::overlap_points_bvh(shp->bvh, shp->pos, shp->radius, pos,
                max_dist, early_exit, dist, eid)) {
            euv = {1, 0, 0, 0};
        }
        return true;
    }

    return false;
}

//
// Instance intersection
//
bool overlap_point(const instance* ist, const ym::vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& eid, ym::vec4f& euv) {
    return overlap_point(ist->shp, ym::transform_point_inverse(ist->frame, pos),
        max_dist, early_exit, dist, eid, euv);
}

//
// Scene intersection
//
bool overlap_point(const scene* scn, const ym::vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& iid, int& eid, ym::vec4f& euv) {
    return ym::overlap_bvh(scn->bvh, pos, max_dist, early_exit, dist, iid,
        [&eid, &euv, early_exit, scn](
            int iid, const ym::vec3f& pos, float max_dist, float& dist) {
            return overlap_point(
                scn->instances[iid], pos, max_dist, early_exit, dist, eid, euv);
        });
}

//
// Find the list of overlaps between shape bounds.
// Public function whose interface is described above.
//
void overlap_instance_bounds(const scene* scn1, const scene* scn2,
    bool skip_duplicates, bool skip_self, std::vector<ym::vec2i>* overlaps) {
    overlaps->clear();
    ym::overlap_bvh_elems(scn1->bvh, scn2->bvh, skip_duplicates, skip_self,
        *overlaps, [scn1, scn2](int i1, int i2) {
            return ym::overlap_bbox(
                scn1->instances[i1]->bbox, scn2->instances[i2]->bbox);
        });
}

//
// Print scene info
//
void print_info(const scene* scn) {
    auto nverts = 0, nnorms = 0, ntexcoords = 0, npoints = 0, nlines = 0,
         ntriangles = 0, nquads = 0;
    for (auto shp : scn->shapes) {
        nverts += shp->pos.size();
        nnorms += shp->norm.size();
        ntexcoords += shp->texcoord.size();
        npoints += shp->points.size();
        nlines += shp->lines.size();
        ntriangles += shp->triangles.size();
        nquads += shp->quads.size();
    }

    auto bbox = compute_bounds(scn);
    auto bboxc = ym::vec3f{(bbox.max[0] + bbox.min[0]) / 2,
        (bbox.max[1] + bbox.min[1]) / 2, (bbox.max[2] + bbox.min[2]) / 2};
    auto bboxs = ym::vec3f{bbox.max[0] - bbox.min[0], bbox.max[1] - bbox.min[1],
        bbox.max[2] - bbox.min[2]};

    printf("number of cameras:      %d\n", (int)scn->cameras.size());
    printf("number of shapes:       %d\n", (int)scn->shapes.size());
    printf("number of instances:    %d\n", (int)scn->instances.size());
    printf("number of materials:    %d\n", (int)scn->materials.size());
    printf("number of textures:     %d\n", (int)scn->textures.size());
    printf("number of environments: %d\n", (int)scn->environments.size());
    printf("number of vertices:     %d\n", nverts);
    printf("number of normals:      %d\n", nnorms);
    printf("number of texcoords:    %d\n", ntexcoords);
    printf("number of points:       %d\n", npoints);
    printf("number of lines:        %d\n", nlines);
    printf("number of triangles:    %d\n", ntriangles);
    printf("number of quads:        %d\n", nquads);
    printf("\n");
    printf("bbox min:    %g %g %g\n", bbox.min[0], bbox.min[1], bbox.min[2]);
    printf("bbox max:    %g %g %g\n", bbox.max[0], bbox.max[1], bbox.max[2]);
    printf("bbox center: %g %g %g\n", bboxc[0], bboxc[1], bboxc[2]);
    printf("bbox size:   %g %g %g\n", bboxs[0], bboxs[1], bboxs[2]);
    printf("\n");
}

}  // namespace yscn
