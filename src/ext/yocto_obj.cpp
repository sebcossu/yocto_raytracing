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
// IMPLEMENTATION FOR YOCTO_OBJ
// -----------------------------------------------------------------------------

#include "yocto_obj.h"

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#ifndef YOBJ_NO_IMAGE
#include "yocto_img.h"
#endif

namespace yobj {

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
// Splits a std::string into an array of strings on whitespace with Python split
// semantic. Modifies original std::string to avoid allocation.
//
inline int splitws(char* str, char** splits, int maxsplits) {
    int n = 0;
    while (*str && n < maxsplits) {
        if (isspace(*str)) {
            *str = 0;
        } else {
            if (n == 0 || !(*(str - 1))) {
                splits[n] = str;
                n++;
            }
        }
        str++;
    }
    splits[n] = nullptr;
    return n;
}

//
// Parses one int.
//
inline int parse_int(char** tok) { return atoi(tok[0]); }

//
// Parses one float.
//
inline float parse_float(char** tok) { return atof(tok[0]); }

//
// Parses two floats.
//
inline ym::vec2f parse_float2(char** tok) {
    return ym::vec2f{(float)atof(tok[0]), (float)atof(tok[1])};
}

//
// Parses three floats.
//
inline ym::vec3f parse_float3(char** tok) {
    return ym::vec3f{
        (float)atof(tok[0]), (float)atof(tok[1]), (float)atof(tok[2])};
}

//
// Parses four floats.
//
inline ym::vec4f parse_float4(char** tok) {
    return ym::vec4f{(float)atof(tok[0]), (float)atof(tok[1]),
        (float)atof(tok[2]), (float)atof(tok[3])};
}

//
// Parses 12 floats.
//
inline ym::frame3f parse_float12(char** tok) {
    ym::frame3f m;
    auto mm = (float*)&m;
    for (auto i = 0; i < 12; i++) mm[i] = (float)atof(tok[i]);
    return m;
}

//
// Parses 16 floats.
//
inline ym::mat4f parse_float16(char** tok) {
    ym::mat4f m;
    auto mm = (float*)&m;
    for (auto i = 0; i < 16; i++) mm[i] = (float)atof(tok[i]);
    return m;
}

//
// Parses an OBJ vertex list. Handles negative values.
//
inline void parse_vertlist(char** tok, int ntoks, std::vector<vertex>& elems,
    const vertex& vert_size) {
    elems.clear();
    for (auto i = 0; i < ntoks; i++) {
        // parse triplet
        char* splits[] = {tok[i], 0, 0, 0, 0};
        auto ns = 1;
        while (*tok[i]) {
            if (*tok[i] == '/') {
                *tok[i] = 0;
                if (ns < 5) splits[ns++] = tok[i] + 1;
            }
            tok[i]++;
        }
        auto v = vertex{-1, -1, -1, -1, -1};
        auto v_ptr = &v.pos;
        auto vs_ptr = &vert_size.pos;
        for (auto i = 0; i < 5; i++) {
            if (!splits[i]) {
                v_ptr[i] = -1;
                continue;
            }
            v_ptr[i] = (int)atoi(splits[i]);
            v_ptr[i] = (v_ptr[i] < 0) ? vs_ptr[i] + v_ptr[i] : v_ptr[i] - 1;
        }
        elems.push_back(v);
    }
}

//
// Parse texture options and name
//
void parse_texture(char** toks, int ntoks, std::string& path,
    property_map<std::string>& info, std::vector<std::string>& textures,
    std::unordered_set<std::string>& texture_set) {
    // texture name
    if (ntoks > 0) {
        path = toks[ntoks - 1];
        for (auto& c : path)
            if (c == '\\') c = '/';
    }

    // texture options
    if (ntoks > 1) {
        auto cur_ntoks = ntoks - 1;
        auto cur_tok = toks;
        while (cur_ntoks) {
            if (cur_tok[0][0] != '-') break;
            auto name = cur_tok[0];
            info[name] = {};
            cur_ntoks--;
            cur_tok++;
            while (cur_ntoks && cur_tok[0][0] != '-') {
                info[name].push_back(cur_tok[0]);
                cur_ntoks--;
                cur_tok++;
            }
        }
    }

    // insert texture
    if (!path.empty() && texture_set.find(path) == texture_set.end()) {
        textures.push_back(path);
        texture_set.insert(path);
    }
}

//
// Load MTL
//
std::vector<material> load_mtl(const std::string& filename, bool flip_tr,
    std::vector<std::string>& textures) {
    // clear materials
    auto materials = std::vector<material>();

    // clear textures
    textures.clear();
    auto texture_set = std::unordered_set<std::string>();

    // open file
    auto file = fopen(filename.c_str(), "rt");
    if (!file) throw std::runtime_error("cannot open filename " + filename);

    // add a material preemptively to avoid crashes
    materials.emplace_back();

    // read the file line by line
    char line[4096];
    char* toks[1024];
    auto linenum = 0;
    while (fgets(line, 4096, file)) {
        linenum += 1;
        int ntok = splitws(line, toks, 1024);

        // skip empty and comments
        if (!ntok) continue;
        if (toks[0][0] == '#') continue;

        // set up code
        auto tok_s = std::string(toks[0]);
        auto cur_tok = toks + 1;
        auto cur_ntok = ntok - 1;

        // possible token values
        if (tok_s == "newmtl") {
            materials.emplace_back();
            materials.back().name = (cur_ntok) ? cur_tok[0] : "";
        } else if (tok_s == "illum") {
            materials.back().illum = parse_int(cur_tok);
        } else if (tok_s == "Ke") {
            materials.back().ke = parse_float3(cur_tok);
        } else if (tok_s == "Ka") {
            materials.back().ka = parse_float3(cur_tok);
        } else if (tok_s == "Kd") {
            materials.back().kd = parse_float3(cur_tok);
        } else if (tok_s == "Ks") {
            materials.back().ks = parse_float3(cur_tok);
        } else if (tok_s == "Kr") {
            materials.back().kr = parse_float3(cur_tok);
        } else if (tok_s == "Kt" || tok_s == "Tf") {
            if (cur_ntok >= 3) {
                materials.back().kt = parse_float3(cur_tok);
            } else {
                auto v = parse_float(cur_tok);
                materials.back().kt = {v, v, v};
            }
        } else if (tok_s == "Tr") {
            if (cur_ntok >= 3) {
                materials.back().kt = parse_float3(cur_tok);
            } else {
                // as tinyobjreader
                if (flip_tr)
                    materials.back().op = 1 - parse_float(cur_tok);
                else
                    materials.back().op = parse_float(cur_tok);
            }
        } else if (tok_s == "Ns") {
            materials.back().ns = parse_float(cur_tok);
        } else if (tok_s == "d") {
            materials.back().op = parse_float(cur_tok);
        } else if (tok_s == "Ni") {
            materials.back().ior = parse_float(cur_tok);
        } else if (tok_s == "map_Ke") {
            parse_texture(cur_tok, cur_ntok, materials.back().ke_txt,
                materials.back().ke_txt_info, textures, texture_set);
        } else if (tok_s == "map_Ka") {
            parse_texture(cur_tok, cur_ntok, materials.back().ka_txt,
                materials.back().ka_txt_info, textures, texture_set);
        } else if (tok_s == "map_Kd") {
            parse_texture(cur_tok, cur_ntok, materials.back().kd_txt,
                materials.back().kd_txt_info, textures, texture_set);
        } else if (tok_s == "map_Ks") {
            parse_texture(cur_tok, cur_ntok, materials.back().ks_txt,
                materials.back().ks_txt_info, textures, texture_set);
        } else if (tok_s == "map_Kr") {
            parse_texture(cur_tok, cur_ntok, materials.back().kr_txt,
                materials.back().kr_txt_info, textures, texture_set);
        } else if (tok_s == "map_Tr") {
            parse_texture(cur_tok, cur_ntok, materials.back().kt_txt,
                materials.back().kt_txt_info, textures, texture_set);
        } else if (tok_s == "map_Ns") {
            parse_texture(cur_tok, cur_ntok, materials.back().ns_txt,
                materials.back().ns_txt_info, textures, texture_set);
        } else if (tok_s == "map_d") {
            parse_texture(cur_tok, cur_ntok, materials.back().op_txt,
                materials.back().op_txt_info, textures, texture_set);
        } else if (tok_s == "map_Ni") {
            parse_texture(cur_tok, cur_ntok, materials.back().ior_txt,
                materials.back().ior_txt_info, textures, texture_set);
        } else if (tok_s == "map_bump" || tok_s == "bump") {
            parse_texture(cur_tok, cur_ntok, materials.back().bump_txt,
                materials.back().bump_txt_info, textures, texture_set);
        } else if (tok_s == "map_disp" || tok_s == "disp") {
            parse_texture(cur_tok, cur_ntok, materials.back().disp_txt,
                materials.back().disp_txt_info, textures, texture_set);
        } else if (tok_s == "map_norm" || tok_s == "norm") {
            parse_texture(cur_tok, cur_ntok, materials.back().norm_txt,
                materials.back().norm_txt_info, textures, texture_set);
        } else {
            // copy into strings
            for (auto i = 0; i < cur_ntok; i++)
                materials.back().unknown_props[tok_s].push_back(cur_tok[i]);
        }
    }

    // remove first fake material
    materials.erase(materials.begin());

    // done
    return materials;
}

//
// Loads textures for an scene.
//
void load_textures(
    scene* asset, const std::string& dirname, bool skip_missing) {
#ifndef YOBJ_NO_IMAGE
    for (auto& txt : asset->textures) {
        auto filename = dirname + txt.path;
        for (auto& c : filename)
            if (c == '\\') c = '/';
        if (yimg::is_hdr_filename(filename)) {
            txt.dataf =
                yimg::load_imagef(filename, txt.width, txt.height, txt.ncomp);
        } else {
            txt.datab =
                yimg::load_image(filename, txt.width, txt.height, txt.ncomp);
        }
        if (txt.datab.empty() && txt.dataf.empty()) {
            if (skip_missing) continue;
            throw std::runtime_error("cannot laod image " + filename);
        }
    }
#endif
}

//
// Loads an OBJ
//
scene* load_obj(const std::string& filename, bool load_txt, bool skip_missing,
    bool flip_texcoord, bool flip_tr) {
    // clear obj
    auto asset = std::unique_ptr<scene>(new scene());

    // open file
    auto file = fopen(filename.c_str(), "rt");
    if (!file) throw std::runtime_error("cannot open filename " + filename);

    // initializing obj
    asset->objects.push_back({});
    asset->objects.back().groups.push_back({});

    // allocate buffers to avoid re-allocing
    auto cur_elems = std::vector<vertex>();
    auto cur_matname = std::string();
    auto cur_mtllibs = std::vector<std::string>();

    // keep track of array lengths
    auto vert_size = vertex{0, 0, 0, 0, 0};

    // read the file line by line
    char line[4096];
    char* toks[1024];
    auto linenum = 0;
    while (fgets(line, 4096, file)) {
        linenum += 1;
        int ntok = splitws(line, toks, 1024);

        // skip empty and comments
        if (!ntok) continue;
        if (toks[0][0] == '#') continue;

        // set up code
        auto tok_s = std::string(toks[0]);
        auto cur_tok = toks + 1;
        auto cur_ntok = ntok - 1;

        // possible token values
        if (tok_s == "v") {
            vert_size.pos += 1;
            asset->pos.push_back(parse_float3(cur_tok));
        } else if (tok_s == "vn") {
            vert_size.norm += 1;
            asset->norm.push_back(parse_float3(cur_tok));
        } else if (tok_s == "vt") {
            vert_size.texcoord += 1;
            asset->texcoord.push_back(parse_float2(cur_tok));
            if (flip_texcoord)
                asset->texcoord.back()[1] = 1 - asset->texcoord.back()[1];
        } else if (tok_s == "vc") {
            vert_size.color += 1;
            asset->color.push_back(parse_float4(cur_tok));
        } else if (tok_s == "vr") {
            vert_size.radius += 1;
            asset->radius.push_back(parse_float(cur_tok));
        } else if (tok_s == "f") {
            parse_vertlist(cur_tok, cur_ntok, cur_elems, vert_size);
            auto& g = asset->objects.back().groups.back();
            g.elems.push_back({(uint32_t)g.verts.size(), element_type::face,
                (uint16_t)cur_elems.size()});
            g.verts.insert(g.verts.end(), cur_elems.begin(), cur_elems.end());
        } else if (tok_s == "l") {
            parse_vertlist(cur_tok, cur_ntok, cur_elems, vert_size);
            auto& g = asset->objects.back().groups.back();
            g.elems.push_back({(uint32_t)g.verts.size(), element_type::line,
                (uint16_t)cur_elems.size()});
            g.verts.insert(g.verts.end(), cur_elems.begin(), cur_elems.end());
        } else if (tok_s == "p") {
            parse_vertlist(cur_tok, cur_ntok, cur_elems, vert_size);
            auto& g = asset->objects.back().groups.back();
            g.elems.push_back({(uint32_t)g.verts.size(), element_type::point,
                (uint16_t)cur_elems.size()});
            g.verts.insert(g.verts.end(), cur_elems.begin(), cur_elems.end());
        } else if (tok_s == "t") {
            parse_vertlist(cur_tok, cur_ntok, cur_elems, vert_size);
            auto& g = asset->objects.back().groups.back();
            g.elems.push_back({(uint32_t)g.verts.size(), element_type::tetra,
                (uint16_t)cur_elems.size()});
            g.verts.insert(g.verts.end(), cur_elems.begin(), cur_elems.end());
        } else if (tok_s == "o") {
            auto name = (cur_ntok) ? cur_tok[0] : "";
            asset->objects.push_back({name, {}});
            asset->objects.back().groups.push_back({cur_matname, ""});
        } else if (tok_s == "usemtl") {
            auto name = (cur_ntok) ? cur_tok[0] : "";
            cur_matname = name;
            asset->objects.back().groups.push_back({cur_matname, ""});
        } else if (tok_s == "g") {
            auto name = (cur_ntok) ? cur_tok[0] : "";
            asset->objects.back().groups.push_back({cur_matname, name});
        } else if (tok_s == "s") {
            auto name = (cur_ntok) ? cur_tok[0] : "";
            auto smoothing = name == std::string("on");
            if (asset->objects.back().groups.back().smoothing != smoothing) {
                asset->objects.back().groups.push_back(
                    {cur_matname, name, smoothing});
            }
        } else if (tok_s == "mtllib") {
            auto name = (cur_ntok) ? cur_tok[0] : "";
            if (name != std::string("")) {
                auto found = false;
                for (auto lib : cur_mtllibs) {
                    if (lib == name) {
                        found = true;
                        break;
                    }
                }
                if (!found) cur_mtllibs.push_back(name);
            }
        } else if (tok_s == "c") {
            asset->cameras.emplace_back();
            auto& cam = asset->cameras.back();
            cam.name = (cur_ntok) ? cur_tok[0] : "";
            cam.ortho = parse_int(cur_tok + 1);
            cam.yfov = parse_float(cur_tok + 2);
            cam.aspect = parse_float(cur_tok + 3);
            cam.aperture = parse_float(cur_tok + 4);
            cam.focus = parse_float(cur_tok + 5);
            cam.frame = parse_float12(cur_tok + 6);
        } else if (tok_s == "e") {
            asset->environments.emplace_back();
            auto& env = asset->environments.back();
            env.name = (cur_ntok) ? cur_tok[0] : "<unnamed>";
            env.matname = (cur_ntok - 1) ? cur_tok[1] : "<unnamed_material>";
            env.frame = parse_float12(cur_tok + 2);
        } else if (tok_s == "i") {
            asset->instances.emplace_back();
            auto& ist = asset->instances.back();
            ist.name = (cur_ntok) ? cur_tok[0] : "<unnamed>";
            ist.objname = (cur_ntok - 1) ? cur_tok[1] : "<unnamed_mesh>";
            ist.frame = parse_float12(cur_tok + 2);
        } else {
            // unused
        }
    }

    // cleanup unused
    for (auto&& o : asset->objects) {
        auto end = std::remove_if(o.groups.begin(), o.groups.end(),
            [](const group& x) { return x.verts.empty(); });
        o.groups.erase(end, o.groups.end());
    }
    auto end = std::remove_if(asset->objects.begin(), asset->objects.end(),
        [](const object& x) { return x.groups.empty(); });
    asset->objects.erase(end, asset->objects.end());

    // parse materials
    auto dirname = get_dirname(filename);
    std::unordered_set<std::string> texture_set;
    for (auto mtllib : cur_mtllibs) {
        auto mtlname = dirname + mtllib;
        std::vector<std::string> textures;
        auto materials = load_mtl(mtlname, flip_tr, textures);
        asset->materials.insert(
            asset->materials.end(), materials.begin(), materials.end());
        for (auto& txt : textures) {
            if (texture_set.find(txt) != texture_set.end()) continue;
            asset->textures.push_back({txt});
            texture_set.insert(txt);
        }
    }

    // load textures
    if (load_txt) load_textures(asset.get(), dirname, skip_missing);

    // done
    return asset.release();
}

//
// write one float prepended by a std::string
//
inline void fwrite_float(
    FILE* file, const char* str, float v, bool newline = true) {
    fprintf(file, "%s %.6g", str, v);
    if (newline) fprintf(file, "\n");
}

//
// write one float prepended by a std::string
//
inline void fwrite_int(
    FILE* file, const char* str, int v, bool newline = true) {
    fprintf(file, "%s %d", str, v);
    if (newline) fprintf(file, "\n");
}

//
// write two floats prepended by a std::string
//
inline void fwrite_float2(
    FILE* file, const char* str, const ym::vec2f& v, bool newline = true) {
    fprintf(file, "%s %.6g %.6g", str, v[0], v[1]);
    if (newline) fprintf(file, "\n");
}

//
// write three floats prepended by a std::string
//
inline void fwrite_float3(
    FILE* file, const char* str, const ym::vec3f& v, bool newline = true) {
    fprintf(file, "%s %.6g %.6g %.6g", str, v[0], v[1], v[2]);
    if (newline) fprintf(file, "\n");
}

//
// write four floats prepended by a std::string
//
inline void fwrite_float4(
    FILE* file, const char* str, const ym::vec4f& v, bool newline = true) {
    fprintf(file, "%s %.6g %.6g %.6g %.6g", str, v[0], v[1], v[2], v[3]);
    if (newline) fprintf(file, "\n");
}

//
// write 16 floats prepended by a std::string
//
inline void fwrite_float12(
    FILE* file, const char* str, const ym::frame3f& v, bool newline = true) {
    const float* vf = (float*)&v;
    fprintf(file, "%s", str);
    for (int i = 0; i < 12; i++) fprintf(file, " %.6g", vf[i]);
    if (newline) fprintf(file, "\n");
}

//
// write 16 floats prepended by a std::string
//
inline void fwrite_float16(
    FILE* file, const char* str, const ym::mat4f& v, bool newline = true) {
    const float* vf = (float*)&v;
    fprintf(file, "%s", str);
    for (int i = 0; i < 16; i++) fprintf(file, " %.6g", vf[i]);
    if (newline) fprintf(file, "\n");
}

//
// write a std::string prepended by another if the std::string is not NULL
//
inline void fwrite_str(FILE* file, const char* str, const std::string& s,
    bool force = false, bool newline = true) {
    if (s.empty() && !force) return;
    fprintf(file, "%s %s", str, s.c_str());
    if (newline) fprintf(file, "\n");
}

//
// write a std::string prepended by another if the std::string is not NULL
//
inline void fwrite_str_props(FILE* file, const char* str, const std::string& s,
    const property_map<std::string>& props, bool force = false,
    bool newline = true) {
    if (s.empty() && !force) return;
    if (props.empty()) {
        fprintf(file, "%s %s", str, s.c_str());
    } else {
        auto props_str = std::string();
        for (auto&& prop : props) {
            props_str += prop.first + " ";
            for (auto&& pp : prop.second) props_str += pp + " ";
        }
        fprintf(file, "%s %s %s", str, props_str.c_str(), s.c_str());
    }
    if (newline) fprintf(file, "\n");
}

//
// write one float prepended by a std::string
//
inline void fwrite_float_props(
    FILE* file, const char* str, float v, float def = 0, bool newline = true) {
    if (v == def) return;
    fprintf(file, "%s %.6g", str, v);
    if (newline) fprintf(file, "\n");
}

//
// write three floats prepended by a std::string
//
inline void fwrite_float3_props(FILE* file, const char* str, const ym::vec3f& v,
    const ym::vec3f& def = {0, 0, 0}, bool newline = true) {
    if (v == def) return;
    fprintf(file, "%s %.6g %.6g %.6g", str, v[0], v[1], v[2]);
    if (newline) fprintf(file, "\n");
}

//
// write an OBJ vertex triplet using only the indices that are active
//
inline void fwrite_objverts(FILE* file, const char* str, int nv,
    const vertex* verts, bool newline = true) {
    fprintf(file, "%s", str);
    for (auto v = 0; v < nv; v++) {
        auto vert = verts[v];
        auto vert_ptr = &vert.pos;
        auto nto_write = 0;
        for (auto i = 0; i < 5; i++) {
            if (vert_ptr[i] >= 0) nto_write = i + 1;
        }
        for (auto i = 0; i < nto_write; i++) {
            if (vert_ptr[i] >= 0) {
                fprintf(file, "%c%d", ((i == 0) ? ' ' : '/'), vert_ptr[i] + 1);
            } else {
                fprintf(file, "%c", '/');
            }
        }
    }
    fprintf(file, "\n");
}

//
// Save an MTL file
//
void save_mtl(const std::string& filename,
    const std::vector<material>& materials, bool flip_tr) {
    auto file = fopen(filename.c_str(), "wt");
    if (!file) throw std::runtime_error("could not open filename " + filename);

    // for each material, dump all the values
    for (auto& mat : materials) {
        fwrite_str(file, "newmtl", mat.name, true);
        fwrite_int(file, "  illum", mat.illum);
        fwrite_float3_props(file, "  Ke", mat.ke);
        fwrite_float3_props(file, "  Ka", mat.ka);
        fwrite_float3_props(file, "  Kd", mat.kd);
        fwrite_float3_props(file, "  Ks", mat.ks);
        fwrite_float3_props(file, "  Kr", mat.kr);
        fwrite_float3_props(file, "  Tf", mat.kt);
        fwrite_float_props(file, "  Ns", mat.ns, 0);
        fwrite_float_props(file, "  d", mat.op, 1);
        fwrite_float_props(file, "  Ni", mat.ior, 1);
        fwrite_str_props(file, "  map_Ke", mat.ke_txt, mat.ke_txt_info);
        fwrite_str_props(file, "  map_Ka", mat.ka_txt, mat.ka_txt_info);
        fwrite_str_props(file, "  map_Kd", mat.kd_txt, mat.kd_txt_info);
        fwrite_str_props(file, "  map_Ks", mat.ks_txt, mat.ks_txt_info);
        fwrite_str_props(file, "  map_Kr", mat.kr_txt, mat.kr_txt_info);
        fwrite_str_props(file, "  map_Kt", mat.kt_txt, mat.kt_txt_info);
        fwrite_str_props(file, "  map_Ns", mat.ns_txt, mat.ns_txt_info);
        fwrite_str_props(file, "  map_d", mat.op_txt, mat.op_txt_info);
        fwrite_str_props(file, "  map_Ni", mat.ior_txt, mat.ior_txt_info);
        fwrite_str_props(file, "  map_bump", mat.bump_txt, mat.bump_txt_info);
        fwrite_str_props(file, "  map_disp", mat.disp_txt, mat.disp_txt_info);
        fwrite_str_props(file, "  map_norm", mat.norm_txt, mat.norm_txt_info);
        for (auto&& p : mat.unknown_props) {
            auto s = std::string();
            for (auto&& v : p.second) s += v + " ";
            fwrite_str(file, p.first.c_str(), s.c_str());
        }
        fprintf(file, "\n");
    }

    fclose(file);
}

//
// Loads textures for an scene.
//
void save_textures(
    const scene* asset, const std::string& dirname, bool skip_missing) {
#ifndef YOBJ_NO_IMAGE
    for (auto& txt : asset->textures) {
        if (txt.datab.empty() && txt.dataf.empty()) continue;
        auto filename = dirname + txt.path;
        for (auto& c : filename)
            if (c == '\\') c = '/';
        auto ok = true;
        if (!txt.datab.empty()) {
            ok = yimg::save_image(
                filename, txt.width, txt.height, txt.ncomp, txt.datab.data());
        }
        if (!txt.dataf.empty()) {
            ok = yimg::save_imagef(
                filename, txt.width, txt.height, txt.ncomp, txt.dataf.data());
        }
        if (!ok) {
            if (skip_missing) continue;
            throw std::runtime_error("cannot save image " + filename);
        }
    }
#endif
}

//
// Save an OBJ
//
void save_obj(const std::string& filename, const scene* asset, bool save_txt,
    bool skip_missing, bool flip_texcoord, bool flip_tr) {
    // open file
    auto file = fopen(filename.c_str(), "wt");
    if (!file) throw std::runtime_error("could not open filename " + filename);

    // linkup to mtl
    auto dirname = get_dirname(filename);
    auto basename = filename.substr(dirname.length());
    basename = basename.substr(0, basename.length() - 4);
    if (!asset->materials.empty()) {
        fwrite_str(file, "mtllib", basename + ".mtl");
    }

    // save cameras
    for (auto& cam : asset->cameras) {
        fwrite_str(file, "c", cam.name, true, false);
        fwrite_int(file, " ", cam.ortho, false);
        fwrite_float(file, " ", cam.yfov, false);
        fwrite_float(file, " ", cam.aspect, false);
        fwrite_float(file, " ", cam.aperture, false);
        fwrite_float(file, " ", cam.focus, false);
        fwrite_float12(file, " ", cam.frame, true);
    }

    // save envs
    for (auto& env : asset->environments) {
        fwrite_str(file, "e", env.name, true, false);
        fwrite_str(file, " ", env.matname, true, false);
        fwrite_float12(file, " ", env.frame, true);
    }

    // save instances
    for (auto& ist : asset->instances) {
        fwrite_str(file, "i", ist.name, true, false);
        fwrite_str(file, " ", ist.objname, true, false);
        fwrite_float12(file, " ", ist.frame, true);
    }

    // save all vertex data
    for (auto& v : asset->pos) fwrite_float3(file, "v", v);
    if (flip_texcoord) {
        for (auto& v : asset->texcoord)
            fwrite_float2(file, "vt", {v[0], 1 - v[1]});
    } else {
        for (auto& v : asset->texcoord) fwrite_float2(file, "vt", v);
    }
    for (auto& v : asset->norm) fwrite_float3(file, "vn", v);
    for (auto& v : asset->color) fwrite_float4(file, "vc", v);
    for (auto& v : asset->radius) fwrite_float(file, "vr", v);

    // save element data
    const char* elem_labels[] = {"", "p", "l", "f", "t"};
    for (auto& object : asset->objects) {
        fwrite_str(file, "o", object.name, true);
        for (auto& group : object.groups) {
            fwrite_str(file, "usemtl", group.matname);
            fwrite_str(file, "g", group.groupname);
            if (!group.smoothing) fwrite_str(file, "s", "off");
            for (auto elem : group.elems) {
                fwrite_objverts(file, elem_labels[(int)elem.type], elem.size,
                    group.verts.data() + elem.start);
            }
        }
    }

    fclose(file);

    // save materials
    if (!asset->materials.empty())
        save_mtl(dirname + basename + ".mtl", asset->materials, flip_tr);

    // save textures
    if (save_txt) save_textures(asset, dirname, skip_missing);
}

//
//
// A hash function for vecs
//
struct vertex_hash {
    std::hash<int> Th;
    size_t operator()(const vertex& vv) const {
        auto v = (const int*)&vv;
        size_t h = 0;
        for (auto i = 0; i < sizeof(vertex) / sizeof(int); i++) {
            // embads hash_combine below
            h ^= (Th(v[i]) + 0x9e3779b9 + (h << 6) + (h >> 2));
        }
        return h;
    }
};
// Flattens an scene
//
mesh* get_mesh(
    const scene* model, const object& oshape, bool facet_non_smooth) {
    // convert meshes
    auto msh = new mesh();
    msh->name = oshape.name;
    for (auto& group : oshape.groups) {
        if (group.verts.empty()) continue;
        if (group.elems.empty()) continue;
        msh->shapes.emplace_back();
        auto prim = &msh->shapes.back();
        prim->name = group.groupname;
        prim->matname = group.matname;

        // insert all vertices
        std::unordered_map<vertex, int, vertex_hash> vert_map;
        std::vector<int> vert_ids;
        // vert_map.clear();
        // vert_ids.clear();
        for (auto& vert : group.verts) {
            if (vert_map.find(vert) == vert_map.end()) {
                // split in two to avoid undefined behaviour
                auto size = (int)vert_map.size();
                vert_map[vert] = size;
            }
            vert_ids.push_back(vert_map.at(vert));
        }

        // convert elements
        for (auto& elem : group.elems) {
            switch (elem.type) {
                case element_type::point: {
                    for (auto i = elem.start; i < elem.start + elem.size; i++) {
                        prim->points.push_back(vert_ids[i]);
                    }
                } break;
                case element_type::line: {
                    for (auto i = elem.start; i < elem.start + elem.size - 1;
                         i++) {
                        prim->lines.push_back({vert_ids[i], vert_ids[i + 1]});
                    }
                } break;
                case element_type::face: {
                    for (auto i = elem.start + 2; i < elem.start + elem.size;
                         i++) {
                        prim->triangles.push_back({vert_ids[elem.start],
                            vert_ids[i - 1], vert_ids[i]});
                    }
                } break;
                case element_type::tetra: {
                    for (auto i = elem.start; i < elem.start + elem.size;
                         i += 4) {
                        if (i + 3 >= vert_ids.size()) continue;
                        prim->tetras.push_back({vert_ids[i], vert_ids[i + 1],
                            vert_ids[i + 2], vert_ids[i + 3]});
                    }
                } break;
                default: { assert(false); }
            }
        }

        // check for errors
        // copy vertex data
        auto v = group.verts[0];
        if (v.pos >= 0) prim->pos.resize(vert_map.size());
        if (v.texcoord >= 0) prim->texcoord.resize(vert_map.size());
        if (v.norm >= 0) prim->norm.resize(vert_map.size());
        if (v.color >= 0) prim->color.resize(vert_map.size());
        if (v.radius >= 0) prim->radius.resize(vert_map.size());
        for (auto& kv : vert_map) {
            if (v.pos >= 0 && kv.first.pos >= 0) {
                prim->pos[kv.second] = model->pos[kv.first.pos];
            }
            if (v.texcoord >= 0 && kv.first.texcoord >= 0) {
                prim->texcoord[kv.second] = model->texcoord[kv.first.texcoord];
            }
            if (v.norm >= 0 && kv.first.norm >= 0) {
                prim->norm[kv.second] = model->norm[kv.first.norm];
            }
            if (v.color >= 0 && kv.first.color >= 0) {
                prim->color[kv.second] = model->color[kv.first.color];
            }
            if (v.radius >= 0 && kv.first.radius >= 0) {
                prim->radius[kv.second] = model->radius[kv.first.radius];
            }
        }

        // fix smoothing
        if (!group.smoothing && facet_non_smooth) {
            auto faceted_ = shape();
            auto faceted = &faceted_;
            faceted->name = prim->name;
            faceted->matname = prim->matname;
            auto pidx = std::vector<int>();
            for (auto point : prim->points) {
                faceted->points.push_back((int)pidx.size());
                pidx.push_back(point);
            }
            for (auto line : prim->lines) {
                faceted->lines.push_back(
                    {(int)pidx.size() + 0, (int)pidx.size() + 1});
                pidx.push_back(line.x);
                pidx.push_back(line.y);
            }
            for (auto triangle : prim->triangles) {
                faceted->triangles.push_back({(int)pidx.size() + 0,
                    (int)pidx.size() + 1, (int)pidx.size() + 2});
                pidx.push_back(triangle.x);
                pidx.push_back(triangle.y);
                pidx.push_back(triangle.z);
            }
            for (auto tetra : prim->tetras) {
                faceted->tetras.push_back(
                    {(int)pidx.size() + 0, (int)pidx.size() + 1,
                        (int)pidx.size() + 2, (int)pidx.size() + 3});
                pidx.push_back(tetra.x);
                pidx.push_back(tetra.y);
                pidx.push_back(tetra.z);
                pidx.push_back(tetra.w);
            }
            for (auto idx : pidx) {
                if (!prim->pos.empty()) faceted->pos.push_back(prim->pos[idx]);
                if (!prim->norm.empty())
                    faceted->norm.push_back(prim->norm[idx]);
                if (!prim->texcoord.empty())
                    faceted->texcoord.push_back(prim->texcoord[idx]);
                if (!prim->color.empty())
                    faceted->color.push_back(prim->color[idx]);
                if (!prim->radius.empty())
                    faceted->radius.push_back(prim->radius[idx]);
            }
            *prim = *faceted;
        }
    }

    // done
    return msh;
}

}  // namespace yobj
