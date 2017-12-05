#include "scene.h"

#include <thread>
#include "ext/yocto_utils.h"

ray3f eval_camera(const camera* cam, const vec2f& uv)
{
    float u = uv.x;
    float v = uv.y;
    
    vec3f o = vec3f();
    vec3f x = vec3f();
    vec3f y = vec3f();
    vec3f z = vec3f();

    o = cam->frame.o;
    x = cam->frame.x;
    y = cam->frame.y * -1;
    z = cam->frame.z;

    float h = 2.0f * cam->focus * tan(cam->fovy / 2.0f);
    float w = h * cam->aspect;
    
    float focus = cam->focus;
    
    vec3f q = vec3f();
    
    q.x = o.x + (u - 0.5f) * w * x.x + (v - 0.5f) * h * y.x - focus * z.x;
    q.y = o.y + (u - 0.5f) * w * x.y + (v - 0.5f) * h * y.y - focus * z.y;
    q.z = o.z + (u - 0.5f) * w * x.z + (v - 0.5f) * h * y.z - focus * z.z;

    ray3f *r = new ray3f();
    r->o = o;
    r->d = normalize(q - o);

    return *r;
}

vec3f lookup_texture(const texture* txt, int i, int j, bool srgb)
{
    vec3f v = vec3f();
    
    float r = txt->ldr.at(i,j).x;
    float g = txt->ldr.at(i,j).y;
    float b = txt->ldr.at(i,j).z;
    
    float gamma = 2.2f;
    
    if(!srgb) gamma = 1.0f;
    
    v.x = fmin(1.0f, pow(r / 255.0f, gamma));
    v.y = fmin(1.0f, pow(g / 255.0f, gamma));
    v.z = fmin(1.0f, pow(b / 255.0f, gamma));

    return v;
}

vec3f eval_texture(const texture* txt, const vec2f& texcoord, bool srgb)
{
    float u = texcoord.x;
    float v = texcoord.y;

    float w = txt->ldr.width;
    float h = txt->ldr.height;
    
    float s = fmod(u, 1) * w;
    float t = fmod(v, 1) * h;

    int i = floor(s);
    int j = floor(t);
    
    int i1 = fmod((i + 1), w);
    int j1 = fmod((j + 1), h);
    
    float wi = s - i;
    float wj = t - j;
    
    vec3f cij = lookup_texture(txt, i, j, srgb) * (1 - wi) * (1 - wj);
    vec3f ci1j = lookup_texture(txt, i1, j, srgb) * wi * (1 - wj);
    vec3f cij1 = lookup_texture(txt, i, j1, srgb) * (1 - wi) * wj;
    vec3f ci1j1 = lookup_texture(txt, i1, j1, srgb) * wi * wj;
    
    auto c = cij + ci1j + cij1 + ci1j1;
    
    return c;
}

vec4f shade(const scene* scn, const std::vector<instance*>& lights, const vec3f& amb, const ray3f& ray)
{
    intersection3f inter = intersection3f();
    inter = intersect_first(scn, ray);

    if (!inter.hit()) return { 0.0f,0.0f,0.0f,1.0f };
    
    int ei = inter.ei;
    vec4f ew = vec4f();
    ew = inter.ew;

    vec3f n = eval_norm(inter.ist, ei, ew);
    vec3f p = eval_pos(inter.ist, ei, ew);
    vec3f c = vec3f{0.0f, 0.0f, 0.0f};

    vec3f kd = inter.ist->mat->kd;
    vec3f ks = inter.ist->mat->ks;

    texture *texkd = new texture();
    texkd = inter.ist->mat->kd_txt;
    
    texture *texks = new texture();
    texks = inter.ist->mat->ks_txt;
    
    vec2f uv = vec2f();
    uv = eval_texcoord(inter.ist->shp, ei, ew);

    vec3f la = vec3f();
    la = amb * kd;

    if(texkd)
        la = la * eval_texture(texkd, uv, true);
    
    for (auto light : lights)
    {
        vec3f ke = vec3f();
        ke = light->mat->ke;

        if ((ke.x > 0.0f && ke.y > 0.0f && ke.z > 0.0f))
        {
            vec3f l = vec3f();
            l = normalize(transform_point(light->frame, light->shp->pos.front() - p));
            float r = length(transform_point(light->frame, light->shp->pos.front() - p));
            ray3f sr = ray3f{ p, l, 0.01f, r - 0.01f };

            if(!intersect_any(scn,sr))
            {
                
                vec3f ld = vec3f();
                vec3f ls = vec3f();
                
                vec3f v = vec3f();
                vec3f h = vec3f();
                
                
                float rs = inter.ist->mat->rs;
                float ns = (rs) ? 2 / std::pow(rs, 4.0f) - 2 : 1e6f;
                
//                v = normalize(scn->cameras.front()->frame.o - p);
                v = normalize(ray.o - p);
                h = normalize(v + l);
                
                kd = inter.ist->mat->kd;
                ks = inter.ist->mat->ks;
                
                if(texkd)
                    kd = kd * eval_texture(texkd, uv, true);

                if(texks)
                    ks = ks * eval_texture(texks, uv, true);
                
                ld = kd * (ke/(r*r));
                ls = ks * (ke/(r*r));

                if(inter.ist->shp->lines.size() > 0)
                {
                    float prodnl = dot(n,l);
                    float prodnh = dot(n,h);

                    if(prodnl < 0.0f) prodnl *= -1;
                    if(prodnh < 0.0f) prodnh *= -1;

                    float sinnl = sqrt(1.0f-prodnl);
                    float sinnh = sqrt(1.0f-prodnh);
                    
                    ld = ld * sinnl;
                    ls = ls * pow(sinnh,ns);
                }
                else
                {
                    ld = ld * max(0.0f, dot(n, l));
                    ls = ls * pow(max(0.0f, dot(n, h)), ns);
                }

                c += ld + ls;
            }
        }
    }

    vec3f kr = vec3f();
    kr = inter.ist->mat->kr;
    
    if((kr.x > 0.0f || kr.y > 0.0f || kr.z > 0.0f))
    {
        vec3f v = vec3f();
        vec3f dr = vec3f();
        vec4f col = vec4f();

        v = normalize(ray.o - p);
        dr = (n * 2.0f * dot(n,v)) - v;

        ray3f newr = ray3f{p, dr, ray_eps, FLT_MAX };

        col = shade (scn, lights, amb, newr);

        c += vec3f{ (col.x * kr.x), (col.y * kr.y), (col.z * kr.z) };
    }
    
    c += la;

    vec4f L = { c.x, c.y, c.z, 1.0f };
    
    return  L;
}

image4f raytrace(const scene* scn, const vec3f& amb, int resolution, int samples)
{
    auto cam = scn->cameras.front();
    image4f img = image4f((int)std::round(cam->aspect * resolution), resolution);
    vec2f uv = vec2f();
/*
    std::vector<instance*> lights;
    for(auto istanza : scn->instances)
    {
        if( istanza->mat->ke.x > 0.0 || istanza->mat->ke.y > 0.0 || istanza->mat->ke.z > 0.0)
        {
            lights.push_back(istanza);
        }
    }
*/
    for (int i = 0; i < img.width; i++)
    {
        for (int j = 0; j < img.height; j++)
        {
            for (auto jj = 0; jj < samples; jj++)
            {
                for (auto ii = 0; ii < samples; ii++)
                {
                    uv = {
                        (i + (ii + 0.5f)/samples) / img.width,
                        (j + (jj + 0.5f)/samples) / img.height
                    };
                    auto raggio = eval_camera(cam, uv);
                    img.at(i,j) += shade(scn, scn->instances, amb, raggio);
                }
            }
            img.at(i, j) = {
                img.at(i,j).x / float(samples * samples),
                img.at(i,j).y / float(samples * samples),
                img.at(i,j).z / float(samples * samples),
                1.0f
            };
        }
    }
    
    return img;
}

int main(int argc, char** argv) {
    // command line parsing
    auto parser =
    yu::cmdline::make_parser(argc, argv, "raytrace", "raytrace scene");
    auto resolution = yu::cmdline::parse_opti(
                                              parser, "--resolution", "-r", "vertical resolution", 720);
    auto samples = yu::cmdline::parse_opti(
                                           parser, "--samples", "-s", "per-pixel samples", 1);
    auto amb = yu::cmdline::parse_optf(
                                       parser, "--ambient", "-a", "ambient color", 0.1f);
    auto imageout = yu::cmdline::parse_opts(
                                            parser, "--output", "-o", "output image", "out.png");
    auto scenein = yu::cmdline::parse_args(
                                           parser, "scenein", "input scene", "scene.obj", true);
    yu::cmdline::check_parser(parser);
    
    // load scene
    printf("loading scene %s\n", scenein.c_str());
    auto scn = load_scene(scenein);
    
    // create bvh
    printf("creating bvh\n");
    build_bvh(scn, false);
    
    // raytrace
    printf("tracing scene\n");
    auto hdr = raytrace(scn, vec3f{amb, amb, amb}, resolution, samples);
    
    // tonemap and save
    printf("saving image %s\n", imageout.c_str());
    save_hdr_or_ldr(imageout, hdr);
}
