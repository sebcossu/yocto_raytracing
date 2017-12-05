
#include "image.h"

// needed for the implementation
#define STB_IMAGE_IMPLEMENTATION
#include "ext/stb_image.h"

// needed for the implementation
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "ext/stb_image_write.h"

// LITC begin load_image
image4f load_image4f(const std::string& filename) {
    auto img = image4f();
    int ncomp = 0;
    auto pixels =
        stbi_loadf(filename.c_str(), &img.width, &img.height, &ncomp, 4);
    assert(ncomp == 4);
    img.pixels = std::vector<vec4f>{
        (vec4f*)pixels, (vec4f*)pixels + img.width * img.height};
    free(pixels);
    return img;
}

image4b load_image4b(const std::string& filename) {
    auto img = image4b();
    int ncomp = 0;
    auto pixels =
        stbi_load(filename.c_str(), &img.width, &img.height, &ncomp, 4);
    assert(ncomp == 4);
    img.pixels = std::vector<vec4b>{
        (vec4b*)pixels, (vec4b*)pixels + img.width * img.height};
    free(pixels);
    return img;
}
// LITC end load_image

// LITC begin save_image
void save_image(const std::string& filename, const image4f& img) {
    stbi_write_hdr(
        filename.c_str(), img.width, img.height, 4, (float*)img.pixels.data());
}

void save_image(const std::string& filename, const image4b& img) {
    stbi_write_png(filename.c_str(), img.width, img.height, 4,
        (unsigned char*)img.pixels.data(), img.width * 4);
}
// LITC end save_image

// LITC begin tonemap
inline float filmic(float h) {
    return (10.55f * h * h + 0.06 * h) / (10.21f * h * h + 1.21f * h + 0.14f);
}

image4b tonemap(
    const image4f& hdr, float exposure, bool use_filmic, bool no_srgb) {
    auto ldr = image4b(hdr.width, hdr.height);
    for (auto j = 0; j < hdr.height; j++) {
        for (auto i = 0; i < hdr.width; i++) {
            auto h = hdr.at(i, j);
            h = {h.x * powf(2, exposure), h.y * powf(2, exposure),
                h.z * powf(2, exposure), h.w};
            if (use_filmic) {
                h = {filmic(h.x), filmic(h.y), filmic(h.z), h.w};
            }
            if (!no_srgb) {
                h = {powf(h.x, 1 / 2.2f), powf(h.y, 1 / 2.2f),
                    powf(h.z, 1 / 2.2f), h.w};
            }
            ldr.at(i, j) = {(unsigned char)(clamp(h.x, 0.0f, 1.0f) * 255),
                (unsigned char)(clamp(h.y, 0.0f, 1.0f) * 255),
                (unsigned char)(clamp(h.z, 0.0f, 1.0f) * 255),
                (unsigned char)(clamp(h.w, 0.0f, 1.0f) * 255)};
        }
    }
    return ldr;
}
// LITC end tonemap

// LITC begin save_hdr_ldr
void save_hdr_or_ldr(const std::string& filename, const image4f& hdr) {
    if (filename.substr(filename.length() - 4) == ".hdr") {
        save_image(filename, hdr);
    } else {
        auto ldr = tonemap(hdr, 0, false);
        save_image(filename, ldr);
    }
}
// LITC end save_hdr_ldr
