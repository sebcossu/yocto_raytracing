#ifndef _IMAGE_H_
#define _IMAGE_H_

#include <string>
#include "vmath.h"

// LITC begin image4f
struct image4f {
    int width = 0, height = 0;
    std::vector<vec4f> pixels;

    image4f() {}
    image4f(int w, int h) : width(w), height(h), pixels(w * h, {0, 0, 0, 0}) {}

    vec4f& at(int i, int j) { return pixels[j * width + i]; }
    const vec4f& at(int i, int j) const { return pixels[j * width + i]; }
};
// LITC end image4f

// LITC begin image4b
struct image4b {
    int width = 0, height = 0;
    std::vector<vec4b> pixels;

    image4b() {}
    image4b(int w, int h) : width(w), height(h), pixels(w * h, {0, 0, 0, 0}) {}

    vec4b& at(int i, int j) { return pixels[j * width + i]; }
    const vec4b& at(int i, int j) const { return pixels[j * width + i]; }
};
// LITC end image4b

// LITC begin load_image_interface
image4f load_image4f(const std::string& filename);
image4b load_image4b(const std::string& filename);
// LITC end load_image_interface

// LITC begin save_image_interface
void save_image(const std::string& filename, const image4f& img);
void save_image(const std::string& filename, const image4b& img);
// LITC end save_image_interface

// LITC begin tonemap_interface
image4b tonemap(
    const image4f& hdr, float exposure, bool use_filmic, bool no_srgb = false);
// LITC end tonemap_interface

// LITC begin save_hdr_ldr_interface
void save_hdr_or_ldr(const std::string& filename, const image4f& hdr);
// LITC end save_hdr_ldr_interface

#endif
