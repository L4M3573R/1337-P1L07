#include <dlfcn.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <limits>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <raylib.h>
#include <rlgl.h>

namespace nb = nanobind;
namespace {
void *library = nullptr;
std::string backend;
template<typename T> T symbol(const char *name) {
  auto result = reinterpret_cast<T>(dlsym(library, name));
  if (!result) throw std::runtime_error(std::string("Missing active Raylib API: ") + name);
  return result;
}
struct GlyphView { nb::object owner; Font *font; };
struct RectangleView { nb::object owner; Font *font; };
int index(int i, int size) {
  if (i < 0) i += size;
  if (i < 0 || i >= size) throw nb::index_error();
  return i;
}
}
// Native twin of ModelRenderer._map_lines_to_polygons (selfdrive/ui/mici/onroad/model_renderer.py), kept in this module
// so it ships with the existing prebuilt artifact. Projects model lines (lanes, road edges, path) into screen-space
// ribbon polygons in one call, replacing ~60 small numpy operations per model update. Same steps and float32 arithmetic:
//   drop points with x < 0, offset each point by -/+ width in y and by its z offset, multiply by the 3x3 car-space
//   transform, drop pairs whose projected z is under 1e-6, divide, keep pairs whose both sides are inside the clip
//   rectangle (inclusive), and for the last line optionally keep only points that climb the screen (hill inversion).
// Output per line: float32 (2k, 2) array, left side in order then right side reversed; (0, 2) when nothing is left.
namespace geometry {
using Points = nb::ndarray<const float, nb::shape<-1, 3>, nb::device::cpu>;
using Vector = nb::ndarray<const float, nb::ndim<1>, nb::device::cpu>;
using Matrix = nb::ndarray<const float, nb::shape<3, 3>, nb::device::cpu>;
using Polygon = nb::ndarray<nb::numpy, float, nb::shape<-1, 2>>;

Polygon make_polygon(const std::vector<float> &lx, const std::vector<float> &ly,
                     const std::vector<float> &rx, const std::vector<float> &ry) {
  const size_t k = lx.size();
  float *data = new float[k ? 4 * k : 1];
  for (size_t i = 0; i < k; ++i) {
    data[2 * i] = lx[i];
    data[2 * i + 1] = ly[i];
    data[2 * (k + i)] = rx[k - 1 - i];
    data[2 * (k + i) + 1] = ry[k - 1 - i];
  }
  nb::capsule owner(data, [](void *p) noexcept { delete[] static_cast<float *>(p); });
  return Polygon(data, {2 * k, size_t(2)}, owner);
}

nb::list project_polygons(const std::vector<Points> &lines, const Vector &widths, const Vector &z_offsets,
                          const Matrix &transform, float clip_x, float clip_y, float clip_w, float clip_h,
                          bool invert_last) {
  if (widths.shape(0) != lines.size() || z_offsets.shape(0) != lines.size()) {
    throw nb::value_error("widths and z_offsets need one value per line");
  }
  float t[9];
  for (size_t r = 0; r < 3; ++r)
    for (size_t c = 0; c < 3; ++c) t[3 * r + c] = transform(r, c);
  const float x_min = clip_x, x_max = clip_x + clip_w, y_min = clip_y, y_max = clip_y + clip_h;

  nb::list out;
  std::vector<float> lx, ly, rx, ry;
  for (size_t li = 0; li < lines.size(); ++li) {
    const Points &line = lines[li];
    const float w = widths(li), zo = z_offsets(li);
    lx.clear(); ly.clear(); rx.clear(); ry.clear();
    for (size_t i = 0; i < line.shape(0); ++i) {
      const float px = line(i, 0), py = line(i, 1), pz = line(i, 2);
      if (!(px >= 0.0f)) continue;
      const float yl = py - w, yr = py + w, z = pz + zo;
      const float lX = t[0] * px + t[1] * yl + t[2] * z;
      const float lY = t[3] * px + t[4] * yl + t[5] * z;
      const float lZ = t[6] * px + t[7] * yl + t[8] * z;
      const float rX = t[0] * px + t[1] * yr + t[2] * z;
      const float rY = t[3] * px + t[4] * yr + t[5] * z;
      const float rZ = t[6] * px + t[7] * yr + t[8] * z;
      if (!(std::fabs(lZ) >= 1e-6f) || !(std::fabs(rZ) >= 1e-6f)) continue;
      const float sx_l = lX / lZ, sy_l = lY / lZ, sx_r = rX / rZ, sy_r = rY / rZ;
      if (!(sx_l >= x_min && sx_l <= x_max && sy_l >= y_min && sy_l <= y_max &&
            sx_r >= x_min && sx_r <= x_max && sy_r >= y_min && sy_r <= y_max)) continue;
      lx.push_back(sx_l); ly.push_back(sy_l); rx.push_back(sx_r); ry.push_back(sy_r);
    }
    if (invert_last && li + 1 == lines.size() && lx.size() > 1) {
      size_t keep = 0;
      float running = ly[0];
      for (size_t i = 0; i < lx.size(); ++i) {
        running = std::fmin(running, ly[i]);
        if (ly[i] == running) {
          lx[keep] = lx[i]; ly[keep] = ly[i]; rx[keep] = rx[i]; ry[keep] = ry[i];
          ++keep;
        }
      }
      lx.resize(keep); ly.resize(keep); rx.resize(keep); ry.resize(keep);
    }
    out.append(make_polygon(lx, ly, rx, ry));
  }
  return out;
}
}  // namespace geometry

NB_MODULE(_raylib_extra, module) {
  nb::class_<GlyphView>(module, "GlyphView")
    .def("__len__", [](GlyphView &v) { return v.font->glyphCount; })
    .def("__getitem__", [](GlyphView &v, int i) { return v.font->glyphs[index(i, v.font->glyphCount)]; });
  nb::class_<RectangleView>(module, "RectangleView")
    .def("__len__", [](RectangleView &v) { return v.font->glyphCount; })
    .def("__getitem__", [](RectangleView &v, int i) { return v.font->recs[index(i, v.font->glyphCount)]; });
  nb::class_<Sound>(module, "Sound").def_ro("frameCount", &Sound::frameCount);
  module.def("install", [](nb::module_ target, const std::string &path) {
    if (!backend.empty()) {
      if (backend != path) throw std::runtime_error("Raylib backend cannot change in a running renderer");
      return;
    }
    if (!nb::type<Image>().is_valid() || !nb::type<Font>().is_valid()) throw std::runtime_error("Load native Raylib types before extensions");
    auto vector = nb::borrow<nb::class_<Vector2>>(nb::type<Vector2>());
    vector.def("__init__", [](Vector2 *self, nb::sequence values) {
      if (nb::len(values) != 2) throw nb::value_error("Vector2 requires two coordinates");
      new (self) Vector2{nb::cast<float>(values[0]), nb::cast<float>(values[1])};
    }, nb::is_implicit());
    nb::implicitly_convertible<nb::tuple, Vector2>();
    nb::implicitly_convertible<nb::list, Vector2>();
    auto color_type = nb::borrow<nb::class_<Color>>(nb::type<Color>());
    color_type.def("__init__", [](Color *self, nb::sequence values) {
      if (nb::len(values) != 4) throw nb::value_error("Color requires four channels");
      new (self) Color{nb::cast<unsigned char>(values[0]), nb::cast<unsigned char>(values[1]), nb::cast<unsigned char>(values[2]), nb::cast<unsigned char>(values[3])};
    }, nb::is_implicit());
    nb::implicitly_convertible<nb::tuple, Color>();
    nb::implicitly_convertible<nb::list, Color>();
    library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(dlerror());
    if (!nb::hasattr(target, "begin_blend_mode")) target.def("begin_blend_mode", symbol<decltype(&BeginBlendMode)>("BeginBlendMode"));
    if (!nb::hasattr(target, "end_blend_mode")) target.def("end_blend_mode", symbol<decltype(&EndBlendMode)>("EndBlendMode"));
    if (!nb::hasattr(target, "color_alpha")) target.def("color_alpha", symbol<decltype(&ColorAlpha)>("ColorAlpha"));
    if (!nb::hasattr(target, "color_lerp")) target.def("color_lerp", symbol<decltype(&ColorLerp)>("ColorLerp"));
    if (!nb::hasattr(target, "color_to_int")) target.def("color_to_int", symbol<decltype(&ColorToInt)>("ColorToInt"));
    if (!nb::hasattr(target, "draw_circle_lines")) target.def("draw_circle_lines", symbol<decltype(&DrawCircleLines)>("DrawCircleLines"));
    if (!nb::hasattr(target, "draw_ellipse")) target.def("draw_ellipse", symbol<decltype(&DrawEllipse)>("DrawEllipse"));
    if (!nb::hasattr(target, "draw_ellipse_lines")) target.def("draw_ellipse_lines", symbol<decltype(&DrawEllipseLines)>("DrawEllipseLines"));
    if (!nb::hasattr(target, "draw_poly")) target.def("draw_poly", symbol<decltype(&DrawPoly)>("DrawPoly"));
    if (!nb::hasattr(target, "draw_spline_segment_bezier_cubic")) target.def("draw_spline_segment_bezier_cubic", symbol<decltype(&DrawSplineSegmentBezierCubic)>("DrawSplineSegmentBezierCubic"));
    if (!nb::hasattr(target, "draw_triangle")) target.def("draw_triangle", symbol<decltype(&DrawTriangle)>("DrawTriangle"));
    if (!nb::hasattr(target, "export_image")) target.def("export_image", symbol<decltype(&ExportImage)>("ExportImage"));
    if (!nb::hasattr(target, "get_glyph_index")) target.def("get_glyph_index", symbol<decltype(&GetGlyphIndex)>("GetGlyphIndex"));
    if (!nb::hasattr(target, "get_image_color")) target.def("get_image_color", symbol<decltype(&GetImageColor)>("GetImageColor"));
    if (!nb::hasattr(target, "get_render_height")) target.def("get_render_height", symbol<decltype(&GetRenderHeight)>("GetRenderHeight"));
    if (!nb::hasattr(target, "get_render_width")) target.def("get_render_width", symbol<decltype(&GetRenderWidth)>("GetRenderWidth"));
    if (!nb::hasattr(target, "get_screen_height")) target.def("get_screen_height", symbol<decltype(&GetScreenHeight)>("GetScreenHeight"));
    if (!nb::hasattr(target, "get_screen_width")) target.def("get_screen_width", symbol<decltype(&GetScreenWidth)>("GetScreenWidth"));
    if (!nb::hasattr(target, "is_key_pressed_repeat")) target.def("is_key_pressed_repeat", symbol<decltype(&IsKeyPressedRepeat)>("IsKeyPressedRepeat"));
    if (!nb::hasattr(target, "is_shader_valid")) target.def("is_shader_valid", symbol<decltype(&IsShaderValid)>("IsShaderValid"));
    if (!nb::hasattr(target, "is_texture_valid")) target.def("is_texture_valid", symbol<decltype(&IsTextureValid)>("IsTextureValid"));
    if (!nb::hasattr(target, "is_image_valid")) target.def("is_image_valid", symbol<decltype(&IsImageValid)>("IsImageValid"));
    if (!nb::hasattr(target, "load_font")) target.def("load_font", symbol<decltype(&LoadFont)>("LoadFont"));
    if (!nb::hasattr(target, "load_image_from_screen")) target.def("load_image_from_screen", symbol<decltype(&LoadImageFromScreen)>("LoadImageFromScreen"));
    if (!nb::hasattr(target, "load_texture")) target.def("load_texture", symbol<decltype(&LoadTexture)>("LoadTexture"));
    if (!nb::hasattr(target, "rl_disable_framebuffer")) target.def("rl_disable_framebuffer", symbol<decltype(&rlDisableFramebuffer)>("rlDisableFramebuffer"));
    if (!nb::hasattr(target, "rl_draw_render_batch_active")) target.def("rl_draw_render_batch_active", symbol<decltype(&rlDrawRenderBatchActive)>("rlDrawRenderBatchActive"));
    if (!nb::hasattr(target, "rl_get_shader_id_default")) target.def("rl_get_shader_id_default", symbol<decltype(&rlGetShaderIdDefault)>("rlGetShaderIdDefault"));
    if (!nb::hasattr(target, "rl_get_texture_id_default")) target.def("rl_get_texture_id_default", symbol<decltype(&rlGetTextureIdDefault)>("rlGetTextureIdDefault"));
    if (!nb::hasattr(target, "rl_rotatef")) target.def("rl_rotatef", symbol<decltype(&rlRotatef)>("rlRotatef"));
    if (!nb::hasattr(target, "rl_set_blend_factors")) target.def("rl_set_blend_factors", symbol<decltype(&rlSetBlendFactors)>("rlSetBlendFactors"));
    if (!nb::hasattr(target, "rl_set_blend_factors_separate")) target.def("rl_set_blend_factors_separate", symbol<decltype(&rlSetBlendFactorsSeparate)>("rlSetBlendFactorsSeparate"));
    if (!nb::hasattr(target, "rl_set_framebuffer_height")) target.def("rl_set_framebuffer_height", symbol<decltype(&rlSetFramebufferHeight)>("rlSetFramebufferHeight"));
    if (!nb::hasattr(target, "rl_set_framebuffer_width")) target.def("rl_set_framebuffer_width", symbol<decltype(&rlSetFramebufferWidth)>("rlSetFramebufferWidth"));
    if (!nb::hasattr(target, "rl_unload_texture")) target.def("rl_unload_texture", symbol<decltype(&rlUnloadTexture)>("rlUnloadTexture"));
    if (!nb::hasattr(target, "rlgl_close")) target.def("rlgl_close", symbol<decltype(&rlglClose)>("rlglClose"));
    if (!nb::hasattr(target, "rlgl_init")) target.def("rlgl_init", symbol<decltype(&rlglInit)>("rlglInit"));
    if (!nb::hasattr(target, "set_exit_key")) target.def("set_exit_key", symbol<decltype(&SetExitKey)>("SetExitKey"));
    if (!nb::hasattr(target, "set_shapes_texture")) target.def("set_shapes_texture", symbol<decltype(&SetShapesTexture)>("SetShapesTexture"));
    if (!nb::hasattr(target, "rl_set_texture")) target.def("rl_set_texture", symbol<decltype(&rlSetTexture)>("rlSetTexture"));
    if (!nb::hasattr(target, "rl_begin")) target.def("rl_begin", symbol<decltype(&rlBegin)>("rlBegin"));
    if (!nb::hasattr(target, "rl_tex_coord2f")) target.def("rl_tex_coord2f", symbol<decltype(&rlTexCoord2f)>("rlTexCoord2f"));
    if (!nb::hasattr(target, "rl_vertex2f")) target.def("rl_vertex2f", symbol<decltype(&rlVertex2f)>("rlVertex2f"));
    if (!nb::hasattr(target, "rl_end")) target.def("rl_end", symbol<decltype(&rlEnd)>("rlEnd"));
    if (!nb::hasattr(target, "init_audio_device")) target.def("init_audio_device", []() { symbol<decltype(&InitAudioDevice)>("InitAudioDevice")(); });
    if (!nb::hasattr(target, "close_audio_device")) target.def("close_audio_device", []() { symbol<decltype(&CloseAudioDevice)>("CloseAudioDevice")(); });
    if (!nb::hasattr(target, "is_audio_device_ready")) target.def("is_audio_device_ready", []() { return symbol<decltype(&IsAudioDeviceReady)>("IsAudioDeviceReady")(); });
    if (!nb::hasattr(target, "load_sound")) target.def("load_sound", [](const char *sound_path) { return symbol<decltype(&LoadSound)>("LoadSound")(sound_path); });
    if (!nb::hasattr(target, "unload_sound")) target.def("unload_sound", [](Sound sound) { symbol<decltype(&UnloadSound)>("UnloadSound")(sound); });
    if (!nb::hasattr(target, "play_sound")) target.def("play_sound", [](Sound sound) { symbol<decltype(&PlaySound)>("PlaySound")(sound); });
    if (!nb::hasattr(target, "stop_sound")) target.def("stop_sound", [](Sound sound) { symbol<decltype(&StopSound)>("StopSound")(sound); });
    if (!nb::hasattr(target, "set_sound_volume")) target.def("set_sound_volume", [](Sound sound, float volume) { symbol<decltype(&SetSoundVolume)>("SetSoundVolume")(sound, volume); });
    target.def("image_format", [](nb::handle object, int format) { if (nb::hasattr(object, "_buffer")) throw nb::value_error("Cannot mutate ownership of a Python-backed image"); auto &image = nb::cast<Image &>(object); symbol<decltype(&ImageFormat)>("ImageFormat")(&image, format); });
    target.def("image_flip_vertical", [](nb::handle object) { if (nb::hasattr(object, "_buffer")) throw nb::value_error("Cannot mutate ownership of a Python-backed image"); auto &image = nb::cast<Image &>(object); symbol<decltype(&ImageFlipVertical)>("ImageFlipVertical")(&image); });
    target.def("image_crop", [](nb::handle object, Rectangle rect) { if (nb::hasattr(object, "_buffer")) throw nb::value_error("Cannot mutate ownership of a Python-backed image"); auto &image = nb::cast<Image &>(object); symbol<decltype(&ImageCrop)>("ImageCrop")(&image, rect); });
    target.def("_image_data", [](nb::handle object) -> nb::object {
      auto &image = nb::cast<Image &>(object);
      if (!image.data) return nb::none();
      if (image.width <= 0 || image.height <= 0 ||
          size_t(image.width) * size_t(image.height) > size_t(std::numeric_limits<int>::max()) / 16 ||
          image.format < PIXELFORMAT_UNCOMPRESSED_GRAYSCALE || image.format > PIXELFORMAT_COMPRESSED_ASTC_8x8_RGBA)
        throw nb::value_error("Invalid image dimensions or pixel format");
      int size = symbol<decltype(&GetPixelDataSize)>("GetPixelDataSize")(image.width, image.height, image.format);
      if (size <= 0) throw std::runtime_error("Invalid Raylib image dimensions or format");
      if (nb::hasattr(object, "_buffer")) {
        auto view = static_cast<Py_buffer *>(nb::cast<nb::capsule>(object.attr("_buffer")).data());
        if (image.data != view->buf || size > view->len) throw nb::value_error("Image metadata exceeds its retained pixel buffer");
        if (view->readonly) {
          using ReadOnly = nb::ndarray<nb::memview, const uint8_t, nb::ndim<1>, nb::device::cpu>;
          return nb::cast(ReadOnly(image.data, {static_cast<size_t>(size)}, object));
        }
      }
      using Bytes = nb::ndarray<nb::memview, uint8_t, nb::ndim<1>, nb::device::cpu>;
      return nb::cast(Bytes(image.data, {static_cast<size_t>(size)}, object));
    });
    target.def("_font_glyphs", [](nb::handle font) { return GlyphView{nb::borrow<nb::object>(font), &nb::cast<Font &>(font)}; });
    target.def("_font_recs", [](nb::handle font) { return RectangleView{nb::borrow<nb::object>(font), &nb::cast<Font &>(font)}; });
    auto property = nb::module_::import_("builtins").attr("property");
    nb::type<Image>().attr("data") = property(target.attr("_image_data"));
    nb::type<Font>().attr("glyphs") = property(target.attr("_font_glyphs"));
    nb::type<Font>().attr("recs") = property(target.attr("_font_recs"));
    target.def("rl_load_extensions", [](uintptr_t loader) { symbol<decltype(&rlLoadExtensions)>("rlLoadExtensions")(reinterpret_cast<void *>(loader)); });
    target.def("rl_load_texture", [](nb::ndarray<const uint8_t, nb::ndim<1>, nb::c_contig> data, int w, int h, int format, int mipmaps) {
      int size = symbol<decltype(&GetPixelDataSize)>("GetPixelDataSize")(w, h, format);
      if (size <= 0 || data.size() < static_cast<size_t>(size) || mipmaps != 1) throw nb::value_error("Invalid texture byte buffer");
      return symbol<decltype(&rlLoadTexture)>("rlLoadTexture")(data.data(), w, h, format, mipmaps);
    });
    target.def("draw_triangle_strip", [](nb::handle points, int count, Color color) {
      auto draw = symbol<decltype(&DrawTriangleStrip)>("DrawTriangleStrip");
      using Vertices = nb::ndarray<float, nb::ndim<2>, nb::c_contig>;
      Vertices packed;
      if (nb::try_cast(points, packed)) {
        if (packed.shape(1) != 2 || count < 0 || static_cast<size_t>(count) > packed.shape(0)) throw nb::value_error("Invalid triangle strip bounds");
        draw(reinterpret_cast<Vector2 *>(packed.data()), count, color);
      } else {
        auto values = nb::borrow<nb::sequence>(points);
        if (count < 0 || static_cast<size_t>(count) > nb::len(values)) throw nb::value_error("Invalid triangle strip bounds");
        std::vector<Vector2> vertices;
        vertices.reserve(count);
        for (int i = 0; i < count; ++i) vertices.push_back(nb::cast<Vector2>(values[i]));
        draw(vertices.data(), count, color);
      }
    });
    nb::dict modes;
    modes["BLEND_ALPHA"] = static_cast<int>(BLEND_ALPHA);
    modes["BLEND_ADDITIVE"] = static_cast<int>(BLEND_ADDITIVE);
    modes["BLEND_MULTIPLIED"] = static_cast<int>(BLEND_MULTIPLIED);
    modes["BLEND_ADD_COLORS"] = static_cast<int>(BLEND_ADD_COLORS);
    modes["BLEND_SUBTRACT_COLORS"] = static_cast<int>(BLEND_SUBTRACT_COLORS);
    modes["BLEND_ALPHA_PREMULTIPLY"] = static_cast<int>(BLEND_ALPHA_PREMULTIPLY);
    modes["BLEND_CUSTOM"] = static_cast<int>(BLEND_CUSTOM);
    modes["BLEND_CUSTOM_SEPARATE"] = static_cast<int>(BLEND_CUSTOM_SEPARATE);
    if (!nb::hasattr(target, "BlendMode")) target.attr("BlendMode") = nb::module_::import_("enum").attr("IntEnum")("BlendMode", modes);
    target.attr("RL_QUADS") = RL_QUADS;
    target.attr("RL_ONE_MINUS_SRC_ALPHA") = RL_ONE_MINUS_SRC_ALPHA;
    target.attr("RL_FUNC_ADD") = RL_FUNC_ADD;
    target.attr("RL_SRC_ALPHA") = RL_SRC_ALPHA;
    target.attr("RL_ONE") = RL_ONE;
    backend = path;
  });
  module.def("project_polygons", &geometry::project_polygons, nb::arg("lines"), nb::arg("widths"), nb::arg("z_offsets"),
             nb::arg("transform"), nb::arg("clip_x"), nb::arg("clip_y"), nb::arg("clip_w"), nb::arg("clip_h"),
             nb::arg("invert_last"));
}
