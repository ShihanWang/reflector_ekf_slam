#ifndef IO_SUBMAP_PAINTER_H_
#define IO_SUBMAP_PAINTER_H_

#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "io/image.h"
#include "mapping/value_conversion_tables.h"
#include "transform/rigid_transform.h"
#include "mapping/submap_2d.h"
#include "mapping/submaps.h"

namespace io {

struct PaintSubmapSlicesResult {
  PaintSubmapSlicesResult(io::UniqueCairoSurfacePtr surface,
                          Eigen::Array2f origin)
      : surface(std::move(surface)), origin(origin) {}
  io::UniqueCairoSurfacePtr surface;

  // Top left pixel of 'surface' in map frame.
  Eigen::Array2f origin;
};

struct SubmapSlice {
  SubmapSlice()
      : surface(io::MakeUniqueCairoSurfacePtr(nullptr)) {}

  // Texture data.
  int width;
  int height;
  int version;
  double resolution;
  transform::Rigid3d slice_pose;
  io::UniqueCairoSurfacePtr surface;
  // Pixel data used by 'surface'. Must outlive 'surface'.
  std::vector<uint32_t> cairo_data;

  // Metadata.
  transform::Rigid3d pose;
  int metadata_version = -1;
};

struct SubmapTexture {
  struct Pixels {
    std::vector<char> intensity;
    std::vector<char> alpha;
  };
  Pixels pixels;
  int width;
  int height;
  double resolution;
  transform::Rigid3d slice_pose;
};

struct SubmapTextures {
  int version;
  std::vector<SubmapTexture> textures;
};

PaintSubmapSlicesResult PaintSubmapSlices(
    const SubmapSlice& submaps,
    double resolution);


void FillSubmapSlice(
    const transform::Rigid3d& global_submap_pose,
    const mapping::SubmapTexture &submap_texture,
    SubmapSlice* const submap_slice,
    mapping::ValueConversionTables* conversion_tables);

// Unpacks cell data as provided by the backend into 'intensity' and 'alpha'.
SubmapTexture::Pixels UnpackTextureData(const std::string& compressed_cells,
                                        int width, int height);

// Draw a texture into a cairo surface. 'cairo_data' will store the pixel data
// for the surface and must therefore outlive the use of the surface.
UniqueCairoSurfacePtr DrawTexture(const std::vector<char>& intensity,
                                  const std::vector<char>& alpha, int width,
                                  int height,
                                  std::vector<uint32_t>* cairo_data);

}  // namespace io

#endif  // IO_SUBMAP_PAINTER_H_
