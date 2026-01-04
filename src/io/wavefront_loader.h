#ifndef WAVEFRONT_LOADER
#define WAVEFRONT_LOADER

#include "core/definitions.h"

#include <vector>
#include <memory>
#include <filesystem>

#include "geometry/mesh.h"
#include "geometry/vec3.h"
#include "geometry/triangle.h"

#include "core/spectrum.h"
#include "materials/bsdf.h"

int parseWavefrontFile(const char *filename, vector<shared_ptr<Mesh>> &mesh, int threshold, BVHType treeType);

#endif
