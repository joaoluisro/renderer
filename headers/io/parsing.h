#ifndef PARSING_H
#define PARSING_H

#include <cstring>
#include <fstream>

#include "core/scene.h"
#include "io/wavefront_loader.h"

namespace Parse{
    Scene* scene_file(const char *filename, const int width, const int height, const float fov);
}

#endif
