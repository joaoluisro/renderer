#ifndef PARSING_H
#define PARSING_H

#include <cstring>
#include <fstream>

#include "core/scene.h"
#include "io/wavefront_loader.h"

std::shared_ptr<Scene> parseSceneFile(const char *filename, const int width, const int height, const float fov);

#endif
