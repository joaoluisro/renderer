#ifndef PARSING_H
#define PARSING_H

#include <fstream>

#include "core/scene.h"

namespace Parse{
    Scene* scene_file(const char *filename);

    vector<shared_ptr<Mesh>> obj_file(const char *filename, int threshold, BVHType treeType);
}

#endif
