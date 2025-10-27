#ifndef WAVEFRONT_LOADER
#define WAVEFRONT_LOADER

#include "bvh/mesh.h"


int parse_obj(const char *filename, vector<shared_ptr<Mesh>> &mesh, int threshold, BVHType treeType);

#endif
