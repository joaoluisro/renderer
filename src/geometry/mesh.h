#ifndef MESH_HEADER
#define MESH_HEADER

#include <memory>
#include <vector>

#include "geometry/ray.h"
#include "geometry/face.h"

#include "accel/bvh.h"
#include "materials/bsdf.h"

class Mesh{

    public:
      Mesh(std::vector<shared_ptr<Face>> f,
            int leaf_threshold,
            BVHType tree_type,
            Material m);

      ~Mesh();

      float hit(shared_ptr<Face> &closest, const Ray &r, float min_found) const;

    public:
        shared_ptr<BVH> bbox;
        std::vector<shared_ptr<Face>> faces;
        Material material;
};

#endif
