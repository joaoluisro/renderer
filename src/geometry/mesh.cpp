#include "geometry/mesh.h"

Mesh::Mesh(std::vector<shared_ptr<Face>> f,
          int leaf_threshold,
          BVHType tree_type,
          Material m)
{
    this->faces = f;
    this->material = m;
    switch (tree_type)
    {
        case MIDPOINT:
            this->bbox = BvhBBox::buildMidpointBVH(f,leaf_threshold, 0);
            break;
        case MEDIAN:
            this->bbox = BvhBBox::buildMedianBVH(f,leaf_threshold);
            break;
        case SAH:
            this->bbox = BvhBBox::buildSAHBVH(f, leaf_threshold);
            break;
        default:
            this->bbox = BvhBBox::buildMidpointBVH(f,leaf_threshold,0);
            break;
    }
}

Mesh::~Mesh()
{
}

float Mesh::hit(shared_ptr<Face> &closest, const Ray &r, float min_found) const
{

  float best_t = std::numeric_limits<float>::infinity();
  return this->bbox->hit(closest, r, best_t, min_found);
}
