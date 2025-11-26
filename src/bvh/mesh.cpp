#include "bvh/mesh.h"

Mesh::Mesh(std::vector<shared_ptr<Face>> faces, 
      bool is_mirror, 
      bool is_transparent,
      int threshold,
      BVHType treeType,
      Material m)
{
  this->is_mirror = is_mirror;
  this->is_transparent = is_transparent;
  this->size = faces.size();
  this->faces = faces;
  switch (treeType)
  {
  case MIDPOINT:
    this->bbox = BvhBBox::buildMidpointBVH(faces,threshold, 0);
    break;
  case MEDIAN:
    this->bbox = BvhBBox::buildMedianBVH(faces,threshold);
    break;
  case SAH:
    this->bbox = BvhBBox::buildSAHBVH(faces, threshold);
    break;
  default:
    this->bbox = BvhBBox::buildMidpointBVH(faces,threshold,0);
    break;
  }
  this->material = m;
}

Mesh::~Mesh()
{
}

float Mesh::hit(shared_ptr<Face> &closest, const Ray &r, float min_found) const
{
  float best_t = 1e+9f;
  return bbox->hit(closest, r,best_t, min_found);
}
