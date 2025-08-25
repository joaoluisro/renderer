#include "bvh/mesh.h"

Mesh::Mesh(std::vector<shared_ptr<BaseObject>> faces, 
      bool is_mirror, 
      bool is_transparent,
      int threshold,
      BVHType treeType)
{
  this->is_mirror = is_mirror;
  this->is_transparent = is_transparent;
  this->size = faces.size();
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
  
}
Mesh::~Mesh()
{
}
double Mesh::hit(shared_ptr<BaseObject> &closest, Ray &r)
{
  double t_global_min = 1e+9f;
  return bbox->hit(closest, r, t_global_min);
}