#ifndef MESH_HEADER
#define MESH_HEADER

#include <memory>
#include <vector>

#include "geometry/ray.h"
#include "geometry/baseObject.h"

#include "bvh/bvh.h"

enum BVHType{
  MIDPOINT,
  MEDIAN,
  SAH
};
// an inclosed set of faces

class Mesh{

public:
  Mesh(std::vector<shared_ptr<BaseObject>> faces, 
    bool is_mirror, 
    bool is_transparent,
    int leaf_threshold,
    BVHType treeType);
  ~Mesh();
  double hit(shared_ptr<BaseObject> &closest, Ray &r);
 
  public:
    shared_ptr<BVH> bbox;
    std::vector<shared_ptr<BaseObject>> faces;
    bool is_mirror;
    bool is_transparent;
    int size;
    Material m;
};

#endif