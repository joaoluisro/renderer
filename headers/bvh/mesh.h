#ifndef MESH_HEADER
#define MESH_HEADER

#include <memory>
#include <vector>

#include "math/ray.h"
#include "math/face.h"

#include "bvh/bvh.h"

enum BVHType{
  MIDPOINT,
  MEDIAN,
  SAH
};

enum IllumType{
    OPAQUE = 0,
    SPECULAR = 1,
    TRANSPARENT = 2,
    MIRROR = 3,
    AREA_LIGHT = 4
};

struct Material{
    Radiance ambient,diffuse,specular;
    float spec_exp;
    Radiance transmittance,emittance;
    float transparency;
    float index_of_ref;
    IllumType illum;
    bool is_transparent;
    bool is_lightsource;
};

class Mesh{

public:
  Mesh(std::vector<shared_ptr<Face>> faces, 
    bool is_mirror, 
    bool is_transparent,
    int leaf_threshold,
    BVHType treeType,
    Material m);

  ~Mesh();
  float hit(shared_ptr<Face> &closest, const Ray &r, float min_found) const;
 
  public:
    shared_ptr<BVH> bbox;
    std::vector<shared_ptr<Face>> faces;
    bool is_mirror;
    bool is_transparent;
    int size;
    Material material;
};

#endif
