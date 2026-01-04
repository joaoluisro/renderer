#ifndef BVH_HEADER
#define BVH_HEADER

#include <memory>
#include <vector>
#include <utility>    
#include <algorithm>

#include "geometry/ray.h"
#include "geometry/face.h"

#include "core/definitions.h"

using namespace std;

enum BVHType{
    MIDPOINT,
    MEDIAN,
    SAH
};

class BVH { 
  public:
    shared_ptr<BVH> left, right;
    vec3 min, max;
    bool is_leaf;
    vector<shared_ptr<Face>> faces;

    BVH(
      shared_ptr<BVH> left, 
      shared_ptr<BVH> right,
      const vec3 &mn, 
      const vec3 &mx, 
      bool is_leaf,
      vector<shared_ptr<Face>> faces): left(left),right(right), min(mn), max(mx), is_leaf(is_leaf), faces(faces){}
    
    BVH(){};
    
    ~BVH(){};

    float hit(shared_ptr<Face> &closest, const Ray &r, float bestT, float bestGlobal) const;
};

namespace BvhBBox{
  vector<pair<float,shared_ptr<Face>>> buildCentroidList(vector<shared_ptr<Face>> faces, int axis);
  
  shared_ptr<BVH> buildMedianBVH(vector<shared_ptr<Face>> &faces, int threshold);
  shared_ptr<BVH> buildMidpointBVH(vector<shared_ptr<Face>>& faces, int threshold, int axis);
  shared_ptr<BVH> buildSAHBVH(vector<shared_ptr<Face>>& faces, int bucket_size);

}


#endif
