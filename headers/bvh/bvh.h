#ifndef BVH_HEADER
#define BVH_HEADER

#include <memory>
#include <vector>
#include <utility>    
#include <algorithm>

#include "geometry/ray.h"
#include "geometry/baseObject.h"

using namespace std;

class BVH { 
  public:
    shared_ptr<BVH> left, right;
    Vector3D min, max;
    bool is_leaf;
    vector<shared_ptr<BaseObject>> faces;

    BVH(
      shared_ptr<BVH> left, 
      shared_ptr<BVH> right,
      const Vector3D &mn, 
      const Vector3D &mx, 
      bool is_leaf,
      vector<shared_ptr<BaseObject>> faces): left(left),right(right), min(mn), max(mx), is_leaf(is_leaf), faces(faces){}
    
    BVH(){};
    
    ~BVH(){};

    double hit(shared_ptr<BaseObject> &closest, Ray &r, double bestT);
};

namespace BvhBBox{
  vector<pair<double,shared_ptr<BaseObject>>> buildCentroidList(vector<shared_ptr<BaseObject>> faces, int axis);
  
  shared_ptr<BVH> buildMedianBVH(vector<shared_ptr<BaseObject>> &faces, int threshold);
  shared_ptr<BVH> buildMidpointBVH(vector<shared_ptr<BaseObject>>& faces, int threshold, int axis);
  shared_ptr<BVH> buildSAHBVH(vector<shared_ptr<BaseObject>>& faces, int bucket_size);

}


#endif