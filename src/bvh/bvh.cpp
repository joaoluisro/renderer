#include "bvh/bvh.h"
using namespace std;

#define THRESHOLD 4

namespace BvhBBox
{
Vector3D computeMax(vector<shared_ptr<BaseObject>> faces)
{
  auto x_max= -1e+9f;
  auto y_max= -1e+9f;
  auto z_max = -1e+9f;
  for(auto &face : faces)
  { 
    auto max = face->max();
    if(max.x > x_max) x_max = max.x;
    if(max.y > y_max) y_max = max.y;
    if(max.z > z_max) z_max = max.z;
  }
  return Vector3D(x_max, y_max, z_max);
}

Vector3D computeMin(vector<shared_ptr<BaseObject>> faces)
{
  auto x_min= 1e+9f;
  auto y_min= 1e+9f;
  auto z_min = 1e+9f;
  for(auto &face : faces)
  { 
    auto min = face->min();
    if(min.x < x_min) x_min = min.x;
    if(min.y < y_min) y_min = min.y;
    if(min.z < z_min) z_min = min.z;
  }
  return Vector3D(x_min, y_min, z_min);
}

vector<pair<double,shared_ptr<BaseObject>>> buildCentroidList(vector<shared_ptr<BaseObject>> faces, int axis)
{
  vector<pair<double,shared_ptr<BaseObject>>> centroid_list;
  for(auto &face : faces)
  {
    centroid_list.push_back(make_pair(face->centroid()[axis],face));
  }
  return centroid_list;
}

shared_ptr<BVH> buildMedianBVH(vector<shared_ptr<BaseObject>> &faces, int threshold) {
  // compute min/max bounds from faces
  Vector3D mn = computeMin(faces);
  Vector3D mx = computeMax(faces);
  if (faces.size() <= threshold)
  {
    return make_shared<BVH>(nullptr,nullptr,mn,mx,true,faces);
  }
  auto diff = mx - mn;
  int axis;

  if(diff.x > diff.y && diff.x > diff.z) 
  {
    axis = 0;
  }
  else if (diff.y > diff.x && diff.y > diff.z) 
  {
    axis = 1;
  }
  else
  {
    axis = 2;
  }

  auto centroids = buildCentroidList(faces,axis);

  sort(centroids.begin(), centroids.end(),
    [](auto const &a, auto const &b) {
        return a.first < b.first;
    });

  size_t n    = centroids.size();
  size_t half = n / 2;
  
  vector<shared_ptr<BaseObject>> leftFaces;
  vector<shared_ptr<BaseObject>> rightFaces;
  leftFaces.reserve(half);
  rightFaces.reserve(centroids.size() - half);

  for (size_t i = 0; i < half; ++i) {
    leftFaces.push_back(move(centroids[i].second));
  }
  for (size_t i = half; i < centroids.size(); ++i) {
    rightFaces.push_back(move(centroids[i].second));
  }
  
  faces.clear();    
  centroids.clear();
  auto leftNode  = buildMedianBVH(leftFaces,threshold);
  auto rightNode = buildMedianBVH(rightFaces,threshold);
  return make_shared<BVH>(leftNode, rightNode, mn, mx, false, faces);
}
 
shared_ptr<BVH> buildMidpointBVH(vector<shared_ptr<BaseObject>>& faces, int threshold, int axis)
{
  // compute min/max bounds from faces
  Vector3D mn = computeMin(faces);
  Vector3D mx = computeMax(faces);
  if (faces.size() <= threshold)
  {
    return make_shared<BVH>(nullptr,nullptr,mn,mx,true,faces);
  }
  auto mid_point = (mn + mx) * 0.5;
  vector<shared_ptr<BaseObject>> left,right;

  for(auto &face : faces)
  {
    auto centroid = face->centroid();
    if(centroid[axis] < mid_point[axis])
    {
      left.push_back(face);
    }
    else
    {
      right.push_back(face);
    }
  }
  faces.clear();
  shared_ptr<BVH> leftNode, rightNode;

  if(left.size() == 0 || right.size() == 0)
  {
    return make_shared<BVH>(nullptr,nullptr,mn,mx,true,faces);
  }

  leftNode  = buildMidpointBVH(left,threshold, (axis + 1) % 3);
  rightNode = buildMidpointBVH(right,threshold, (axis + 1) % 3);

  return make_shared<BVH>(leftNode, rightNode, mn, mx, false, faces);

}

shared_ptr<BVH> buildSAHBVH(vector<shared_ptr<BaseObject>>& faces, int threshold)
{

  auto max = computeMax(faces);
  auto min = computeMin(faces);
  if(faces.size() <= 2)
  {
    return make_shared<BVH>(nullptr, nullptr, min, max, true, faces);
  }
  int best_axis = -1;
  double best_length = -1;
  double best_cost = 1e+6f;

  double extent[3] = {max[0] - min[0], max[1] - min[1],  max[2] - min[2]};
  double total_area = 2*(extent[0]*extent[1]) + 2*(extent[0]*extent[2]) + 2*(extent[1]*extent[2]);

  double traversal_time = 1.0f;

  for(int i = 0; i < 3; i++)
  {
    double interval = extent[i]/threshold;

    for(int j = 1; j < threshold; j++)
    {
      int left_count = 0;
      int right_count = 0;
      for(auto face : faces)
      {
        if(face->centroid()[i] < min[i] + interval * j)
        {
          left_count++;
        }
        else
        {
          right_count++;
        }
      }

      int axis1 = (i + 1)%3;
      int axis2 = (i + 2)%3;
      double left_axis0_length = (interval * j);
      double right_axis0_length = extent[i] - left_axis0_length;

      double left_area  = 2*(left_axis0_length * extent[axis1]) + 2*(left_axis0_length * extent[axis2]) + 2*(extent[axis2] * extent[axis1]);
      auto right_area = 2*(right_axis0_length * extent[axis1]) + 2*(right_axis0_length * extent[axis2]) + 2*(extent[axis2] * extent[axis1]);


      auto cost = traversal_time + (left_area/total_area)*(left_count) + (right_area/total_area)*(right_count);
      if(cost < best_cost)
      {
        best_axis = i;
        best_length = interval * j;
        best_cost = cost;
      }
    }
  }
  
  vector<shared_ptr<BaseObject>> left,right;
  for(auto &face : faces)
  {
    auto centroid = face->centroid();
    if(centroid[best_axis] < min[best_axis] + best_length)
    {
      left.push_back(face);
    }
    else
    {
      right.push_back(face);
    }
  }
  shared_ptr<BVH> leftNode, rightNode;
  if(left.size() == 0 || right.size() == 0)
  {
    return make_shared<BVH>(nullptr, nullptr, min, max, true, faces);
  }
  faces.clear();

  leftNode  = buildSAHBVH(left, threshold);
  rightNode = buildSAHBVH(right, threshold);
  return make_shared<BVH>(leftNode, rightNode, min, max, false, faces);

}


}

inline bool slabAABB(const Ray& r,
                     const Vector3D& bmin,
                     const Vector3D& bmax,
                     double& tNear,
                     double& tFar)
{
    tNear = 0.0;
    tFar  = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 3; ++i) {
        double o = r.origin[i];
        double d = r.direction[i];

        if (fabs(d) < 1e-7) {
            if (o < bmin[i] || o > bmax[i]) return false;
            continue;
        }

        double invD = 1.0 / d;
        double t0   = (bmin[i] - o) * invD;
        double t1   = (bmax[i] - o) * invD;
        if (invD < 0.0) std::swap(t0, t1);

        tNear = std::max(tNear, t0);
        tFar  = std::min(tFar, t1);
        if (tFar < tNear) return false;
    }
    return tFar >= 0.0;
}


double BVH::hit(std::shared_ptr<BaseObject> &closest, Ray &r, double bestT)
{

  // 1. Test this node's bounding box
  double tNear, tFar;
  if (!slabAABB(r, min, max, tNear, tFar) || tNear > bestT) 
      return -1.0;  // No hit with this node

  r.box_tests+= 1;
  // 2. If this is a leaf node, test all faces
  if (is_leaf) {
      double closestT = bestT;
      std::shared_ptr<BaseObject> closestObj = nullptr;

      for (auto& face : faces) {
          double t = face->intersects(r);
          r.leaf_tests += 1;

          if (t > 0.0 && t < closestT) {
              closestT = t;
              closestObj = face;
          }
      }
      if (closestObj) {
          closest = closestObj;
          return closestT;
      }
      return -1.0; // No hit among faces
  }
  
  // 3. If not a leaf, traverse children (closest-first)
  double tNearL = std::numeric_limits<double>::infinity();
  double tFarL  = std::numeric_limits<double>::infinity();
  double tNearR = std::numeric_limits<double>::infinity();
  double tFarR  = std::numeric_limits<double>::infinity();

  bool hitLeft  = left  && slabAABB(r, left->min, left->max, tNearL, tFarL);
  bool hitRight = right && slabAABB(r, right->min, right->max, tNearR, tFarR);

  if (!hitLeft && !hitRight) return -1.0;
  std::shared_ptr<BaseObject> leftClosest, rightClosest;
  double leftHit  = -1.0, rightHit = -1.0;

  // Visit nearer child first
  if (hitLeft && hitRight) {
      if (tNearL < tNearR) {
          leftHit = left->hit(leftClosest, r, bestT);
          if (leftHit > 0.0) bestT = leftHit;  // tighten bestT
          rightHit = right->hit(rightClosest, r, bestT);
      } else {
          rightHit = right->hit(rightClosest, r, bestT);
          if (rightHit > 0.0) bestT = rightHit;
          leftHit = left->hit(leftClosest, r, bestT);
      }
  } else if (hitLeft) {
      leftHit = left->hit(leftClosest, r, bestT);
  } else if (hitRight) {
      rightHit = right->hit(rightClosest, r, bestT);
  }

  // 4. Determine which child produced the closest hit
  if (leftHit > 0.0 && rightHit > 0.0) {
      if (leftHit < rightHit) {
          closest = leftClosest;
          return leftHit;
      } else {
          closest = rightClosest;
          return rightHit;
      }
  } else if (leftHit > 0.0) {
      closest = leftClosest;
      return leftHit;
  } else if (rightHit > 0.0) {
      closest = rightClosest;
      return rightHit;
  }

  return -1.0;
}