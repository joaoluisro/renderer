#include "bvh/bvh.h"
using namespace std;

#define THRESHOLD 4
#define EPSILON 1e-5

namespace BvhBBox
{
Vector3D computeMax(vector<shared_ptr<Face>> faces)
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

Vector3D computeMin(vector<shared_ptr<Face>> faces)
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

vector<pair<float,shared_ptr<Face>>> buildCentroidList(vector<shared_ptr<Face>> faces, int axis)
{
  vector<pair<float,shared_ptr<Face>>> centroid_list;
  for(auto &face : faces)
  {
    centroid_list.push_back(make_pair(face->getCentroid()[axis],face));
  }
  return centroid_list;
}

shared_ptr<BVH> buildMedianBVH(vector<shared_ptr<Face>> &faces, int threshold) {
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
  
  vector<shared_ptr<Face>> leftFaces;
  vector<shared_ptr<Face>> rightFaces;
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
 
shared_ptr<BVH> buildMidpointBVH(vector<shared_ptr<Face>>& faces, int threshold, int axis)
{
  // compute min/max bounds from faces
  Vector3D mn = computeMin(faces);
  Vector3D mx = computeMax(faces);
  if (faces.size() <= threshold)
  {
    return make_shared<BVH>(nullptr,nullptr,mn,mx,true,faces);
  }
  auto mid_point = (mn + mx) * 0.5;
  vector<shared_ptr<Face>> left,right;

  for(auto &face : faces)
  {
    auto centroid = face->getCentroid();
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

shared_ptr<BVH> buildSAHBVH(vector<shared_ptr<Face>>& faces, int threshold)
{

  auto max = computeMax(faces);
  auto min = computeMin(faces);
  if(faces.size() <= 2)
  {
    return make_shared<BVH>(nullptr, nullptr, min, max, true, faces);
  }
  int best_axis = -1;
  float best_length = -1;
  float best_cost = 1e+6f;

  float extent[3] = {max[0] - min[0], max[1] - min[1],  max[2] - min[2]};
  float total_area = 2*(extent[0]*extent[1]) + 2*(extent[0]*extent[2]) + 2*(extent[1]*extent[2]);

  float traversal_time = 1.0f;

  for(int i = 0; i < 3; i++)
  {
    float interval = extent[i]/threshold;

    for(int j = 1; j < threshold; j++)
    {
      int left_count = 0;
      int right_count = 0;
      for(auto face : faces)
      {
        if(face->getCentroid()[i] < min[i] + interval * j)
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
      float left_axis0_length = (interval * j);
      float right_axis0_length = extent[i] - left_axis0_length;

      float left_area  = 2*(left_axis0_length * extent[axis1]) + 2*(left_axis0_length * extent[axis2]) + 2*(extent[axis2] * extent[axis1]);
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
  
  vector<shared_ptr<Face>> left,right;
  for(auto &face : faces)
  {
    auto centroid = face->getCentroid();
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
                     float& tNear,
                     float& tFar)
{
    const Vector3D bounds[2] = { bmin, bmax };

    float txmin = (bounds[r.sign[0]].x - r.origin.x) * r.invDir.x;
    float txmax = (bounds[1 - r.sign[0]].x - r.origin.x) * r.invDir.x;

    float tymin = (bounds[r.sign[1]].y - r.origin.y) * r.invDir.y;
    float tymax = (bounds[1 - r.sign[1]].y - r.origin.y) * r.invDir.y;

    if ((txmin > tymax) || (tymin > txmax))
        return false;

    if (tymin > txmin) txmin = tymin;
    if (tymax < txmax) txmax = tymax;

    float tzmin = (bounds[r.sign[2]].z - r.origin.z) * r.invDir.z;
    float tzmax = (bounds[1 - r.sign[2]].z - r.origin.z) * r.invDir.z;

    if ((txmin > tzmax) || (tzmin > txmax))
        return false;

    if (tzmin > txmin) txmin = tzmin;
    if (tzmax < txmax) txmax = tzmax;

    tNear = txmin;
    tFar  = txmax;
    return tFar >= 0.0f;
}

float BVH::hit(std::shared_ptr<Face> &closest, const Ray &r, float bestT, float bestGlobal) const
{

  // test this node's bounding box
  float tNear, tFar;
  if (!slabAABB(r, min, max, tNear, tFar) || tNear > bestT || tNear > bestGlobal)
      return -1.0;  // No hit with this node

  // if leaf node, test all faces
  if (is_leaf)
  {
      float closestT = bestT;
      std::shared_ptr<Face> closestObj = nullptr;

      for (const auto &face : faces)
      {
          float t = face->intersects(r);

          if (t > EPSILON && t < closestT)
          {
              closestT = t;
              closestObj = face;
          }
      }
      if (closestObj)
      {
          closest = closestObj;
          return closestT;
      }
      return -1.0;
  }
  
  // if not a leaf, traverse children (closest-first)
  float tNearL = std::numeric_limits<float>::infinity();
  float tFarL  = std::numeric_limits<float>::infinity();
  float tNearR = std::numeric_limits<float>::infinity();
  float tFarR  = std::numeric_limits<float>::infinity();

  bool hitLeft  = left  && slabAABB(r, left->min, left->max, tNearL, tFarL);
  bool hitRight = right && slabAABB(r, right->min, right->max, tNearR, tFarR);

  if(!hitLeft && !hitRight) return -1.0;

  std::shared_ptr<Face> leftClosest, rightClosest;
  float leftHit  = -1.0, rightHit = -1.0;

  // visit nearer child first
  if(hitLeft && hitRight)
  {
    if (tNearL < tNearR)
    {
      leftHit = left->hit(leftClosest, r, bestT, bestGlobal);
      if (leftHit > 0.0) bestT = leftHit;

      rightHit = right->hit(rightClosest, r, bestT, bestGlobal);
    }
    else
    {
      rightHit = right->hit(rightClosest, r, bestT, bestGlobal);
      if (rightHit > 0.0) bestT = rightHit;

      leftHit = left->hit(leftClosest, r, bestT, bestGlobal);
    }
  }
  else if(hitLeft)
  {
      leftHit = left->hit(leftClosest, r, bestT, bestGlobal);
  }
  else if(hitRight)
  {
      rightHit = right->hit(rightClosest, r, bestT, bestGlobal);
  }

  // determine which child produced the closest hit
  if(leftHit > 0.0 && rightHit > 0.0)
  {
    if(leftHit < rightHit)
    {
      closest = leftClosest;
      return leftHit;
    }
    else
    {
      closest = rightClosest;
      return rightHit;
    }
  }
  else if (leftHit > 0.0)
  {
    closest = leftClosest;
    return leftHit;
  }
  else if (rightHit > 0.0)
  {
    closest = rightClosest;
    return rightHit;
  }

  return -1.0;
}
