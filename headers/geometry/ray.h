#ifndef RAY_HEADER
#define RAY_HEADER

#include "geometry/vector3d.h"

class Ray
{
public:
  Vector3D origin, direction;
  int box_tests, leaf_tests;
public:
  Ray(Vector3D const &origin, Vector3D const &direction);
  ~Ray();
  bool intersects(Vector3D const &p){

    return true;
  }
  Vector3D at(double t) const{ 
    return origin + t*direction; 
  }
};

#endif