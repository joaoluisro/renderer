#ifndef RAY_HEADER
#define RAY_HEADER

#include "math/vector3d.h"
class Ray
{
public:
  Vector3D origin, direction;
  Vector3D invDir;
  int sign[3];
public:
  Ray(const Vector3D &origin, const Vector3D &direction);
  ~Ray();
  bool intersects(Vector3D const &p){

    return true;
  }
  Vector3D at(float t) const{ 
    return origin + t*direction; 
  }
};

#endif
