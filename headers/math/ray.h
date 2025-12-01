#ifndef RAY_HEADER
#define RAY_HEADER

#include "math/vector3d.h"

class Ray
{
public:
  vec3 origin, direction;
  vec3 invDir;
  int sign[3];
public:
  Ray(const vec3 &origin, const vec3 &direction);
  ~Ray();
  bool intersects(vec3 const &p){

    return true;
  }
  vec3 at(float t) const{ 
    return origin + t*direction; 
  }
};

#endif
