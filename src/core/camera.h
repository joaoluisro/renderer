#ifndef CAMERA_HEADER
#define CAMERA_HEADER


#include "core/definitions.h"
#include "geometry/vec3.h"

#include <math.h>

class Camera
{
  public:
    Camera(vec3 &origin, vec3 &view_point, float fov, int height, int width);
    ~Camera();
    vec3 toWorldSpace(int i, int j, float e1, float e2) const;
  public:
    vec3 origin;
    vec3 view_point;
    vec3 front,up,right;
    float fov;
    int height, width;
    float aspect, scale;
};

#endif
