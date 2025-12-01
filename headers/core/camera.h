#ifndef CAMERA_HEADER
#define CAMERA_HEADER

#include "math/vector3d.h"
#include "math/ray.h"

#include <math.h>
#include <vector>

class Camera
{
  public:
    Camera(vec3 &origin, vec3 &view_point, float fov, int height, int width);
    ~Camera();
    vec3 pixelToWorldSpace(int i, int j, float e1, float e2) const;
    void zoom(float z);
  public:
    vec3 origin;
    vec3 view_point;
    vec3 front,up,right;
    float fov;
    int height, width;
    float aspect, scale;
};

#endif
