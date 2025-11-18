#ifndef CAMERA_HEADER
#define CAMERA_HEADER

#include "math/vector3d.h"
#include "math/ray.h"

#include <math.h>
#include <vector>

class Camera
{
  public:
    Camera(Vector3D &origin, Vector3D &view_point, float fov, int height, int width);
    ~Camera();
    Vector3D pixelToWorldSpace(int i, int j) const;
    void zoom(float z);
  public:
    Vector3D origin;
    Vector3D view_point;
    Vector3D front,up,right;
    float fov;
    int height, width;
    float aspect, scale;
};

#endif
