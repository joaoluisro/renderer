#ifndef CAMERA_HEADER
#define CAMERA_HEADER

#include "geometry/vector3d.h"
#include "geometry/ray.h"

#include <math.h>
#include <vector>

class Camera
{
  public:
    Camera(Vector3D &origin, Vector3D &view_point, double fov, int height, int width);
    ~Camera();
    Vector3D pixelToWorldSpace(int i, int j) const;
    void zoom(double z);
  public:
    Vector3D origin;
    Vector3D view_point;
    Vector3D front,up,right;
    double fov;
    int height, width;
    double aspect, scale;
};

#endif
