#include <math.h>

#include "core/camera.h"
#include "math/ray.h"

Camera::Camera(Vector3D &origin, 
               Vector3D &view_point, 
               float fov,
               int height,
               int width)
{
  auto view_point_transformed = view_point.to_blender();
  auto origin_transformed = origin.to_blender();

  this->origin = origin_transformed;
  this->aspect = float(width) / float(height);
  this->width = width;
  this->height = height;
  Vector3D worldUp(0,1,0);
  this->front = (view_point_transformed - origin_transformed).normalized();
  this->right = (front.cross(worldUp)).normalized();
  this->up    = (right.cross(front)).normalized();
  this->fov = fov;
  this->scale = tan((fov * M_PI/180.0) * 0.5);
}

Camera::~Camera()
{
}

Vector3D Camera::pixelToWorldSpace(int i, int j, float e1, float e2) const
{
  float ndcX = (2*(i + e1)/width  - 1) * aspect * scale;
  float ndcY = (1 - 2*(j + e2)/height) * scale;
  return ((right*ndcX + up*ndcY) + front).normalized();
}

void Camera::zoom(float z)
{
    Ray r(origin, front);
    this->origin = this->origin + r.at(z);
}
