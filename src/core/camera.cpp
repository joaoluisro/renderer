#include <math.h>

#include "core/camera.h"

Camera::Camera(vec3 &origin,
               vec3 &view_point,
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
  this->front = (view_point_transformed - origin_transformed).normalized();
  this->right = (front.cross(WORLD_UP)).normalized();
  this->up    = (right.cross(front)).normalized();
  this->fov = fov;
  this->scale = tan((fov * M_PI/180.0f) * 0.5);
}

Camera::~Camera()
{
}

vec3 Camera::toWorldSpace(const int i,const int j,const float e1,const float e2) const
{
  float ndcX = (2*(i + e1)/width  - 1) * aspect * scale;
  float ndcY = (1 - 2*(j + e2)/height) * scale;
  return ((right*ndcX + up*ndcY) + front).normalized();
}
