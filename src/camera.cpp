#include "camera.h"

Camera::Camera(Vector3D &origin, 
               Vector3D &view_point, 
               float fov,
               int height,
               int width)
{
  this->origin = origin;
  // T - C / len(T - C)
  Vector3D diff =  view_point - origin;
  this->front = diff.normalized();

  // world up: [0,0,1]
  Vector3D worldUp(0,1,0);
  // if (fabs(front.dot(worldUp)) > 0.99f) {
  //     worldUp = Vector3D(1,0,0);
  // }

  // Vector3D upProj = worldUp - front * front.dot(worldUp);
  this->up = worldUp.normalized();

  this->right = this->front.cross(this->up);

  this->fov = fov;
  this->height = height;
  this->width = width;

  this->aspect = width/height;
  this->scale = tan(fov/2.0f);
}

Camera::~Camera()
{
}

Vector3D Camera::pixelToWorldSpace(int i, int j) const
{
  float x,y;
  
  x = ( (i + 0.5f) / width  ) * 2.0f - 1.0f;  
  y = 1.0f - ( (j + 0.5f) / height ) * 2.0f;

  x *= aspect * scale;
  y *= scale;

  return(right * x + up * y + front).normalized();
}
