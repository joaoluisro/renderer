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
  // this->up = this->front.cross(Vector3D(0.0f,0.0f,1.0f)).normalized();
  Vector3D worldUp(0,0,1);
  if (fabs(front.dot(worldUp)) > 0.99f) {
      worldUp = Vector3D(1,0,0);
  }

  Vector3D upProj = worldUp - front * front.dot(worldUp);
  this->up = upProj;
  // this->up = worldUp;
  this->up.normalize();

  this->right = this->front.cross(this->up);

  this->fov = fov;
  this->height = height;
  this->width = width;

  this->aspect = width/height;
  this->scale = tan(fov/2);
}

Camera::~Camera()
{
}

Vector3D Camera::pixelToWorldSpace(int i, int j) const
{
  float x,y;
  // x = ((i + 0.5) - width/2)/width/2;
  // y = (height/2 - (j + 0.5))/height/2;
  
  // normalized screen coords u,v ∈ [-1,1]
  x = ( (i + 0.5f) / width  ) * 2.0f - 1.0f;  
  y = 1.0f - ( (j + 0.5f) / height ) * 2.0f;  // flip Y so +v is up

  x *= aspect * scale;
  y *= scale;

  // return front;
  return(right * x + up * y + front).normalized();
  // return(x * right + y * up - front);

  // Vector3D c_space_direction(x,y,-1);
  // Vector3D w_space_camera_direction(x);
  // return Vector3D();
}
