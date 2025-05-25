
#include "ray.h"

Ray::Ray(Vector3D const &origin, Vector3D const &direction)
{
  this->origin = origin;
  this->direction = direction;
}

Ray::~Ray()
{
}
