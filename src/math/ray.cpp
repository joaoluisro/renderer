
#include "math/ray.h"

Ray::Ray(Vector3D const &origin, Vector3D const &direction)
{
  this->origin = origin;
  this->direction = direction;
  this->box_tests = 0;
  this->leaf_tests = 0;
}

Ray::~Ray()
{
}
