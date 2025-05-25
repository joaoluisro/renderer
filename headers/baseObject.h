#pragma once
#ifndef BASE_OBJECT_HEADER
#define BASE_OBJECT_HEADER

#include "vector3d.h"
#include "ray.h"
#include "color.h"

class BaseObject
{
public:
  virtual ~BaseObject() = default;
  virtual bool intersects(Ray &r, Vector3D &p) const = 0;  
  virtual Vector3D get_normal(Vector3D &at) const = 0;
  virtual Color get_color() const = 0;
  virtual void translate(Vector3D &to) = 0;
  virtual Vector3D centroid() const = 0;
};

#endif