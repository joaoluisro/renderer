#pragma once
#ifndef BASE_OBJECT_HEADER
#define BASE_OBJECT_HEADER

#include "math/vector3d.h"
#include "math/ray.h"
#include "core/color.h"

class Face
{
public:
  virtual ~Face() = default;
  virtual float intersects(const Ray& r) const = 0;
  virtual Vector3D get_normal(const Vector3D& at) const = 0;
  virtual Color get_color() const = 0;
  virtual Vector3D centroid() const = 0;
  virtual Vector3D max() const  = 0;
  virtual Vector3D min() const = 0;
  virtual bool isOutOfBounds(const Vector3D &max,const Vector3D &min) const = 0;
  virtual Vector3D generateUniform(float e1, float e2) const = 0;
  virtual float getArea() const = 0;
};

#endif
