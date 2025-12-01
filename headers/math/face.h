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
  virtual vec3 get_normal(const vec3& at) const = 0;
  virtual Radiance get_color() const = 0;
  virtual vec3 getCentroid() const = 0;
  virtual vec3 max() const  = 0;
  virtual vec3 min() const = 0;
  virtual bool isOutOfBounds(const vec3 &max,const vec3 &min) const = 0;
  virtual vec3 generateUniform() const = 0;
  virtual float getArea() const = 0;
};

#endif
