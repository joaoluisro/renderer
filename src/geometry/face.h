#pragma once
#ifndef BASE_OBJECT_HEADER
#define BASE_OBJECT_HEADER

#include "geometry/vec3.h"
#include "geometry/ray.h"
#include "core/spectrum.h"

class Face
{
public:
  virtual ~Face() = default;
  virtual float intersects(const Ray& r) const = 0;
  virtual vec3 getNormal(const vec3& at) const = 0;
  virtual Spectrum getColor() const = 0;
  virtual vec3 getCentroid() const = 0;
  virtual vec3 max() const  = 0;
  virtual vec3 min() const = 0;
  virtual bool isOutOfBounds(const vec3 &max,const vec3 &min) const = 0;
  virtual vec3 generateUniform() const = 0;
  virtual float getArea() const = 0;
};

#endif
