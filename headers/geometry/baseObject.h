#pragma once
#ifndef BASE_OBJECT_HEADER
#define BASE_OBJECT_HEADER

#include <memory>
#include <vector>

#include "geometry/vector3d.h"
#include "geometry/ray.h"
#include "core/color.h"

struct Material{
  Color ambient,diffuse,specular;
  double spec_exp;
  double reflectance;
  double transparency;
  Color transmittance;
  double index_of_ref;
};

class BaseObject
{
public:
  virtual ~BaseObject() = default;
  virtual double intersects(Ray &r) const = 0;  
  virtual Vector3D get_normal(Vector3D &at) const = 0;
  virtual Color get_color() const = 0;
  virtual Vector3D centroid() const = 0;
  virtual Material material() = 0;
  virtual Vector3D max() const  = 0;
  virtual Vector3D min() const = 0;
  virtual bool isOutOfBounds(Vector3D &max, Vector3D &min) const = 0;

};

#endif