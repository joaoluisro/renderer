#pragma once 
#ifndef LIGHT_HEADER
#define LIGHT_HEADER

#include "core/color.h"
#include "geometry/vector3d.h"
#include "geometry/face.h"

class Light
{
public:
  Vector3D position;
  Color color;
  float intensity;
  std::shared_ptr<Face> geometry;

public:
  Light(Vector3D position, Color color, float intensity, std::shared_ptr<Face> geometry);
  ~Light();
  Vector3D getDirection(const Vector3D& p);
  float getIntensity(const Vector3D& p);
};

#endif

