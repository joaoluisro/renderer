#pragma once 
#ifndef LIGHT_HEADER
#define LIGHT_HEADER

#include "core/color.h"
#include "math/vector3d.h"
#include "math/face.h"

class Light
{
public:
  Radiance color;
  float intensity;
  std::shared_ptr<Face> geometry;

public:
  Light(Radiance color, float intensity, std::shared_ptr<Face> geometry);
  ~Light();
};

#endif

