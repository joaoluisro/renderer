#pragma once 
#ifndef LIGHT_HEADER
#define LIGHT_HEADER

#include "core/color.h"
#include "math/vector3d.h"
#include "math/face.h"

class Light
{
public:
  Color color;
  float intensity;
  std::shared_ptr<Face> geometry;

public:
  Light(Color color, float intensity, std::shared_ptr<Face> geometry);
  ~Light();
};

#endif

