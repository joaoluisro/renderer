#pragma once 
#ifndef LIGHT_HEADER
#define LIGHT_HEADER

#include "core/spectrum.h"
#include "geometry/face.h"

class Light
{
public:
  Spectrum color;
  float intensity;
  std::shared_ptr<Face> geometry;

public:
  Light(Spectrum color, float intensity, std::shared_ptr<Face> geometry);
  ~Light();
};

#endif

