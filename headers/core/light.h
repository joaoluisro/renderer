#pragma once 
#ifndef LIGHT_HEADER
#define LIGHT_HEADER

#include "core/color.h"
#include "geometry/vector3d.h"

class Light
{
public:
  Vector3D position;
  Color color;
public:
  Light(Vector3D position, Color color) : 
  position(position),color(color){};
  ~Light(){};
};

#endif

