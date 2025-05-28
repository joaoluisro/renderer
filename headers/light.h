#pragma once 
#ifndef LIGHT_HEADER
#define LIGHT_HEADER

#include "vector3d.h"
#include "color.h"

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

