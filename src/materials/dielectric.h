#ifndef DIELECTRIC_HEADER
#define DIELECTRIC_HEADER

#include "geometry/vec3.h"

vec3 refractDirection(const vec3 &I, const vec3 &N, float ior);
float fresnel(const vec3 &I, const vec3 &N, const float &ior);

#endif
