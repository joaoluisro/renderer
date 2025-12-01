#include "core/light.h"
#include <math.h>

Light::Light(Radiance color, float intensity, std::shared_ptr<Face> geometry) :
             color(color),intensity(intensity), geometry(geometry){}

Light::~Light(){}
