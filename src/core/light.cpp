#include "core/light.h"
#include <math.h>

Light::Light(Color color, float intensity, std::shared_ptr<Face> geometry) :
             color(color),intensity(intensity), geometry(geometry){}

Light::~Light(){}
