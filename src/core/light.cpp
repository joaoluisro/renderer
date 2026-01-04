#include "core/light.h"

Light::Light(Spectrum color, float intensity, std::shared_ptr<Face> geometry) :
             color(color),intensity(intensity), geometry(geometry){}

Light::~Light(){}
