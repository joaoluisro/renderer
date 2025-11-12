#include "core/light.h"
#include <math.h>

Light::Light(Vector3D position, Color color, float intensity, std::shared_ptr<Face> geometry) :
    position(position),color(color),intensity(intensity), geometry(geometry){}

Light::~Light(){}

Vector3D Light::getDirection(const Vector3D &p)
{
    return (position - p).normalized();
}

float Light::getIntensity(const Vector3D &p)
{
    auto r = (position - p).length();
    auto falloff = 1/(M_PI * r *r);
    return intensity * falloff;
}
