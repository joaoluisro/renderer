#ifndef SAMPLER_H
#define SAMPLER_H

#include <geometry/vec3.h>
#include <cmath>
#include <math/rng.h>
#include <geometry/mesh.h>

class Sampler
{
public:
    Sampler();
    float nextScalar();
public:
    RandomNumberGenerator rng;
};

namespace Sample
{
    vec3 fromBSDF(const vec3 &n,const vec3 &w0, const Material &m);
    vec3 uniform(const vec3 &n);
    vec3 phongSpecular(const vec3 &n, const vec3 &w0, const float shininess);
    vec3 cosineWeighted(const vec3 &n);
}

#endif // SAMPLER_H
