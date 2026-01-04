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
private:
    RandomNumberGenerator rng;
};

namespace Sample
{
    vec3 fromBSDF(vec3 n, vec3 w0, Material m);
    vec3 uniform(vec3 n);
    vec3 phongSpecular(vec3 n,vec3 w0, float shininess);
    vec3 cosineWeighted(vec3 n);
}

#endif // SAMPLER_H
