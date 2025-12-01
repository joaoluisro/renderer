#ifndef SAMPLER_H
#define SAMPLER_H

#include <math/vector3d.h>
#include <cmath>
#include <math/randomnumbergenerator.h>
#include <bvh/mesh.h>

namespace Sample
{
    vec3 fromBSDF(vec3 n, vec3 w0, Material m);
    vec3 uniform(vec3 n);
    vec3 phongSpecular(vec3 n,vec3 w0, float shininess);
    vec3 cosineWeighted(vec3 n);
}

#endif // SAMPLER_H
