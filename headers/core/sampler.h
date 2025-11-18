#ifndef SAMPLER_H
#define SAMPLER_H

#include <math/vector3d.h>
#include <cmath>

namespace Sample
{
    Vector3D uniformSample(const float eta1, const float eta2);
    Vector3D cosineWeightedSample(const float eta1, const float eta2);
}

#endif // SAMPLER_H
