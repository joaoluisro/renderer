#ifndef SAMPLER_H
#define SAMPLER_H

#include <math/vector3d.h>
#include <cmath>
#include <math/randomnumbergenerator.h>


namespace Sampler
{
    Vector3D uniform();
    Vector3D cosineWeighted(Vector3D n);
}

#endif // SAMPLER_H
