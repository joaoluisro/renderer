#include "core/sampler.h"

namespace Sample
{

    Vector3D uniformSample(const float eta1, const float eta2)
    {
        float theta, phi;
        theta = acos(sqrt(eta1));
        phi = 2 * M_PI * eta2;
        float x,y,z;
        x = cos(phi)*sin(theta);
        y = sin(phi)*sin(theta);
        z = cos(theta);
        return Vector3D(x,y,z);
    }

    Vector3D cosineWeightedSample(const float eta1, const float eta2)
    {
        float theta, phi;
        theta = asin(sqrt(eta1));
        phi = 2 * M_PI * eta2;
        float x,y,z;
        x = cos(phi)*sin(theta);
        y = sin(phi)*sin(theta);
        z = cos(theta);
        return Vector3D(x,y,z);
    }

}
