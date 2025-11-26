#include "core/sampler.h"

namespace Sampler
{
    Vector3D uniform()
    {
        static RandomNumberGenerator rng;

        float eta1 = rng.generate();
        float eta2 = rng.generate();

        float theta, phi;
        theta = acos(sqrt(eta1));
        phi = 2 * M_PI * eta2;
        float x,y,z;
        x = cos(phi)*sin(theta);
        y = sin(phi)*sin(theta);
        z = cos(theta);

        return Vector3D(x,y,z);
    }

    Vector3D cosineWeighted(Vector3D n)
    {
        static RandomNumberGenerator rng;

        float eta1 = rng.generate();
        float eta2 = rng.generate();

        float theta, phi;
        theta = asin(sqrt(eta1));
        phi = 2 * M_PI * eta2;
        float x,y,z;
        x = cos(phi)*sin(theta);
        y = sin(phi)*sin(theta);
        z = cos(theta);

        Vector3D ze(0.0f,0.0f,1.0f);
        Vector3D t;
        // n is assumed normalized
        if (fabs(n.dot(ze)) < 0.999f)
        {
            t = n.cross(Vector3D(0.0f, 0.0f, 1.0f)).normalized();
        }
        else
        {
            t = n.cross(Vector3D(0.0f, 1.0f, 0.0f)).normalized();
        }

        Vector3D b = n.cross(t);

        auto dir = (x * t) + (y * b) + (z * n);
        return dir.normalized();
    }

}
