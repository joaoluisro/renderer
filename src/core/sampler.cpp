#include "core/sampler.h"


namespace Sample
{
    vec3 fromBSDF(vec3 n, vec3 w0, Material m)
    {
        vec3 wi;
        switch (m.illum)
        {
        case IllumType::OPAQUE:
            wi = cosineWeighted(n);
            break;
        case IllumType::SPECULAR:
        {
            // wi = phongSpecular(n, w0, m.spec_exp);

            static RandomNumberGenerator rng;
            float k = rng.generate();
            if(k < 0.7)
            {
                wi = phongSpecular(n, w0, m.spec_exp);
            }
            else
            {
                wi = cosineWeighted(n);
            }
        }
            break;
        default:
            wi = uniform(n);
            break;
        }
        return wi;
    }

    void build_orthonormal_basis(vec3 v, vec3 &t, vec3 &b)
    {
        vec3 ze(0.0f,0.0f,1.0f);

        // n is assumed normalized
        if (fabs(v.dot(ze)) < 0.999f)
        {
            t = v.cross(vec3(0.0f, 0.0f, 1.0f)).normalized();
        }
        else
        {
            t = v.cross(vec3(0.0f, 1.0f, 0.0f)).normalized();
        }
        b = v.cross(t);
    }

    vec3 phongSpecular(vec3 n, vec3 w0, float s)
    {
        vec3 r = w0.reflect(n).normalized();
        static RandomNumberGenerator rng;

        float eta1 = rng.generate();
        float eta2 = rng.generate();

        float phi = 2.0f * float(M_PI) * eta1;
        float cosTheta = std::pow(eta2, 1.0f / (s + 1.0f)); // importance sampling
        float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));

        float x = sinTheta * std::cos(phi);
        float y = sinTheta * std::sin(phi);
        float z = cosTheta;

        vec3 t, b;
        build_orthonormal_basis(n, t, b);

        vec3 wi = x * t + y * b + z * r;
        return wi.normalized();
    }

    vec3 uniform(vec3 n)
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

        vec3 t, b;
        build_orthonormal_basis(n, t, b);

        auto wi = (x * t) + (y * b) + (z * n);
        return wi.normalized();
    }

    vec3 cosineWeighted(vec3 n)
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

        vec3 t, b;
        build_orthonormal_basis(n, t, b);

        auto wi = (x * t) + (y * b) + (z * n);
        return wi.normalized();
    }

    // radiance L += Sampler::select( L_direct, L_indirect,{0.1,0.9}, kwargs*)


    // int select(vector<float> p, int* m, kwargs...)
    // {
    //     static RandomNumberGenerator rng;
    //     if(rng.generate() < p1) return m1(kwaargs)
    //     ...
    // }

}






