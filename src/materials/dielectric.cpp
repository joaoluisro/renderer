
#include "materials/dielectric.h"

vec3 refractDirection(const vec3 &I, const vec3 &N, float ior)
{
    vec3 dir = I.normalized();
    vec3 n   = N.normalized();

    float cosi = std::clamp(dir.dot(n), -1.0f, 1.0f);
    float etai = 1.0f;       // air
    float etat = ior;        // medium

    if (cosi < 0.0f) {
        // Ray is outside -> entering the medium
        cosi = -cosi;
    } else {
        // Ray is inside -> exiting the medium
        std::swap(etai, etat);
        n = n * -1.0f;     // flip normal
    }

    float eta = etai / etat;
    float k   = 1.0f - eta * eta * (1.0f - cosi * cosi);

    if (k < 0.0f) {
        // Total internal reflection – caller should not use this direction
        return vec3(0, 0, 0);
    }

    return eta * dir + (eta * cosi - sqrtf(k)) * n;
}

float fresnel(const vec3 &I, const vec3 &N, const float &ior)
{
    float cosi = std::clamp(I.dot(N), -1.0f, 1.0f);
    float etai = 1.0f;
    float etat = ior;

    if (cosi > 0.0f) {
        std::swap(etai, etat);  // ray is inside the object
    }

    // Snell's law
    float sint = etai / etat * sqrtf(std::max(0.0f, 1.0f - cosi * cosi));
    if (sint >= 1.0f) {
        return 1.0f;  // Total internal reflection
    }

    float cost = sqrtf(std::max(0.0f, 1.0f - sint * sint));
    cosi = std::fabs(cosi);
    float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
    float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
    return (Rs * Rs + Rp * Rp) * 0.5f;
}

// Spectrum PathTracer::L_transparent(const vec3 &w0,
//                                    const Intersection &hit,
//                                    const vec3 &n_hit,
//                                    const int n_samples,
//                                    int depth)
// {
//     if(GOTHROUGH)
//     {
//         Ray passthrough(hit.point + w0 * EPSILON, w0);
//         return Li(passthrough, n_samples, true, depth + 1);
//     }
//     // w0: ray *direction* (from camera to PathTracer), so incoming at the surface is -w0
//     vec3 wo = w0; // outgoing from surface toward camera? adjust to your convention

//     vec3 nl = (wo.dot(n_hit) < 0.0f) ? n_hit : n_hit * -1.0f;

//     bool outside = (wo.dot(n_hit) < 0.0f);
//     vec3 bias = nl * EPSILON;
//     vec3 origin = outside ? hit.point + bias : hit.point - bias;

//     float ior = hit.material.index_of_ref;

//     // Compute reflection dir
//     vec3 reflectDir = wo.reflect(nl).normalized();

//     // Compute refraction dir (use oriented normal nl!)
//     vec3 refractDir = refractDirection(w0, nl, ior).normalized();

//     // Fresnel using oriented normal nl
//     float kr = fresnel(wo, nl, ior); // kr = reflectance

//     // If refractDirection fails (TIR), just reflect
//     if (refractDir.x == 0 && refractDir.y == 0 && refractDir.z == 0) {
//         Ray reflectRay(origin, reflectDir);
//         return Li(reflectRay, n_samples, true, depth + 1);
//     }

//     Ray reflectRay(origin, reflectDir);
//     Ray refractRay(origin, refractDir);

//     // Spectrum Lr = Li(reflectRay, n_samples, true, depth + 1);
//     Spectrum Lr;
//     Spectrum Lt = Li(refractRay, n_samples, true, depth + 1);

//     return Lr * kr + Lt * (1.0f - kr);
// }
