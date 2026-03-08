
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
