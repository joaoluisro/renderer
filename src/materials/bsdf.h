#ifndef BSDF_HEADER
#define BSDF_HEADER


#include "core/spectrum.h"
#include "core/definitions.h"
#include "geometry/vec3.h"
struct Material{
    Spectrum ambient,diffuse,specular;
    float spec_exp;
    Spectrum transmittance,emittance;
    float transparency;
    float index_of_ref;
    IllumType illum;
    bool is_transparent;
    bool is_lightsource;
};

namespace BSDF
{
    Spectrum eval(const Material &m,
                  const vec3 &p,
                  const vec3 &n,
                  const vec3 &wi,
                  const vec3 &wo,
                  float &pdf_factor,
                  bool &is_delta);
}

#endif
