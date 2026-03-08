
#include "materials/bsdf.h"
#include "core/definitions.h"

namespace BSDF
{

Spectrum eval(const Material &m,
              const vec3 &p,
              const vec3 &n,
              const vec3 &wi,
              const vec3 &wo,
              float &pdf_factor,
              bool &is_delta)
{
    Spectrum shading;

    float cosI = std::max(0.0f, n.dot(wi));

    switch (m.illum)
    {
        case IllumType::OPAQUE:
        {
            Spectrum diff = m.diffuse / M_PI;
            shading = diff;
            pdf_factor = cosI / M_PI;
            is_delta = false;
            break;
        }
        case IllumType::SPECULAR:
        {
            float s = m.spec_exp;
            Spectrum diff = m.diffuse / M_PI;
            // Phong specular BRDF
            vec3 r = wo.reflect(n).normalized(); // or wo.reflect(n) depending on your convention
            float roi = std::max(0.0f, r.dot(wi));

            Spectrum spec = m.specular * ((s + 2.0f) / (2.0f * M_PI)) * std::pow(roi, s);
            // Full BRDF is sum of lobes
            shading = spec;
            float diff_factor = cosI / M_PI;

            float spec_factor = (s + 1.0f) * std::pow(roi, s) / (2.0f * M_PI);

            pdf_factor = spec_factor;
            is_delta   = false;
            break;
        }
        case IllumType::SPECULAR_DIFFUSE:
        {
            float s = m.spec_exp;

            // Diffuse BRDF
            Spectrum diff = m.diffuse / M_PI;

            // Phong specular BRDF
            vec3 r = wo.reflect(n).normalized(); // or wo.reflect(n) depending on your convention
            float roi = std::max(0.0f, r.dot(wi));

            Spectrum spec = m.specular * ((s + 2.0f) / (2.0f * M_PI)) * std::pow(roi, s);
            // Full BRDF is sum of lobes
            shading = (diff + spec) * 0.5;

            // PDFs for each lobe
            float diff_factor = cosI / M_PI;
            float spec_factor = (s + 1.0f) * std::pow(roi, s) / (2.0f * M_PI);
            // Sampling weights (example: based on energy)
            float Ed = 0.5;
            float Es = 0.5;
            float sum = Ed + Es;
            float wd = (sum > 0.0f) ? Ed / sum : 0.5f;
            float ws = (sum > 0.0f) ? Es / sum : 0.5f;

            pdf_factor = wd * diff_factor + ws * spec_factor;
            is_delta   = false;
            break;
        }

        case IllumType::TRANSPARENT:
            pdf_factor = 0.0f;
            is_delta = true;
            break;

        default:
            shading = Spectrum(1,1,1);
            pdf_factor = 0.0f;
            is_delta = false;
            break;
    }
#if UNIFORM_SAMPLING_ONLY
    pdf_factor = 1.0f/(2 * M_PI);
#elif COSINE_SAMPLING_ONLY
    pdf_factor = cosI / M_PI;
#endif
    return shading;
}
}

