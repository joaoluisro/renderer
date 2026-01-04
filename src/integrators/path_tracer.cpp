#include "integrators/path_tracer.h"

#include <sstream>
#include <iomanip>

PathTracer::~PathTracer()
{
}

float getLightFactor(shared_ptr<Face> geometry,
                     const vec3 &p_light,
                     const vec3 &p_surface)
{
    vec3 n_light = geometry->getNormal(p_light).normalized() * -1.0f;
    float dist = (p_light - p_surface).length();
    float cos_y = n_light.dot((p_surface - p_light).normalized());
    return (dist*dist/cos_y);
}

Spectrum PathTracer::LDirect(const vec3 &wo,
                             const Intersection &hit,
                             const vec3 &n_hit)
{
    Spectrum L;

    std::shared_ptr<Light> light = scene->sampleLight();

    const vec3 sample_pos   = light->geometry->generateUniform();
    const float light_area = light->geometry->getArea();

    const vec3 n_light = (light->geometry->getNormal(sample_pos)).normalized() * -1.0f;

    // Direction from light to shading point
    vec3 wi = sample_pos - hit.point;
    const float distance = wi.length();
    wi.normalize();

    const float cos_on_surface = std::min(1.0f, n_hit.dot(wi));   // between surface normal and light→hit
    const float cos_on_light   = std::min(1.0f, n_light.dot(wi * -1.0f));  // between light normal and light→hit

    // Light or surface facing away → no contribution
    if (cos_on_surface <= 0.0f || cos_on_light <= 0.0f) return L;

    const Ray shadow_ray(sample_pos + EPSILON * n_light, wi * -1.0f);
    const Intersection shadow_hit = scene->intersectionTest(shadow_ray);

    // Visible if we hit the same surface, or a transparent one
    const bool visible = (shadow_hit.face == hit.face) || shadow_hit.material.is_transparent;

    if (!visible) return L;

    const float G = (distance * distance/ cos_on_light) / light_area;
    float pdf_light_solid = 1.0/light_area;
    pdf_light_solid *= G;

    float pdf_bsdf = 0.0f;
    bool is_delta  = false;

    const Spectrum fr = BSDF::eval(
        hit.material,
        hit.point,
        n_hit,
        wi,
        wo,
        pdf_bsdf,
        is_delta
        );

    if(pdf_bsdf <= 0.0f) return L;

    const float mis_weight = (pdf_light_solid) / (pdf_bsdf + (pdf_light_solid));

    return fr * mis_weight * (1.0f/pdf_light_solid) * cos_on_surface * light->color * LIGHT_INTENSITY;
}


Spectrum PathTracer::LIndirect(const vec3 &wo,
                                const Intersection &hit,
                                const vec3 &n_hit,
                                int depth)
{
    bool is_delta = false;
    float pdf_bsdf = 0.0f;

    const vec3 wi = Sample::fromBSDF(n_hit, wo, hit.material);

    float cos_wi = wi.dot(n_hit);

    cos_wi = std::max(0.0f, cos_wi);


    const Spectrum fr = BSDF::eval(
        hit.material,
        hit.point,
        n_hit,
        wi,
        wo,
        pdf_bsdf,
        is_delta);

    if(pdf_bsdf <= 0.0f) return NULL_RADIANCE;

    // Offset origin to avoid self-intersection
    const vec3 sample_origin = hit.point + EPSILON * n_hit;
    const Ray shadow_ray(sample_origin, wi);

    const Intersection bounce_hit = scene->intersectionTest(shadow_ray);


    // If we don't hit a light, no MIS with light sampling: only BSDF sampling
    if (!bounce_hit.material.is_lightsource)
    {
        Spectrum Li = traceRay(shadow_ray, is_delta, depth + 1);
        return Li * (fr * cos_wi / pdf_bsdf);
    }

    // Hit a light: MIS with light sampling
    Spectrum Li = bounce_hit.material.emittance * LIGHT_INTENSITY;

    const float area      = bounce_hit.face->getArea();
    const float pdf_light_A = 1.0f / area;

    const vec3 light_pos  = bounce_hit.point;
    const vec3 n_light    = bounce_hit.face->getNormal(light_pos).normalized() * -1.0f;
    const float distance2 = (light_pos - sample_origin).length();
    const float cos_light = std::max(0.0f, n_light.dot(wi * -1.0f));

    if (cos_light <= 0.0f) return NULL_RADIANCE;

    const float pdf_light_ω = pdf_light_A * distance2 / cos_light;
    const float pdf_bsdf_ω  = pdf_bsdf;

    const float w_bsdf = pdf_bsdf_ω / (pdf_bsdf_ω + pdf_light_ω);

    return Li * fr * cos_wi * w_bsdf / pdf_bsdf_ω;
}

Spectrum PathTracer::LMirror(const vec3 &w0,
                              const Intersection &hit,
                              const vec3 &n_hit,
                              int depth)
{

    vec3 reflection_dir = w0.reflect(n_hit).normalized();
    Ray r_reflect(hit.point + EPSILON * n_hit, reflection_dir);
    return traceRay(r_reflect, true, depth + 1) * MIRROR_FALLOFF;
}

Spectrum PathTracer::traceRay(const Ray &r, bool is_delta, int depth)
{
    Intersection hit = scene->intersectionTest(r);

    if(hit.missed) return NULL_RADIANCE;

    Spectrum Le, Li;

    if(hit.material.is_lightsource && (is_delta || depth == 0))
    {
        Le = hit.material.emittance * LIGHT_INTENSITY;
        return Le;
    }

    if(depth > MAX_BOUNCE) return Spectrum(1,1,1);

    vec3 n_hit = hit.face->getNormal(hit.point).normalized();

    if(hit.material.illum == IllumType::MIRROR) return LMirror(r.direction * -1.0f, hit, n_hit, depth);

    return Le +
           LDirect(r.direction.normalized() * -1.0f, hit, n_hit) +
           LIndirect(r.direction.normalized() * -1.0f, hit, n_hit, depth);

}

void PathTracer::render(const int spp, FrameBuffer &fb)
{
    // cout << scene->camera.origin << "\n";
    RandomNumberGenerator rng;

    int height = scene->camera.height;
    int width = scene->camera.width;
    auto camera = scene->camera;
    std::vector<Spectrum> L(height * width);
    FrameBuffer frame_tmp(width, height);

    for(auto k = 0; k < spp; k++)
    {
        #pragma omp parallel for collapse(2)
        for(auto i = 0; i < height; i++)
        {
            for(auto j = 0; j < width; j++)
            {
                Ray traced_ray(camera.origin, camera.toWorldSpace(i,j, rng.generate(), rng.generate()));
                L[i*width + j] += traceRay(traced_ray,false, 0) * (1.0f/spp);
                frame_tmp.set(i,j,L[i*width + j]);
            }
        }

        frame_tmp.clamp();
        std::ostringstream ss;
        ss << "f" << std::setw(4) << std::setfill('0') << k << ".ppm";
        std::string filename = ss.str();
        string name = "analysis-lucy/cosine/" + filename;
        frame_tmp.writeToPPM(name.c_str(), width, height);
    }

    // #pragma omp parallel for collapse(3)
    // for(auto k = 0; k < spp; k++)
    // {
    //     for(auto i = 0; i < height; i++)
    //     {
    //         for(auto j = 0; j < width; j++)
    //         {
    //             Ray r(scene->camera.origin, scene->camera.toWorldSpace(i,j, rng.generate(), rng.generate()));
    //             L[i*width + j] += traceRay(r, false, 0)/spp;
    //         }
    //     }
    // }
    // for(auto i = 0; i < height; i++)
    // {
    //     for(auto j = 0; j < width; j++){
    //         fb.set(i,j,L[i*width + j]);
    //     }
    // }
}
