#include "core/scene.h"

#define MAX_BOUNCE 4
#define MAX_VAL 1e+9
#define BACKGROUND_COLOR Radiance(0,0,0)
#define NULL_RADIANCE Radiance(0,0,0)
#define EPSILON 1e-6

#define LIGHT_INTENSITY 5.0f
#define MIRROR_FALLOFF 0.8f
#define WEIGHT_DIRECT_LIGHT 0.5f
#define WEIGHT_INDIRECT_LIGHT 0.5f

#define RR_START_DEPTH 3
#define RR_TERMINATION_PROB 0.8f

#define GOTHROUGH false

#include <sstream>
#include <iomanip>
#include <algorithm>

// error metrics for report, varying number of params
// quantifying error in light transport algorithms
// try different materials, specular and diffuse
// move into MIS, if mixed then mix the pdf
// russian roullete
// path tracing
// only one sample for transparent materials
// 0.5 is not fixed, there exist different weights, see eric veach phd

Scene::Scene(Camera &c,
             vector<shared_ptr<Mesh>> &m,
             vector<shared_ptr<Light>> &l) : camera(c), meshes(m), lights(l)
{
}

Scene::~Scene()
{
}

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

inline Intersection Scene::testIntersection(const Ray& r) const
{
    Material m;
    float min_dist = 1e+9f;
    shared_ptr<Face> closest_found = nullptr;
    for(const auto &mesh : meshes)
    {
        shared_ptr<Face> closest_buffer = nullptr;
        auto t_found = mesh->hit(closest_buffer, r, min_dist);
        if(closest_buffer && t_found < min_dist)
        {
            min_dist = t_found;
            closest_found = closest_buffer;
            m = mesh->material;
        }
    }
    return Intersection{.missed=closest_found == nullptr,
                        .face = closest_found,
                        .point = r.at(min_dist),
                        .t = min_dist,
                        .material = m};
}

// Radiance evalBSDF(const Material &m,
//                 const vec3 &p,
//                 const vec3 &n,
//                 const vec3 &wi,
//                 const vec3 &w0,
//                 float &pdf_factor,
//                 bool &is_delta)
// {
//     Radiance shading;

//     switch (m.illum)
//     {
//     case IllumType::OPAQUE:
//         shading = m.diffuse/(M_PI);
//         pdf_factor = wi.dot(n)/M_PI;
//         break;
//     case IllumType::SPECULAR:
//     {
//         Radiance diff = m.diffuse /(M_PI);
//         vec3 r = w0.reflect(n).normalized();
//         Radiance spec = m.specular * std::pow(std::max(0.0f,r.dot(wi)), m.spec_exp);
//         shading = (spec * 0.7) + (diff * 0.3);

//         float roi = std::clamp(r.dot(wi),0.0f, 1.0f);

//         float spec_factor = (m.spec_exp + 2.0f) * std::pow(roi, m.spec_exp)/(2.0f* M_PI);

//         float diff_factor = wi.dot(n)/M_PI;

//         pdf_factor = spec_factor * 0.7 + diff_factor * 0.3;

//         is_delta = false;
//         break;
//     }
//     case IllumType::TRANSPARENT:
//         break;
//     default:
//         shading = Radiance(1,1,1);
//         break;
//     }
//     return shading;
// }


Radiance evalBSDF(const Material &m,
                  const vec3 &p,
                  const vec3 &n,
                  const vec3 &wi,
                  const vec3 &wo,
                  float &pdf_factor,
                  bool &is_delta)
{
    Radiance shading;

    float cosI = std::max(0.0f, n.dot(wi));

    switch (m.illum)
    {
    case IllumType::OPAQUE:
    {
        Radiance diff = m.diffuse / M_PI;
        shading = diff;
        pdf_factor = cosI / M_PI;
        is_delta = false;
        break;
    }

    case IllumType::SPECULAR:
    {
        float s = m.spec_exp;

        // Diffuse BRDF
        Radiance diff = m.diffuse / M_PI;

        // Phong specular BRDF
        vec3 r = wo.reflect(n).normalized(); // or wo.reflect(n) depending on your convention
        float roi = std::max(0.0f, r.dot(wi));

        Radiance spec = m.specular * ((s + 2.0f) / (2.0f * M_PI)) * std::pow(roi, s);
        // Full BRDF is sum of lobes
        shading = (diff + spec) * 0.5;

        // PDFs for each lobe
        float diff_factor = cosI / M_PI;
        float spec_factor = (s + 1.0f) * std::pow(roi, s) / (2.0f * M_PI);
        // Sampling weights (example: based on energy)
        float Ed = 0.3;
        float Es = 0.7;
        float sum = Ed + Es;
        float wd = (sum > 0.0f) ? Ed / sum : 0.5f;
        float ws = (sum > 0.0f) ? Es / sum : 0.5f;

        pdf_factor = wd * diff_factor + ws * spec_factor;
        is_delta   = false;
        break;
    }

    case IllumType::TRANSPARENT:
        // handled elsewhere via delta reflection/refraction
        pdf_factor = 0.0f;
        is_delta = true; // here it *is* delta
        break;

    default:
        shading = Radiance(1,1,1);
        pdf_factor = 0.0f;
        is_delta = false;
        break;
    }

    return shading;
}


float getLightFactor(shared_ptr<Face> geometry, const vec3 &p_light, const vec3 &p_surface)
{
    float area = geometry->getArea();
    vec3 n_light = geometry->get_normal(p_light).normalized() * -1.0f;
    float dist = (p_light - p_surface).length();
    float cos_y = n_light.dot((p_surface - p_light).normalized());
    return (dist*dist/cos_y)/area;
}

Radiance Scene::integrateNEE(const vec3 &w0, const Intersection &hit, const vec3 &n_hit, const int n_samples) const
{
    Radiance L;
    for(const auto &l : lights)
    {
        for(int i = 0; i < n_samples; i++)
        {
            vec3 sampled_point = l->geometry->generateUniform();

            float area = l->geometry->getArea();

            vec3 n_light = l->geometry->get_normal(sampled_point).normalized() * -1.0f;

            vec3 wi = (hit.point - sampled_point);
            float dist = wi.length();
            wi.normalize();

            float cos_omega = n_hit.dot(wi * -1.0f);    // cos between face normal and ray from light source
            float cos_y = n_light.dot(wi);              // cos between light normal and ray from light source

            if(cos_omega <= 0 || cos_y <= 0) continue;

            cos_omega = std::min(cos_omega, 1.0f);
            cos_y     = std::min(cos_y, 1.0f);

            Ray r_from_light(sampled_point + EPSILON * n_light, wi);

            Intersection intersection_light = testIntersection(r_from_light);

            if(intersection_light.face == hit.face || intersection_light.material.is_transparent)
            {
                float pdf_factor_light = (dist*dist/cos_y)/area;
                float pdf_factor_bsdf;
                bool is_delta;
                Radiance fr = evalBSDF(hit.material, hit.point, n_hit, wi, w0, pdf_factor_bsdf, is_delta);

                float msi_weight = (pdf_factor_light)/(pdf_factor_bsdf + pdf_factor_light);
                L += (fr * msi_weight) * cos_omega * l->color * LIGHT_INTENSITY;
            }
        }
    }
    return L/n_samples;
}

Radiance Scene::integrateIndirect(const vec3 &w0, const Intersection &hit, const vec3 &n_hit, int n_samples, int depth)
{
    float bsdf_factor;
    bool is_delta = false;

    vec3 wi = Sample::fromBSDF(n_hit, w0, hit.material);

    float cos_omega = wi.dot(n_hit);

    if(cos_omega < 0.0f) return NULL_RADIANCE;

    cos_omega = std::min(1.0f, cos_omega);

    vec3 sample_origin = hit.point + (EPSILON * n_hit);
    Ray sample_ray(sample_origin, wi);

    Radiance fr = evalBSDF(hit.material, hit.point, n_hit, wi, w0, bsdf_factor, is_delta);
    Radiance sampled_radiance;


    Intersection hit_indirect = testIntersection(sample_ray);
    float msi_weight = (1.0f/bsdf_factor);
    float light_factor;
    if(hit_indirect.material.is_lightsource)
    {
        sampled_radiance = hit_indirect.material.emittance * LIGHT_INTENSITY;
        light_factor = getLightFactor(hit_indirect.face,hit_indirect.point, sample_origin);
        msi_weight = bsdf_factor/(bsdf_factor + light_factor);
    }
    else
    {
        sampled_radiance = Li(sample_ray, n_samples, is_delta, depth + 1);
    }
    return (sampled_radiance * msi_weight) * fr * cos_omega;
}

Radiance Scene::L_mirror(const Ray &w0, const Intersection &hit, const vec3 &n_hit, const int n_samples, int depth)
{
    vec3 reflection_dir = w0.direction.reflect(n_hit).normalized();
    Ray r_reflect(hit.point + EPSILON * n_hit, reflection_dir);
    return Li(r_reflect, n_samples, true, depth + 1) * MIRROR_FALLOFF;
}

Radiance Scene::L_transparent(const vec3 &w0,
                              const Intersection &hit,
                              const vec3 &n_hit,
                              const int n_samples,
                              int depth)
{
    if(GOTHROUGH)
    {
        Ray passthrough(hit.point + w0 * EPSILON, w0);
        return Li(passthrough, n_samples, true, depth + 1);
    }
    // w0: ray *direction* (from camera to scene), so incoming at the surface is -w0
    vec3 wo = w0; // outgoing from surface toward camera? adjust to your convention

    vec3 nl = (wo.dot(n_hit) < 0.0f) ? n_hit : n_hit * -1.0f;

    bool outside = (wo.dot(n_hit) < 0.0f);
    vec3 bias = nl * EPSILON;
    vec3 origin = outside ? hit.point + bias : hit.point - bias;

    float ior = hit.material.index_of_ref;

    // Compute reflection dir
    vec3 reflectDir = wo.reflect(nl).normalized();

    // Compute refraction dir (use oriented normal nl!)
    vec3 refractDir = refractDirection(w0, nl, ior).normalized();

    // Fresnel using oriented normal nl
    float kr = fresnel(wo, nl, ior); // kr = reflectance

    // If refractDirection fails (TIR), just reflect
    if (refractDir.x == 0 && refractDir.y == 0 && refractDir.z == 0) {
        Ray reflectRay(origin, reflectDir);
        return Li(reflectRay, n_samples, true, depth + 1);
    }

    Ray reflectRay(origin, reflectDir);
    Ray refractRay(origin, refractDir);

    // Radiance Lr = Li(reflectRay, n_samples, true, depth + 1);
    Radiance Lr;
    Radiance Lt = Li(refractRay, n_samples, true, depth + 1);

    return Lr * kr + Lt * (1.0f - kr);
}

Radiance Scene::Li(const Ray &w0, int n_samples, bool is_delta, int depth)
{
    Intersection hit = testIntersection(w0);

    if(hit.missed) return NULL_RADIANCE;

    Radiance Le;

    if(hit.material.is_lightsource && (is_delta || depth == 0))
    {
        Le = hit.material.emittance * LIGHT_INTENSITY;
        return Le;
    }

    if(depth > MAX_BOUNCE) return Le;

    vec3 n_hit = hit.face->get_normal(hit.point).normalized();

    if(hit.material.illum == IllumType::MIRROR)
    {
        return L_mirror(w0, hit, n_hit, n_samples, depth);
    }
    if(hit.material.illum == IllumType::TRANSPARENT)
    {
        return L_transparent(w0.direction.normalized(), hit, n_hit, n_samples, depth);
    }

    float rr_scale = 1.0f;


    Radiance L_scene;
    static RandomNumberGenerator rng;

    if(depth >= RR_START_DEPTH)
    {

        if(rng.generate() > RR_TERMINATION_PROB) return Le;


        rr_scale = 1.0f / RR_TERMINATION_PROB;
    }


    if(rng.generate() > 0.5)
    {
        L_scene = integrateNEE(w0.direction.normalized() * -1.0f, hit, n_hit, n_samples);
    }
    else
    {
        L_scene = integrateIndirect(w0.direction.normalized() * -1.0f, hit, n_hit, n_samples, depth) * rr_scale;
    }



    Radiance L = Le + L_scene;

    return L;
}

void Scene::render(const char *filename,
                   const int width,
                   const int height,
                   const int n_sample)
{
  RandomNumberGenerator rng;
  FrameBuffer frame_buffer(width, height);
  FrameBuffer frame_tmp(width, height);

  int spp = n_sample;
  std::vector<Radiance> L(height * width);
  // for(auto k = 0; k < spp; k++)
  // {
  //     #pragma omp parallel for collapse(2)
  //     for(auto i = 0; i < height; i++)
  //     {
  //         for(auto j = 0; j < width; j++)
  //         {
  //             Ray traced_ray(camera.origin, camera.pixelToWorldSpace(i,j, rng.generate(), rng.generate()));
  //             L[i*width + j] += Li(traced_ray, n_sample,false, 0) * (1.0f/spp);
  //             frame_buffer.set(i,j,L[i*width + j]);
  //             auto color = frame_buffer.getColor(i,j);
  //             frame_tmp.set(i,j,color);
  //         }
  //     }
  //     frame_tmp.clamp();
  //     std::ostringstream ss;
  //     ss << "f" << std::setw(4) << std::setfill('0') << k << ".ppm";
  //     std::string filename = ss.str();
  //     string name = "result/" + filename;
  //     frame_tmp.writeToPPM(name.c_str(), width, height);
  // }
  #pragma omp parallel for collapse(3)
  for(auto k = 0; k < spp; k++)
  {
      for(auto i = 0; i < height; i++)
      {
          for(auto j = 0; j < width; j++)
          {
              Ray traced_ray(camera.origin, camera.pixelToWorldSpace(i,j, rng.generate(), rng.generate()));
              L[i*width + j] += Li(traced_ray, n_sample, false, 0) * (1.0f/spp);
          }
      }
  }
  for(auto i = 0; i < height; i++)
  {
      for(auto j = 0; j < width; j++){
          frame_buffer.set(i,j,L[i*width + j]);
      }
  }
  frame_buffer.clamp();
  frame_buffer.writeToPPM(filename, width, height);

}
