#include "core/scene.h"

#define MAX_BOUNCE 1
#define MAX_VAL 1e+9
#define BACKGROUND_COLOR Color(1,1,1)
#define NULL_RADIANCE Color(0,0,0)
#define EPSILON 1e-6

#define LIGHT_INTENSITY 15.0f
#define MIRROR_FALLOFF 0.8f
#define WEIGHT_DIRECT_LIGHT 0.5f
#define WEIGHT_INDIRECT_LIGHT 0.5f
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

Vector3D refractDirection(const Vector3D &I, const Vector3D &N, float ior)
{
    Vector3D dir = I.normalized();
    Vector3D n   = N.normalized();

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
        return Vector3D(0, 0, 0);
    }

    return eta * dir + (eta * cosi - sqrtf(k)) * n;
}

void fresnel(const Vector3D &I, const Vector3D &N, const float &ior, float &kr)
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
        kr = 1.0f;  // Total internal reflection
        return;
    }

    float cost = sqrtf(std::max(0.0f, 1.0f - sint * sint));
    cosi = std::fabs(cosi);
    float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
    float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
    kr = (Rs * Rs + Rp * Rp) * 0.5f;
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

Color brdf(Material m,
           Vector3D p,
           Vector3D n,
           Ray r)
{
    Color shading;
    switch (m.illum)
    {
    case IllumType::OPAQUE:
        shading = m.diffuse * (1.0f/M_PI);

        break;
    case IllumType::SPECULAR:
    {
        Color diff = m.diffuse * (1.0f/M_PI);
        Color spec = m.specular * std::pow(std::max(0.0f,n.dot(r.direction * -1)), m.spec_exp);
        shading = (diff * 0.5) + (spec * 0.5);
        break;
    }
    case IllumType::TRANSPARENT:
        std::cout << " transparency !\n";
        break;
    default:
        shading = Color(1,1,1);
        break;
    }
    return shading;
}

// L0(p <- w0) =

Color Scene::Li(const Ray &w0, int n_samples, int depth)
{
    Intersection hit = testIntersection(w0);

    if(hit.missed) return NULL_RADIANCE;

    Color Le(0,0,0);

    if(hit.material.is_lightsource && depth == 0)
    {
        Le = hit.material.emittance * LIGHT_INTENSITY;
        return Le;
    }

    if(depth > MAX_BOUNCE) return Le;

    Vector3D n_hit = hit.face->get_normal(hit.point).normalized();

    /* Mirror hit, just reflect */
    if(hit.material.illum == IllumType::MIRROR)
    {
        Vector3D reflection_dir = w0.direction.reflect(n_hit).normalized();
        Ray r_reflect(hit.point + EPSILON * n_hit, reflection_dir);
        return Li(r_reflect, n_samples, depth + 1) * MIRROR_FALLOFF;
    }
    /* Create refracted and reflected ray */
    if(hit.material.illum == IllumType::TRANSPARENT)
    {
        Color refractionColor(0,0,0);
        Color reflectionColor(0,0,0);
        float kr;
        Vector3D nl = (w0.direction.dot(n_hit) < 0.0) ? n_hit : n_hit * -1;

        fresnel(w0.direction, n_hit, hit.material.index_of_ref, kr);
        bool outside = w0.direction.dot(n_hit) < 0;
        Vector3D bias = nl * EPSILON;
        // there is refraction
        if(kr < 1)
        {
            auto refractDir = refractDirection(w0.direction, n_hit, hit.material.index_of_ref);
            refractDir.normalize();
            Vector3D refractOrigin = outside ? hit.point + bias : hit.point - bias;
            Ray refractRay(refractOrigin, refractDir);
            refractionColor = Li(refractRay, n_samples, depth + 1);
        }
        Vector3D reflectionDir = w0.direction.reflect(nl).normalized();
        Vector3D reflectOrigin = outside ? hit.point + bias : hit.point - bias;
        Ray reflectRay(reflectOrigin, reflectionDir);
        reflectionColor = Li(reflectRay, n_samples, depth + 1);

        auto transparent_color = (reflectionColor * kr) + refractionColor * (1 - kr);
        return transparent_color;
    }


    Color L_indirect, L_direct;
    for(const auto &l : lights)
    {
        for(int i = 0; i < n_samples; i++)
        {
            Vector3D sampled_point = l->geometry->generateUniform();

            float area = l->geometry->getArea();

            Vector3D n_light = l->geometry->get_normal(sampled_point).normalized() * -1.0f;

            Vector3D w = (hit.point - sampled_point);
            float dist = w.length();
            w.normalize();

            float cos_omega = n_hit.dot(w * -1.0f);    // cos between face normal and ray from light source
            float cos_y = n_light.dot(w);              // cos between light normal and ray from light source

            if(cos_omega <= 0 || cos_y <= 0) continue;

            cos_omega = std::min(cos_omega, 1.0f);
            cos_y     = std::min(cos_y, 1.0f);

            Ray r_from_light(sampled_point + EPSILON * n_light, w);

            Intersection intersection_light = testIntersection(r_from_light);

            if(intersection_light.face == hit.face)
            {
                float p_w = (1.0f/area)*(dist*dist/cos_y);
                L_direct += brdf(hit.material, hit.point, n_hit, r_from_light) * cos_omega * l->color * LIGHT_INTENSITY  * (1.0f/p_w);
            }
        }
    }

    for(int i = 0; i < n_samples; i++)
    {
        Vector3D wi = Sampler::cosineWeighted(n_hit);

        float cos_omega = wi.dot(n_hit);

        if(cos_omega < 0.0f) continue;

        cos_omega = std::min(1.0f,cos_omega);

        Vector3D sample_origin = hit.point + (EPSILON * n_hit);
        Ray sample_ray(sample_origin, wi);
        float pdf_factor = cos_omega/M_PI;

        Color fr = brdf(hit.material, hit.point, n_hit, sample_ray);

        L_indirect += Li(sample_ray, n_samples, depth + 1) * fr * cos_omega * (1.0f/pdf_factor);
    }

    Color L;
    L = Le + (L_direct * (1.0f/n_samples)) + (L_indirect * (1.0f/n_samples));

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

  int spp = 128;
  std::vector<Color> L(height * width);
  // for(auto k = 0; k < spp; k++)
  // {
  //     #pragma omp parallel for collapse(2)
  //     for(auto i = 0; i < height; i++)
  //     {
  //         for(auto j = 0; j < width; j++)
  //         {
  //             Ray traced_ray(camera.origin, camera.pixelToWorldSpace(i,j, rng.generate(), rng.generate()));
  //             L[i*width + j] += Li(traced_ray, n_sample, 0) * (1.0f/spp);
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
  // std::vector<Color> L(height * width);
  #pragma omp parallel for collapse(3)
  for(auto k = 0; k < spp; k++)
  {
      for(auto i = 0; i < height; i++)
      {
          for(auto j = 0; j < width; j++)
          {
              Ray traced_ray(camera.origin, camera.pixelToWorldSpace(i,j, rng.generate(), rng.generate()));
              L[i*width + j] += Li(traced_ray, n_sample, 0) * (1.0f/spp);
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
