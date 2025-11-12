#include "core/scene.h"

#define MAX_BOUNCE 2
#define MAX_VAL 1e+9
#define BACKGROUND_COLOR Color(1,1,1)
#define NULL_RADIANCE Color(0,0,0)
#define EPSILON 1e-5
#define REFLECT false
#include <algorithm>

#include <cmath>

Scene::Scene(Camera &c,
             vector<shared_ptr<Mesh>> &m,
             vector<shared_ptr<Light>> &l) : camera(c), meshes(m), lights(l)
{
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0.0, 1.0);
    this->gen = gen;
    this->dis = dis;
}

Scene::~Scene()
{
}

inline float Scene::intersects(shared_ptr<Face> &closest,
                               Material &m,
                               const Ray& r) const
{
    float min_dist = 1e+9;
    shared_ptr<Face> closest_found = nullptr;
    for(const auto &mesh : meshes)
    {
        shared_ptr<Face> closest_buffer = nullptr;
        auto t_found = mesh->hit(closest_buffer, r);
        if(closest_buffer && t_found < min_dist)
        {
            min_dist = t_found;
            closest_found = closest_buffer;
            m = mesh->material;
        }
    }
    if(closest_found)
    {
        closest = closest_found;
        return min_dist;
    }
    return -1;
}

/*
Vector3D refractDirection(const Vector3D &dir, const Vector3D &n, const float &ior)
{
    float eta_interface = ior;
    float eta_medium = 1.0;

    // if n.dot(r) < 0 => medium from ray to interface
    // else            => medium from interface to ray
    // we need to flip the normal to account for hitting the "back" of a face.
    float ndotr = n.dot(dir);
    bool is_front = ndotr < 0.0;

    Vector3D normal_refract = n;
    if(is_front)
    {
        ndotr = -ndotr;
    }
    else
    {
        normal_refract = n * -1;
        std::swap(eta_interface, eta_medium);
    }
    float eta = eta_medium/eta_interface;
    float k = 1 - eta*eta * (1.0 - ndotr*ndotr);

    // total internal reflection case
    if(k < 0)
    {
        return Vector3D(0,0,0);
    }

    Vector3D transmission_dir = eta * dir + (eta * ndotr - sqrtf(k)) * normal_refract ;
    return transmission_dir;
}

void fresnel(const Vector3D &I, const Vector3D &N, const float &ior, float &kr)
{
    float cosi = std::clamp(I.dot(N),-1.0f, 1.0f);
    float etai = 1, etat = ior;
    if (cosi > 0) { std::swap(etai, etat); }
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.0f, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1) {
        kr = 1;
    }
    else {
        float cost = sqrtf(std::max(0.0f, 1 - sint * sint));
        cosi = fabs(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        kr = (Rs * Rs + Rp * Rp) / 2;
    }
}

Color Scene::traceSampledRay(const Ray &r, const Vector3D &p,  const Vector3D &n,const shared_ptr<Face> f, int &hit_count) const
{
    shared_ptr<Face> closest_face = nullptr;
    float t = intersects(closest_face, r);
    Color L = NULL_RADIANCE;

    if(t < EPSILON || closest_face->material().illum != AREA_LIGHT) return NULL_RADIANCE;

    hit_count++;
    auto distance = (r.at(t) - p).length();
    auto falloff = 1/(distance);
    Color light_color = closest_face->material().diffuse;
    Color diff = f->material().diffuse;
    Color spec = f->material().specular;
    auto reflection_vector = n.reflect(r.direction);
    float spec_angle = max(reflection_vector.dot(r.direction), 0.0f);

    L += diff * light_color;
    L *= falloff;
    L *= 10;
    return L;
}
*/
Vector3D generateUniformSample(float eta1, float eta2)
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
Vector3D generateCosineWeightedSample(const float eta1, const float eta2)
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

Color Scene::integrateNEE(const Ray &r, int n_samples, int depth)
{
    /* reached max recursion depth, return no radiance */
    if(depth > MAX_BOUNCE) return NULL_RADIANCE;

    bool is_primary_ray = depth == 0;

    shared_ptr<Face> hit = nullptr;
    Material hit_material;
    float t = intersects(hit, hit_material, r);

    /* ray doesnt hit anything, no radiance */
    if(t < EPSILON) return NULL_RADIANCE;

    /* primary ray hits light source */
    if(hit_material.illum == AREA_LIGHT && is_primary_ray) return Color(1,1,1) * 10.0f;

    Vector3D p = r.at(t);
    Vector3D n = hit->get_normal(p).normalized();

    /* flip normal when primary ray so it always faces the camera */
    if(is_primary_ray)
    {
        if(r.direction.dot(n) > 0.0f) n = n * -1.0f;
    }
    Color L(0,0,0);

    // /* Account for direct lighting */
    Color L_direct(0,0,0);
    for(auto &l : lights)
    {
        for(int i = 0; i < n_samples; i++)
        {
            float eta1 = this->dis(this->gen);
            float eta2 = this->dis(this->gen);
            auto sample_point = l->geometry->generateUniform(eta1, eta2);
            float area = l->geometry->getArea();
            Vector3D n_light = l->geometry->get_normal(sample_point).normalized();
            n_light = n_light * -1.0f;
            Vector3D w = (p - sample_point);
            float dist = max(1.0f,w.length());
            w.normalize();

            float cos_x = n.dot(w * -1.0f);    // cos between face normal and ray from light source
            float cos_y = n_light.dot(w);   // cos between light normal and ray from light source

            shared_ptr<Face> hit_face;
            Ray r_visible(sample_point + EPSILON * n_light, w);
            Material m;
            float t_from_face = intersects(hit_face, m, r_visible);
            if(hit_face == hit)
            {
                float p_w = (1.0f/area)*(dist*dist/cos_y);
                L_direct += l->color * hit_material.diffuse * cos_x * (1.0f/p_w) * 15.0f * 0.5;
            }
        }
    }

    /* Account for indirect, only consider opaque surfaces */
    Color L_indirect(0,0,0);
    for(int i = 0; i < n_samples; i++)
    {
        float eta1 = this->dis(this->gen);
        float eta2 = this->dis(this->gen);

        // Vector3D v = generateUniformSample(eta1, eta2);
        Vector3D sample_dir = generateCosineWeightedSample(eta1, eta2);

        float vdotn = sample_dir.dot(n);
        /* generated ray is facing the wrong direction, flip it. */
        if(vdotn < EPSILON)
        {
            sample_dir = sample_dir * -1.0f;
            vdotn = -vdotn;
        }

        float sin_theta = sqrt(1.0f - (vdotn*vdotn));
        float pdf_factor = (sin_theta * vdotn)/(M_PI);

        Vector3D sample_origin = p + (EPSILON * n);
        Ray sample_ray(sample_origin, sample_dir);

        shared_ptr<Face> sample_hit = nullptr;
        Material sample_material;
        float t_hit = intersects(sample_hit,sample_material, sample_ray);

        if(t_hit > EPSILON)
        {
            float dist = (p - r.at(t_hit)).length();
            float falloff = std::min(1.0f, 1.0f/(dist * dist));
            if(sample_material.illum == AREA_LIGHT)
            {
                auto Le = hit_material.diffuse;
                L_indirect += Le * falloff * vdotn * (1.0f/pdf_factor) * 15.0f * 0.5;
            }
            else
            {
                Ray r_reflected(sample_origin, sample_dir * -1);
                // auto reflect_dir = n.reflect(r.direction);
                // float rdotn = max(0.0f, reflect_dir.dot(sample_dir));
                L_indirect += integrateNEE(r_reflected, n_samples, depth + 1) * vdotn * (1.0f/pdf_factor);
            }
        }
    }
    L = L_direct;
    return L * (1.0f/(2.0f * n_samples));
}

/*
inline Color Scene::integrate(const Ray &r, int n_samples, int depth) const
{

    if(depth > MAX_BOUNCE) return NULL_RADIANCE;

    shared_ptr<Face> closest_face = nullptr;
    float t = intersects(closest_face, r);

    if(t < EPSILON) return BACKGROUND_COLOR;

    Vector3D intersection_point = r.at(t);
    Vector3D n = closest_face->get_normal(intersection_point).normalized();


    if(closest_face->material().illum == AREA_LIGHT && depth == 0)
    {
        auto dist = (r.origin - r.at(t)).length();
        return Color(1,1,1) * 5.0f;
    }

    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // generate even samples on the hemisphere, only account for opaque surfaces
    // #pragma omp parallel for reduction(+:L)
    Color L(0,0,0);
    int sample_count = n_samples/(pow(2,depth));
    for(int i = 0; i < n_samples; i++)
    {
        float eta1 = dis(gen);
        float eta2 = dis(gen);

        // Vector3D v = generateUniformSample(eta1, eta2);
        Vector3D v = generateCosineWeightedSample(eta1, eta2);

        if(r.direction.dot(n) > 0.0f) n = n * -1;

        auto vdotn = v.dot(n);
        if(vdotn < 0.0f)
        {
            v = v * -1.0f;
            vdotn = -vdotn;
        }

        float sin_theta = sqrt(1.0f - (vdotn*vdotn));
        float den = (sin_theta*vdotn)/(M_PI);

        Vector3D sample_origin = intersection_point + (EPSILON * n);
        Ray r_sample(sample_origin, v);

        shared_ptr<Face> hit = nullptr;
        float t_hit = intersects(hit, r_sample);

        if(t_hit > EPSILON)
        {
            float dist = (hit->get_normal(r.at(t_hit)) - n).length();
            float falloff = 1.0f/(dist*dist);
            if(hit->material().illum == AREA_LIGHT)
            {
                auto Le = hit->material().diffuse * 10;
                L += closest_face->material().diffuse * falloff * Le * vdotn * (1.0f/den);
            }
            else
            {
                Ray r_reflected(sample_origin, v * -1);
                L += integrate(r_reflected, n_samples, depth + 1) * vdotn * (1.0f/den);
            }
        }
    }
    return L * (1.0f/n_samples);
}

*/

/*
inline Color Scene::traceRay(const Ray &r,
                             int depth) const
{
    if(depth <= 0) return BACKGROUND_COLOR;

    shared_ptr<Face> closest_face = nullptr;
    float t = intersects(closest_face, r);

    if(t < EPSILON) return BACKGROUND_COLOR;

    Vector3D intersection_point = r.at(t);
    Vector3D shadow_ray_direction = r.direction.normalized() * -1;
    Color L(0,0,0);
    Vector3D n = closest_face->get_normal(intersection_point).normalized();

    switch (closest_face->material().illum)
    {
        case OPAQUE:
        {
            L += shadeOpaque(shadow_ray_direction, closest_face, intersection_point);
            break;
        }

        case MIRROR:
        {
            Vector3D reflection_dir = shadow_ray_direction.reflect(closest_face->get_normal(intersection_point));
            Ray reflection_ray(intersection_point, reflection_dir.normalized() * -1);
            L += traceRay(reflection_ray, depth - 1) * 0.8;
            break;

        }

        case TRANSPARENT:
        {
            Color refractionColor(0,0,0);
            Color reflectionColor(0,0,0);
            float kr;
            Vector3D nl = (r.direction.dot(n) < 0.0) ? n : n * -1;

            fresnel(r.direction,nl,closest_face->material().index_of_ref,kr);
            bool outside = r.direction.dot(n) < 0;
            Vector3D bias = nl * EPSILON;
            // there is refraction
            if(kr < 1)
            {
                auto refractDir = refractDirection(r.direction, nl, closest_face->material().index_of_ref);
                refractDir.normalize();
                Vector3D refractOrigin = outside ? intersection_point + bias : intersection_point - bias;
                Ray refractRay(intersection_point + refractDir * EPSILON, refractDir);
                refractionColor = traceRay(refractRay, depth - 1);
            }
            Vector3D reflectionDir = r.direction.reflect(nl).normalized();
            Vector3D reflectOrigin = outside ? intersection_point + bias : intersection_point - bias;
            Ray reflectRay(intersection_point + reflectionDir * EPSILON, reflectionDir);
            reflectionColor = traceRay(reflectRay, depth - 1);

            // mix the two
            L += (reflectionColor * kr) + refractionColor * (1 - kr);
            // L += refractionColor;
            break;
        }
        default:
        {
            return BACKGROUND_COLOR;
            break;
        }
    }

    return L;
}
*/
void Scene::render(const char *filename,
                   int width,
                   int height,
                   int n_sample)
{
  FrameBuffer frame_buffer(width, height);
  long unsigned int total_leaf = 0;
  long unsigned int total_box = 0;

  #pragma omp parallel for collapse(2)
  for(auto i = 0; i < height; i++)
  {
    for(auto j = 0; j < width; j++)
    {
      auto shooting_dir = camera.pixelToWorldSpace(i,j);
      
      Ray r(camera.origin, shooting_dir);

      auto final_color = integrateNEE(r, n_sample, 0);
      frame_buffer.set(i,j,final_color);
    }
  }
  frame_buffer.clamp();
  frame_buffer.writeToPPM(filename, width, height);
}
