#include "core/scene.h"

#define MAX_BOUNCE 3
#define MAX_VAL 1e+9
#define BACKGROUND_COLOR Color(0.07,0.07,0.07)
#define EPSILON 1e-8
#define REFLECT false

Scene::~Scene()
{
}

inline bool Scene::isShadowed(Vector3D &light_origin, 
  Vector3D &p,
  shared_ptr<BaseObject> &face) const
{
  auto light_ray_direction = (p - light_origin).normalized();

  Ray shadow_ray(light_origin,light_ray_direction);

  shared_ptr<BaseObject> other_face;

  auto t = hit(other_face, shadow_ray);

  if(t > 0 && other_face != face)
  {
    return true;
  } 
  return false;
}

inline double Scene::hit(shared_ptr<BaseObject> &closest, Ray r) const
{
    double min_dist = 1e+9;
    shared_ptr<BaseObject> closest_found = nullptr;
    for(const auto &mesh : scene_meshes)
    {
        shared_ptr<BaseObject> closest_buffer = nullptr;
        auto t_found = mesh->hit(closest_buffer, r);
        if(closest_buffer && t_found < min_dist)
        {
            min_dist = t_found;
            closest_found = closest_buffer;
        }
    }
    if(closest_found)
    {
        closest = closest_found;
        return min_dist;
    }
    return -1;
}



inline Color Scene::shade(Vector3D dir, shared_ptr<BaseObject> face, Vector3D p, double &distance) const
{
    // Ambient component
    auto ambient_color = face->material().ambient;
    auto final_color = ambient_color;

    for(const auto &light : scene_lights)
    {
        Vector3D L = (light->position - p);
        distance = L.length();
        L.normalize();
        Vector3D N = face->get_normal(p).normalized();
        double ndotl = N.dot(L);
        if(ndotl <= 0.0) continue;

        if(isShadowed(light->position, p, face)) continue;

        // Diffuse component
        Color diff = (face->material().diffuse * ndotl) * light->color ;
        // Specular component
        auto reflection_vector = ((2 * ndotl) * N - L).normalized();

        double specAngle = max(reflection_vector.dot(dir), 0.0);
        Color spec = (face->material().specular * pow(specAngle, face->material().spec_exp)) * light->color;
        final_color += diff + spec;
    }
    return final_color;
}

inline Color Scene::traceRay(Ray &r, int depth) const
{
  shared_ptr<BaseObject> closest = nullptr;

  auto t = hit(closest, r);
  // early termination, ray doesnt hit anything
  if(t < EPSILON)
  {
    return BACKGROUND_COLOR;
  }
  auto p = r.at(t);
  Vector3D reflection_dir;
  double distance = 0;
  Vector3D shade_dir = r.direction.normalized() * -1;
  auto radiance = shade(shade_dir, closest, p, distance);
  // radiance *= (1/(distance*distance));

  if(depth <= 0)
  {
    return radiance;
  }

  reflection_dir = shade_dir - 2*(shade_dir.dot(closest->get_normal(p).normalized())) * closest->get_normal(p).normalized();
  Ray reflection_ray(p, reflection_dir * -1);
  return (radiance) + (traceRay(reflection_ray, depth - 1) * 0.1);
}

void Scene::render(const char *filename, int width, int height)
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

      auto final_color = traceRay(r, MAX_BOUNCE);
      frame_buffer.set(i,j,final_color);
    }
  }
  frame_buffer.clamp();
  frame_buffer.writeToPPM(filename, width, height);
}
