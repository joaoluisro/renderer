#include "scene.h"

#define cimg_display 0  // (no X11 needed)
#include "CImg-3.5.4/CImg.h"
using namespace cimg_library;


Scene::~Scene()
{
}

void write_to_ppm(const char* filename, Framebuffer &frame_buffer, int width, int height)
{
  frame_buffer.clamp();
  CImg<unsigned char> image(width, height, 1, 3);
  for(auto i = 0; i < width; i++)
  {
    for(auto j = 0; j < height; j++)
    {
      image(i,j,0) = frame_buffer.at(i,j).r;
      image(i,j,1) = frame_buffer.at(i,j).g;
      image(i,j,2) = frame_buffer.at(i,j).b;
    }
  }
  image.save(filename);
}

void get_fragment_color(Vector3D &p,
                        Vector3D &camera_origin, 
                        shared_ptr<BaseObject> obj, 
                        Color& final_color, 
                        vector<shared_ptr<Light>> scene_lights)
{
  Vector3D V = (camera_origin - p).normalized();
  float shininess = 1;
  /* ambient term */
  final_color = obj->get_color() * 0.05;

  for(const auto &light : scene_lights)
  {
    Vector3D L = (light->position - p).normalized();
    Vector3D N = obj->get_normal(p).normalized();
    float ndotl = N.dot(L);
    if(ndotl <= 0.0f) return;

    // Diffuse component
    Color diff = obj->get_color() * light->color * ndotl;

    // Specular component
    Vector3D R = ((2 * ndotl) * N - L).normalized();
    float specAngle = max(R.dot(V), 0.0f);
    Color spec = obj->get_color() * light->color * pow(specAngle, shininess);
    
    final_color += diff + spec;
  }

}

bool Scene::render(const char *filename, int width, int height)
{
  Framebuffer frame_buffer(width, height);

  #pragma omp parallel for collapse(2)
  for(auto i = 0; i < width; i++)
  {
    for(auto j = 0; j < height; j++)
    {
      auto shooting_dir = camera.pixelToWorldSpace(i,j);
      Ray r(camera.origin, shooting_dir);
      Color final_color(0,0,0);
      
      float min_dist = 1e+5;
      bool hit = false;
      shared_ptr<BaseObject> closest;
      int count = 0;
      for(auto &obj : scene_objs)
      {
        float t = obj->intersects(r);
        if(t > 0 && t < min_dist)
        {
          hit = true;
          count++;
          min_dist = t;
          closest = obj;
        }
      }
      if(hit)
      {
        auto p = r.at(min_dist);
        get_fragment_color(p, camera.origin, closest, final_color, scene_lights);
      }
      frame_buffer.set(i,j,final_color);
    
    }
  }

  write_to_ppm(filename, frame_buffer, width, height);
  return true;
}
