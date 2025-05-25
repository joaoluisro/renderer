#include "scene.h"

#define cimg_display 0  // (no X11 needed)
#include "CImg-3.5.4/CImg.h"
using namespace cimg_library;

Scene::~Scene()
{
}

void render_object(Vector3D &p,
                  Vector3D &camera_origin, 
                  shared_ptr<BaseObject> obj, 
                  Color& final_color, 
                  vector<shared_ptr<Vector3D>> scene_lights)
{
  /* ambient term */
  auto ambient_component = obj->get_color();
  final_color = Color(0,0,0);
  for(auto light : scene_lights)
  {
    /* diffuse component */
    auto n = obj->get_normal(p);
    n.normalize();

    auto l = *light - n;
    l.normalize();
    float alpha = max(0.0f,n.dot(l));

    auto diff_component = obj->get_color();
    diff_component *= alpha;

    /* specular component */
    auto v = camera_origin - p;
    v.normalize();
    
    auto r = (2*n.dot(l))*n - l;
    float beta = max(0.0f,r.dot(v));
    auto spec_component = Color(1.0f,1.0f,1.0f);
    spec_component *= beta * 0.5;
    
    final_color = final_color + diff_component + spec_component;
  }
}

bool Scene::render(const char *filename, int width, int height)
{
  CImg<unsigned char> image(width, height, 1, 3);

  for(auto i = 0; i < width; i++)
  {
    for(auto j = 0; j < height; j++)
    {
      auto shooting_dir = camera.pixelToWorldSpace(i,j);
      Ray r(camera.origin, shooting_dir);
      Color final_color(0,0,0);
      Vector3D p;
 
      for(auto &obj : scene_objs)
      {
        Vector3D p;
        
        if(obj->intersects(r, p))
        {
          render_object(p, camera.origin, obj, final_color, scene_lights);
          final_color = Color::clamp(final_color);
        }
      }
      final_color *= (float)255.0;
      image(i,j, 0) = final_color.r;
      image(i,j, 1) = final_color.g;
      image(i,j, 2) = final_color.b;
    }
  }

  image.save(filename);
  return true;
}
