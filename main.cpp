#include <memory>
#include <vector>
#include <filesystem>
#include <iostream>

#include "camera.h"
#include "ray.h"
#include "scene.h"
#include "sphere.h"
#include "color.h"
#include "triangle.h"
#include "wavefront_loader.h"

using namespace std;


bool load_sphere(vector<shared_ptr<BaseObject>> &objs)
{
  Vector3D sphere_origin(0.0, 0, 0);
  Color sphere_color(1.0f, 0.0f, 0.0f);
  float sphere_radius = 1.5f;
  objs.push_back(make_shared<Sphere>(sphere_origin, sphere_radius, sphere_color));
}

int main(int argc, char *argv[])
{
  if(argc < 4)
  {
    std::cerr << "Missing args." << std::endl;
    return 1;
  }

  const char *filename = argv[1];
  if(!std::filesystem::exists(filename))
  {
    std::cerr << ".obj file does not exist." << std::endl;
  }

  int width = atoi(argv[2]);
  int height = atoi(argv[3]);

  /* Scene objects */
  vector<shared_ptr<BaseObject>> objs;
  if(parse_obj(filename, objs))
  {
    std::cout << "wavefront file parsed with " << objs.size() << " vertices\n"; 
  }
  else
  {
    std::cerr << "Error parsing wavefront file " << filename << "\n";
    return 1;
  }

  Vector3D c_origin(5,1,1), c_viewpoint(0.0f,  0.0f,  0.0f);

  float fov = 45.0f;
  Camera camera(c_origin, c_viewpoint, fov, width, height);

  /* Light origins */
  vector<shared_ptr<Light>> lights;
  lights.push_back(make_shared<Light>(Vector3D(5,6,7), Color(1,1,1)));

  Scene main_scene(camera, objs, lights);
  main_scene.render("output.ppm", width, height);
}