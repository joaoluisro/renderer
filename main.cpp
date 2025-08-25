#include <memory>
#include <vector>
#include <filesystem>
#include <iostream>
#include <chrono>

#include "core/camera.h"
#include "core/scene.h"
#include "core/color.h"

#include "geometry/ray.h"
#include "geometry/sphere.h"
#include "geometry/triangle.h"

#include "bvh/mesh.h"

#include "io/wavefront_loader.h"

using namespace std;

// bistro scene:
/*
  Vector3D c_origin(-1.01884,1.35107,2.69633), c_viewpoint(8.42609,  1.84162,  2.35351);

  lights.push_back(make_shared<Light>(Vector3D(3.3904,0.978005,3.5282), Color(1,1,1)));
  lights.push_back(make_shared<Light>(Vector3D(5.00171,0.511438,3.08568), Color(1,1,1)));
  lights.push_back(make_shared<Light>(Vector3D(7.97968,3.65849,3.99706), Color(1,1,1)));
  lights.push_back(make_shared<Light>(Vector3D(7.7172,4.60972,2.40192), Color(1,1,1)));
  lights.push_back(make_shared<Light>(Vector3D(11.0316,1.99278,4.01327), Color(1,1,1)));
  lights.push_back(make_shared<Light>(Vector3D(7.96845,-1.02716,3.07679), Color(1,1,1)));
*/

// hairball:
/*
Vector3D c_origin(-11.8798,18.7134,9.98171), c_viewpoint(0,0,0);

lights.push_back(make_shared<Light>(Vector3D(-3,18,13), Color(1,1,1)));
lights.push_back(make_shared<Light>(Vector3D(-3,18,14), Color(1,1,1)));
lights.push_back(make_shared<Light>(Vector3D(-3,18,15), Color(1,1,1)));

lights.push_back(make_shared<Light>(Vector3D(-2,18,13), Color(1,1,1)));
lights.push_back(make_shared<Light>(Vector3D(-2,18,14), Color(1,1,1)));
lights.push_back(make_shared<Light>(Vector3D(-2,18,15), Color(1,1,1)));


lights.push_back(make_shared<Light>(Vector3D(-1,18,13), Color(1,1,1)));
lights.push_back(make_shared<Light>(Vector3D(-1,18,14), Color(1,1,1)));
lights.push_back(make_shared<Light>(Vector3D(-1,18,15), Color(1,1,1)));
*/


/* rotate

  c_origin = Vector3D(sqrt(25)*cos(rotation), 2, sqrt(25)*sin(rotation));
  Vector3D c_origin(24.309576757*cos(rotation_rad),24.309576757*sin(rotation_rad),9.98171), c_viewpoint(0,0,0);

  lights.push_back(make_shared<Light>(Vector3D(sqrt(29)*cos(rotation), 3, sqrt(29)*sin(rotation)), Color(1,1,1)));
*/


/* lucy, dragon, bunny scene

  Vector3D c_origin(-11.8798,18.7134,9.98171), c_viewpoint(0,0,0);
  lights.push_back(make_shared<Light>(Vector3D(-3,18,13).to_blender(), Color(1,1,1)));


*/

/* LUcy small

  Vector3D c_origin(6.70319,0.863768,4.25675), c_viewpoint(0,0,0);
  lights.push_back(make_shared<Light>(Vector3D(6.70319,0.863768,5.25675).to_blender(), Color(1,1,1)));

*/

/* Dragon stanford 
  Vector3D c_origin(-1.73571,1.00179,0.781727), c_viewpoint(0,0,0);
*/



/* INPUT

  - out image width
  - out image height
  - BVH Threshold
  - bvh_type

*/

/* OUTPUT

1) csv row containing:

- n_faces (triangles)
- build_time_mean (ms)
- n_tests_box
- n_tests_leaf
- n_test_box_max

2) Heatmap Image .ppm

*/

void render_sphere(int width, int height)
{
  vector<shared_ptr<Mesh>> scene_meshes;
  vector<shared_ptr<BaseObject>> faces;

  Vector3D sphere_origin(0,0,0);
  Vector3D sphere_origin2(2,0,0);
  Vector3D sphere_origin3(-2,0,0);

  Color sphere_color(1,0,0);
  Color sphere_color2(1,1,0);
  Color sphere_color3(0,0,1);

  faces.push_back(make_shared<Sphere>(sphere_origin,1.0,sphere_color));
  faces.push_back(make_shared<Sphere>(sphere_origin2,1.0,sphere_color2));
  faces.push_back(make_shared<Sphere>(sphere_origin3,1.0,sphere_color3));

  scene_meshes.push_back(make_shared<Mesh>(faces,false,false,2,BVHType::MEDIAN));

  Vector3D c_origin(10,10,0), c_viewpoint(0,0,0);
  const double fov = 30.0;
  Camera camera(c_origin, c_viewpoint, fov, width, height);

  vector<shared_ptr<Light>> lights;
  lights.push_back(make_shared<Light>(Vector3D(5,5,5).to_blender(), Color(1,1,1)));

  Scene main_scene(camera, scene_meshes, lights);
  
  main_scene.render("sphere.ppm", width, height);
}

void benchmark(const char* filename, int width, int height, int threshold, string bvh_type)
{  
  BVHType type = BVHType::MIDPOINT;
  if(bvh_type == "median") type = BVHType::MEDIAN;
  if(bvh_type == "sah")    type = BVHType::SAH;

  vector<shared_ptr<Mesh>> scene_meshes;
  vector<double> times;
  int n_test_box_max;
  int parsed_faces;
  typedef std::chrono::milliseconds ms;
  ms elapsed_ms;

  auto start = std::chrono::high_resolution_clock::now();

  parsed_faces = parse_obj(filename, scene_meshes, threshold, type);

  auto finish = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed = finish - start;
  
  elapsed_ms = std::chrono::duration_cast<ms>(elapsed);

  double build_time_mean = elapsed_ms.count();

  vector<shared_ptr<Light>> lights;

  Vector3D c_origin(0,-13,5), c_viewpoint(0,0,0);

  // lights.push_back(make_shared<Light>((c_origin + Vector3D(0,1,0)).to_blender(), Color(1,1,1)));
  lights.push_back(make_shared<Light>(Vector3D(3.5,-4.5,6).to_blender(), Color(1,1,1)));
  // lights.push_back(make_shared<Light>(Vector3D(3.5,-4.5,7).to_blender(), Color(1,1,1)));
  // lights.push_back(make_shared<Light>(Vector3D(3.5,-3.5,6).to_blender(), Color(1,1,1)));
  // lights.push_back(make_shared<Light>(Vector3D(3.5,-3.5,7).to_blender(), Color(1,1,1)));

  const double fov = 45.0;

  Camera camera(c_origin, c_viewpoint, fov, width, height);

  vector<shared_ptr<BaseObject>> faces;
  Scene main_scene(camera, scene_meshes, lights);
  
  int box_tests, leaf_tests;
  // main_scene.render_heatmap("output.ppm", width, height, box_tests, leaf_tests, n_test_box_max);
  main_scene.render("output.ppm", width, height);

#ifdef DEBUG_BUILD
  cout << bvh_type << ","
       << threshold << ","
       << parsed_faces << ","
       << build_time_mean << ","
       << box_tests << ","
       << leaf_tests << ","
       << n_test_box_max << "\n";
#endif
}

int main(int argc, char *argv[])
{
  if(argc < 6)
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
  auto leaf_threshold = atoi(argv[4]);
  std::string bvh_type = argv[5];
  // benchmark(filename, width, height,leaf_threshold,bvh_type);
  render_sphere(width,height);
}