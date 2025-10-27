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
#include "io/parsing.h"

using namespace std;


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

  // Arbitrary position, aimed at stanford bunny
  Vector3D c_origin(0,-8,5), c_viewpoint(0,0,0);

  // Light on top of camera
  Vector3D light_pos;
  light_pos = c_origin + Vector3D(0,3,0);
  lights.push_back(make_shared<Light>(light_pos.to_blender(), Color(1,1,1)));
 
  const double fov = 60.0;
  Camera camera(c_origin, c_viewpoint, fov, width, height);
  vector<shared_ptr<BaseObject>> faces;
  Scene main_scene(camera, scene_meshes, lights);
  int box_tests, leaf_tests;

//  main_scene.render_heatmap("output-heatmap.ppm", width, height, box_tests, leaf_tests, n_test_box_max);
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
    std::cerr << ".scene file does not exist." << std::endl;
  }

  int width = atoi(argv[2]);
  int height = atoi(argv[3]);
  auto leaf_threshold = atoi(argv[4]);
  std::string bvh_type = argv[5];
  float fov = 60.0f;
  Scene *s = Parse::scene_file(filename, width, height, fov);
  s->render("output.ppm",width,height);
}

