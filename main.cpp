#include <memory>
#include <vector>

#include "camera.h"
#include "ray.h"
#include "scene.h"
#include "sphere.h"
#include "color.h"
#include "triangle.h"
#include "wavefront_loader.h"

using namespace std;

// meeting 19/05
// Triangles intersection test
// handle obj files and meshes
// box
// implementing mirror, checker board under it -> use procedural texture
// tone mapping - reinhard mapping
// -|> gamma correction, would like to see compression on brighter areas, more contrast for dark colors

int main(int argc, char *argv[])
{
  const char *filename = argv[1];

  int width = atoi(argv[2]);
  int height = atoi(argv[3]);

  /* Scene objects */
  Vector3D sphere_origin(0.0, 1.0, 0.5);
  Color sphere_color(1.0f, 0.0f, 0.0f);
  float sphere_radius = 3.5f;

  vector<shared_ptr<BaseObject>> objs;

  // obj->translate(Vector3D(0,0,5));
  // objs.push_back(make_shared<Triangle>(
  // Vector3D(0.0f, 0.0f, 0.0f),
  // Vector3D(1.0f, 0.0f, 0.0f),
  // Vector3D(0.0f, 1.0f, 0.0f),
  // Vector3D(0.0f, 0.0f, 1.0f),
  // Color(255,0,0)));
  // 1) Triangle in the XY plane, normal pointing +Z

  // 2) Triangle in the XZ plane, normal pointing +Y
  // objs.push_back(make_shared<Triangle>(
  //     Vector3D(0.0f, 0.0f, 0.0f),
  //     Vector3D(1.0f, 0.0f, 0.0f),
  //     Vector3D(0.0f, 0.0f, 1.0f),
  //     Vector3D(0.0f, 1.0f, 0.0f),
  //     Color(0,255,0)
  // ));
  // // 3) Triangle in the YZ plane, normal pointing +X
  // objs.push_back(make_shared<Triangle>(
  //     Vector3D(0.0f, 0.0f, 0.0f),
  //     Vector3D(0.0f, 1.0f, 0.0f),
  //     Vector3D(0.0f, 0.0f, 1.0f),
  //     Vector3D(1.0f, 0.0f, 0.0f),
  //     Color(0,0,255)
  // ));
  parse_obj(argv[1], objs);
  Vector3D centroid{0, 0, 0};
  int i = 0;
  for (auto &obj : objs)
  {
    centroid = centroid + obj->centroid();
    i++;
  }
  centroid = centroid * (-1.0f / i);
  for(auto &obj : objs)
  {
    obj->translate(centroid);
  }
  // cout << objs.size() << "\n";
  // objs.push_back(make_shared<Sphere>(sphere_origin, sphere_radius, sphere_color));
  // cout << objs[0].get_color() << "\n";

  /* Camera setup */
  Vector3D c_origin(-2, -5, 3), c_viewpoint(0.0f, 0.0f, 0.0f);
  // Vector3D c_origin(0.35f, -1, 0.0), c_viewpoint(0.0f,0.0f,0.0f);
  // Vector3D c_origin(-0.2f, 0.1f, 1.0f), c_viewpoint(0.0f,0.0f,0.0f);

  float fov = 45.0f;
  Camera camera(c_origin, c_viewpoint, fov, width, height);

  /* Light origins */
  vector<shared_ptr<Vector3D>> lights;
  // lights.push_back(make_shared<Vector3D>(c_origin.x, c_origin.y - 1.0f, c_origin.z -1.0f));
  auto l = centroid;
  l = l + Vector3D(0,10,0);
  lights.push_back(make_shared<Vector3D>(l.x,l.y,l.z));

  Scene main_scene(camera, objs, lights);
  main_scene.render("output.ppm", width, height);

  // float t = 0.0f;
  // float dt = 0.1f;
  // int j = 0;
  // while(t < 12.0f)
  // {
  //     lights.push_back(make_shared<Vector3D>(2.0f * cos(t),2.0f   * sin(t),2.0f));
  //     Scene main_scene(camera, objs, lights);
  //     std::string s = filename + std::to_string(j) + ".ppm";
  //     auto f = s.c_str();
  //     main_scene.render(f, width, height);
  //     lights.pop_back();
  //     t += dt;
  //     j++;
  // }
}