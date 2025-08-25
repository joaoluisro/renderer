#ifndef SCENE_HEADER
#define SCENE_HEADER

#include <math.h>
#include <vector>
#include <memory>

#include "core/camera.h"
#include "core/color.h"
#include "core/light.h"

#include "geometry/baseObject.h"

#include "bvh/mesh.h"

#include "io/framebuffer.h"

#define PI 3.1419

using namespace std;

class Scene{
  public:
    Scene(Camera &c, 
      vector<shared_ptr<Mesh>> &scene_meshes, 
      vector<shared_ptr<Light>> &lights) :
      camera(c), scene_meshes(scene_meshes), scene_lights(lights)
    {
    }
    ~Scene();

    void render(const char *filename, int width, int height);
    inline Color traceRay(Ray &r, int depth) const;
    inline double hit(shared_ptr<BaseObject> &closest, Ray &r) const;
    inline bool isShadowed( Vector3D &l,  Vector3D &p,  shared_ptr<BaseObject> &obj) const;
    inline Color computePhong( Vector3D &p,  shared_ptr<BaseObject> obj) const;
  

    inline void traceRayHeatmap(Ray &r, vector<float> &radiance, vector<float> &hits) const;
    inline float computePhongRadiance(Vector3D &p,  shared_ptr<BaseObject> obj) const;
    void render_heatmap(const char *filename, int width, int height, int &box_tests, int &leaf_tests, int &n_test_max);
  
  private:
    Camera camera;
    vector<shared_ptr<Mesh>> scene_meshes;
    vector<shared_ptr<Light>>  scene_lights;
};

#endif