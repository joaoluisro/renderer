#ifndef SCENE_HEADER
#define SCENE_HEADER

#include <math.h>
#include <vector>
#include <memory>

#include "core/camera.h"
#include "core/color.h"
#include "core/light.h"

#include "geometry/face.h"

#include "bvh/mesh.h"

#include "io/framebuffer.h"


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

    void render(const char *filename, int width, int height, int n_sample);
    inline Color traceRay(const Ray &r, int depth) const;
    inline Color integrate(const Ray &r, int n_samples) const;
    Color traceSampledRay(const Ray &r, const Vector3D &p, const Vector3D &n, const shared_ptr<Face> f, int &hit_count) const;

    inline float intersects(shared_ptr<Face> &closest,const Ray& r) const;
    inline bool isOccluded(const Vector3D &l, const Vector3D &p, shared_ptr<Face> obj) const;
    inline Color shadeOpaque(const Vector3D& dir, const shared_ptr<Face> face, const Vector3D& p) const;
    inline Color shadeTransparent(const Vector3D& dir, const shared_ptr<Face> face, const Vector3D& p) const;
    inline Color getFresnel(float &trn, float &ref) const;
  private:
    Camera camera;
    vector<shared_ptr<Mesh>> scene_meshes;
    vector<shared_ptr<Light>>  scene_lights;
};

#endif
