#ifndef SCENE_HEADER
#define SCENE_HEADER

#include <math.h>
#include <vector>
#include <memory>
#include <random>

#include "core/camera.h"
#include "core/color.h"
#include "core/light.h"

#include "geometry/face.h"

#include "bvh/mesh.h"

#include "io/framebuffer.h"


using namespace std;

class Scene{
  public:
    Scene(Camera &c, vector<shared_ptr<Mesh>> &meshes,vector<shared_ptr<Light>> &lights);
    ~Scene();

    void render(const char *filename, int width, int height, int n_sample);
    inline Color traceRay(const Ray &r, int depth) const;
    inline Color integrate(const Ray &r, int n_samples, int depth) const;
    Color traceSampledRay(const Ray &r, const Vector3D &p, const Vector3D &n, const shared_ptr<Face> f, int &hit_count) const;
    Color integrateNEE(const Ray &r, int n_samples, int depth);

    inline float intersects(shared_ptr<Face> &closest, Material &m, const Ray& r) const;
    inline Color shadeTransparent(const Vector3D& dir, const shared_ptr<Face> face, const Vector3D& p) const;
    inline Color getFresnel(float &trn, float &ref) const;
  private:
    Camera camera;
    vector<shared_ptr<Mesh>> meshes;
    vector<shared_ptr<Light>> lights;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
};

#endif
