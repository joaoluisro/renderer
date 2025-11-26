#ifndef SCENE_HEADER
#define SCENE_HEADER

#include <math.h>
#include <vector>
#include <memory>
#include <random>

#include "core/camera.h"
#include "core/color.h"
#include "core/light.h"

#include "math/face.h"

#include "bvh/mesh.h"

#include "io/framebuffer.h"
#include "math/randomnumbergenerator.h"
#include "core/sampler.h"

using namespace std;

struct Intersection{
    bool missed;
    shared_ptr<Face> face;
    Vector3D point;
    float t;
    Material material;
};

class Scene{
  public:
    Scene(Camera &c, vector<shared_ptr<Mesh>> &meshes,vector<shared_ptr<Light>> &lights);
    ~Scene();

    void render(const char *filename, const int width,const int height, const int n_sample);

    inline Intersection testIntersection(const Ray& r) const;

    Color Li(const Ray &w0, int n_samples, int depth);


    // inline Color traceRay(const Ray &r,const int n_samples, int depth);
    // Color integrateNEE(Vector3D p, Vector3D n, Material hit_material, shared_ptr<Face> hit, int n_samples);
    // Color integrateIndirect(Vector3D p, Vector3D n, Ray r, Material hit_material, shared_ptr<Face> hit, int n_samples, int depth);
    inline Color shadeTransparent(const Vector3D& dir, const shared_ptr<Face> face, const Vector3D& p) const;
    inline Color getFresnel(float &trn, float &ref) const;
  private:
    Camera camera;
    vector<shared_ptr<Mesh>> meshes;
    vector<shared_ptr<Light>> lights;
};

#endif
