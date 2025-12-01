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
    vec3 point;
    float t;
    Material material;
};


class Scene{
  public:
    Scene(Camera &c, vector<shared_ptr<Mesh>> &meshes,vector<shared_ptr<Light>> &lights);
    ~Scene();

    void render(const char *filename, const int width,const int height, const int n_sample);

    inline Intersection testIntersection(const Ray& r) const;

    Radiance integrateNEE(const vec3 &w0, const Intersection &hit, const vec3 &n_hit, const int n_samplesr) const;
    Radiance integrateIndirect(const vec3 &w0, const Intersection &hit, const vec3 &n_hit, const int n_samples, int depth);
    Radiance L_transparent(const vec3 &w0, const Intersection &hit, const vec3 &n_hit, const int n_samples, int depth);

    Radiance L_mirror(const Ray &w0, const Intersection &hit, const vec3 &n_hit, int n_samples, int depth);

    Radiance Li(const Ray &w0, int n_samples,bool is_delta, int depth);

    inline Radiance shadeTransparent(const vec3& dir, const shared_ptr<Face> face, const vec3& p) const;
    inline Radiance getFresnel(float &trn, float &ref) const;
  private:
    Camera camera;
    vector<shared_ptr<Mesh>> meshes;
    vector<shared_ptr<Light>> lights;
};

#endif
