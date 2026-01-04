#ifndef PATH_TRACER_HEADER
#define PATH_TRACER_HEADER

#include "core/definitions.h"

#include "core/scene.h"
#include "io/framebuffer.h"

#include "math/rng.h"
#include "core/sampler.h"
#include "materials/bsdf.h"

class PathTracer
{
public:
    PathTracer(const shared_ptr<Scene> scene, const Sampler &sampler) : scene(scene), sampler(sampler){}
    ~PathTracer();

    void render(const int spp, FrameBuffer &fb);
    Spectrum traceRay(const Ray &r, bool is_delta, int depth);
    Spectrum LDirect(const vec3 &w0, const Intersection &hit, const vec3 &n_hit);
    Spectrum LIndirect(const vec3 &w0, const Intersection &hit, const vec3 &n_hit, int depth);
    Spectrum LMirror(const vec3 &w0,const Intersection &hit,const vec3 &n_hit,int depth);
public:
    shared_ptr<Scene> scene;
    Sampler sampler;
};

#endif
