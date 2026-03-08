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

    void renderSingle(const int spp, FrameBuffer &fb);
    void renderMultiple(const int spp, FrameBuffer &fb, const char* dir_path);

    Spectrum L_transparent(const vec3 &w0,
                           const Intersection &hit,
                           const vec3 &n_hit,
                            bool inside,
                           int depth);
    Spectrum traceRay(const Ray &r, bool is_delta, bool is_inside, int depth);
    Spectrum LDirect(const vec3 &w0, const Intersection &hit, const vec3 &n_hit);
    Spectrum LIndirect(const vec3 &w0, const Intersection &hit, const vec3 &n_hit, int depth);
    Spectrum LMirror(const vec3 &w0,const Intersection &hit,const vec3 &n_hit,int depth);
public:
    shared_ptr<Scene> scene;
    Sampler sampler;
};

#endif
