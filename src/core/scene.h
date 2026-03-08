#ifndef SCENE_HEADER
#define SCENE_HEADER


#include "core/camera.h"
#include "geometry/mesh.h"
#include "core/light.h"
#include "math/rng.h"

struct Intersection{
    bool missed;
    shared_ptr<Face> face;
    vec3 point;
    float t;
    Material material;
};

struct Scene{
    Camera camera;
    vector<shared_ptr<Mesh>> meshes;
    vector<shared_ptr<Light>> lights;

    Scene(Camera &c,
          vector<shared_ptr<Mesh>> &m,
          vector<shared_ptr<Light>> &l) : camera(c), meshes(m), lights(l)
    {
    }

    inline Intersection intersectionTest(const Ray &r) const
    {
        Material m;
        float min_dist = std::numeric_limits<float>::infinity();
        bool missed = true;
        shared_ptr<Face> min_found = nullptr;
        for(const auto &mesh : meshes)
        {
            shared_ptr<Face> tmp;
            float t_found = mesh->hit(tmp, r, min_dist);
            if(tmp && t_found < min_dist)
            {
                missed = false;
                min_dist = t_found;
                min_found = tmp;
                m = mesh->material;
            }
        }
        return Intersection{.missed=missed,
                            .face = min_found,
                            .point = r.at(min_dist),
                            .t = min_dist,
                            .material = m};
    }

    shared_ptr<Light> sampleLight() const
    {
        static pcg32 rng(pcg_extras::seed_seq_from<std::random_device>{});
        static std::uniform_int_distribution<int> dist(0, lights.size() - 1);

        return lights.at(dist(rng));
    }
};

#endif
