#include <filesystem>
#include <iostream>
#include <memory>

#include "core/scene.h"
#include "io/scene_loader.h"
#include "core/sampler.h"

#include "integrators/path_tracer.h"

#include <omp.h>
#include <iostream>


int main(int argc, char *argv[])
{
    if(argc < 5)
    {
        std::cerr << "Missing args." << std::endl;
        return 1;
    }

    const char *filename = argv[1];
    if(!std::filesystem::exists(filename))
    {
        std::cerr << ".scene file does not exist." << std::endl;
        return 1;
    }

    int width = atoi(argv[2]);
    int height = atoi(argv[3]);
    int n_samples = atoi(argv[4]);
    float fov = 60.0f;
    std::shared_ptr<Scene> scene = parseSceneFile(filename, width, height, fov);

    if(!scene)
    {
        std::cerr << "Error parsing .scene file." << std::endl;
        return 1;
    }
    Sampler sampler;
    PathTracer p(scene, sampler);
    FrameBuffer fb(width, height);
    p.render(n_samples, fb);
    fb.clamp();
    fb.writeToPPM("output.ppm", width, height);
    return 0;
}

