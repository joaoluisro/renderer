#include <filesystem>
#include <iostream>

#include "core/scene.h"
#include "io/parsing.h"

using namespace std;


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
  }

  int width = atoi(argv[2]);
  int height = atoi(argv[3]);
  int n_samples = atoi(argv[4]);
  float fov = 60.0f;
  Scene *s = Parse::scene_file(filename, width, height, fov);
  s->render("output.ppm",width,height,n_samples);
}

