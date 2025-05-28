#ifndef SCENE_HEADER
#define SCENE_HEADER

#include <math.h>
#include <vector>
#include <memory>
#include "camera.h"
#include "baseObject.h"
#include "color.h"
#include "framebuffer.h"
#include "light.h"

#define PI 3.1419

using namespace std;

class Scene{
  public:
    Scene(Camera &c, 
      vector<shared_ptr<BaseObject>> &objs, 
      vector<shared_ptr<Light>> &lights) :
      camera(c), scene_objs(objs), scene_lights(lights)
    {
    }
    ~Scene();

    bool render(const char *filename, int width, int height);
  
  private:
    Camera camera;
    vector<shared_ptr<BaseObject>> scene_objs;
    vector<shared_ptr<Light>>  scene_lights;
};

#endif