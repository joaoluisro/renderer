#include "core/scene.h"

#define cimg_display 0  // (no X11 needed)
#include "CImg-3.5.4/CImg.h"
using namespace cimg_library;

#define MAX_BOUNCE 3
#define MAX_VAL 1e+9
#define BACKGROUND_COLOR Color(0.07,0.07,0.07)
#define EPSILON 1e-3
#define REFLECT false

static long int max_hits = 0;

Scene::~Scene()
{
}

void writeToPPM(const char* filename, Framebuffer &frame_buffer, int width, int height)
{
  frame_buffer.clamp();
  CImg<unsigned char> image(width, height, 1, 3,0.5);
  for(auto i = 0; i < height; i++)
  {
    for(auto j = 0; j < width; j++)
    {
      image(i,j,0) = frame_buffer.at(i,j).r;
      image(i,j,1) = frame_buffer.at(i,j).g;
      image(i,j,2) = frame_buffer.at(i,j).b;
    }
  }
  image.save(filename);
}

inline double Scene::hit(shared_ptr<BaseObject> &closest, Ray &r) const
{
  double min_dist = 1e+9;
  shared_ptr<BaseObject> closest_found = nullptr;
  
  for(auto &mesh : scene_meshes)
  {
    shared_ptr<BaseObject> closest_buffer = nullptr;
    auto t_found = mesh->hit(closest_buffer, r);
    if(closest_buffer && t_found < min_dist)
    {
      min_dist = t_found;
      closest_found = closest_buffer;
    }
  }
  if(closest_found)
  {
    closest = closest_found;
    return min_dist;
  }
  return -1;
}

inline bool Scene::isShadowed(Vector3D &light_origin, 
  Vector3D &p,
  shared_ptr<BaseObject> &face) const
{
  auto light_ray_direction = (p - light_origin).normalized();

  Ray shadow_ray(light_origin,light_ray_direction);

  shared_ptr<BaseObject> other_face;

  auto t = hit(other_face, shadow_ray);

  if(t > 0 && other_face != face)
  {
    return true;
  } 
  return false;
}

inline Color Scene::computePhong(Vector3D &p,
                                shared_ptr<BaseObject> obj) const
{

  Vector3D V = (camera.origin - p).normalized();
  const double shininess = 16;
  /* ambient term */
  auto final_color = BACKGROUND_COLOR;
  auto obj_color = obj->get_color();

  final_color = obj_color * 0.05;

  for(const auto &light : scene_lights)
  {
    Vector3D L = (light->position - p).normalized();

    Vector3D N = obj->get_normal(p).normalized();
    double ndotl = N.dot(L);
    // if(ndotl < 0.0) ndotl = -ndotl;
    if(ndotl < 0.0) continue;

    // if(isShadowed(L, p, obj)) continue;
    if(isShadowed(light->position, p, obj)) continue;

    // Diffuse component
    Color diff = (obj_color* ndotl) * light->color ;

    // Specular component
    auto reflection_vector = ((2 * ndotl) * N - L).normalized();
    double specAngle = max(reflection_vector.dot(V), 0.0);
    Color spec = (obj_color * pow(specAngle, shininess)) * light->color;

    final_color += diff + spec;
  }
  // cout << final_color.r << "\n";
  return final_color;
}

inline Color Scene::traceRay(Ray &r, int depth) const
{

  shared_ptr<BaseObject> closest = nullptr;

  auto t = hit(closest, r);
  if(t < 0 || !closest)
  {
    return BACKGROUND_COLOR;
  }

  auto intersection_point = r.at(t);
  if (r.box_tests > max_hits){
    max_hits = r.box_tests;
  }
  auto local_color = computePhong(intersection_point, closest);
  if(depth <= 0)
  {
    return local_color;
  }

  Material mat = closest->material();
  
  if (REFLECT) 
  {
    // auto reflection_origin    = intersection_point;
    // Vector3D D = r.direction;
    // Vector3D N = closest->get_normal(intersection_point);
    // auto R = D - 2*(D.dot(N) * N);
    // auto reflection_direction = R;

    // Ray reflection_ray(reflection_origin, reflection_direction);
    // auto reflected_color = traceRay(reflection_ray, depth - 1);
    // local_color += reflected_color * 0.1;
  }

  return local_color;
}


void Scene::render(const char *filename, int width, int height)
{
  Framebuffer frame_buffer(width, height);
  long unsigned int total_leaf = 0;
  long unsigned int total_box = 0;

  #pragma omp parallel for collapse(2)
  for(auto i = 0; i < height; i++)
  {
    for(auto j = 0; j < width; j++)
    {
      auto shooting_dir = camera.pixelToWorldSpace(i,j);
      
      Ray r(camera.origin, shooting_dir);

      auto final_color = traceRay(r, MAX_BOUNCE);
      total_box += r.box_tests;
      total_leaf += r.leaf_tests;
      frame_buffer.set(i,j,final_color);
    }
  }
  // cout << total_box;

  writeToPPM(filename, frame_buffer, width, height);
}


inline float Scene::computePhongRadiance(Vector3D &p,
  shared_ptr<BaseObject> obj) const
{

  Vector3D V = (camera.origin - p).normalized();
  const double shininess = 16;
  
  float obj_color = 1;
  float final_color = 0;

  for(const auto &light : scene_lights)
  {
    Vector3D L = (light->position - p).normalized();

    Vector3D N = obj->get_normal(p).normalized();
    double ndotl = N.dot(L);
    if(ndotl < 0.0) ndotl = -ndotl;
    // if(ndotl < 0.0) continue;

    // if(isShadowed(L, p, obj)) continue;
    // if(isShadowed(light->position, p, obj)) continue;

    // Diffuse component
    float diff = (obj_color* ndotl);

    // Specular component
    auto reflection_vector = ((2 * ndotl) * N - L).normalized();
    double specAngle = max(reflection_vector.dot(V), 0.0);
    float spec = (obj_color * pow(specAngle, shininess));

    final_color += diff + spec;

    }

  return final_color/2;
}



Color jet(float n)
{
  // auto t = std::clamp(n, 0.0f, 1.0f);
  auto t = n;
  // each channel is a piecewise “tent” function:
  double r = 1.5f - std::fabs(4.0f*t - 3.0f);
  double g = 1.5f - std::fabs(4.0f*t - 2.0f);
  double b = 1.5f - std::fabs(4.0f*t - 1.0f);
  // convert to 8‑bit
  return Color(r,g,b);
}


// /// Convert HSV (h in [0,360), s,v in [0,1]) to RGB in [0,1]
Color hsv2rgb(float h, float s, float v) {
  float c = v * s;
  float h_prime = std::fmod(h / 60.0f, 6.0f);
  float x = c * (1.0f - std::fabs(std::fmod(h_prime, 2.0f) - 1.0f));
  float m = v - c;

  float rp, gp, bp;
  if      (h_prime < 1) { rp = c; gp = x; bp = 0; }
  else if (h_prime < 2) { rp = x; gp = c; bp = 0; }
  else if (h_prime < 3) { rp = 0; gp = c; bp = x; }
  else if (h_prime < 4) { rp = 0; gp = x; bp = c; }
  else if (h_prime < 5) { rp = x; gp = 0; bp = c; }
  else                  { rp = c; gp = 0; bp = x; }

  return Color(rp + m, gp + m, bp + m);
}

/// Rainbow colormap: t=0→blue, t=0.5→green, t=1→red
Color rainbow(float t) 
{
  // map t to hue angle: 240° (blue) → 0° (red)
  float hue = (1.0f - t) * 240.0f;
  // cout << "hue " << hue << "\n";
  return hsv2rgb(hue, 1.0f, 1.0f);
}

Color plasma(float t) {
  // 1) clamp to [0,1]
  // 2) control points at t = 0, 0.2, 0.4, 0.6, 0.8, 1.0
  static  size_t N = 6;
  static  Color C[6] = {
      Color(0.050383f, 0.029803f, 0.527975f),  // deep purple
      Color(0.309868f, 0.154282f, 0.701817f),  // dark blue–purple
      Color(0.597594f, 0.219739f, 0.604556f),  // blue–pink
      Color(0.815384f, 0.282139f, 0.368623f),  // orange–pink
      Color(0.959478f, 0.536779f, 0.181176f),  // orange–yellow
      Color(0.988362f, 0.998364f, 0.644924f)   // bright yellow
  };

  // 3) locate segment
  float ft = t * (N - 1);                // scale to [0, N-1]
  size_t i = std::min((size_t)std::floor(ft), N - 2);
  float  u = ft - float(i);              // local [0,1] within segment

  // 4) linearly interpolate between C[i] and C[i+1]
  return Color(
      (1 - u) * C[i].r + u * C[i+1].r,
      (1 - u) * C[i].g + u * C[i+1].g,
      (1 - u) * C[i].b + u * C[i+1].b
  );
}


/// Viridis colormap (5‑point linear interpolation)
Color viridis(float t) {
  // 1) clamp to [0,1]
  t = std::clamp(t, 0.0f, 1.0f);

  // 2) control points at t = 0,0.25,0.5,0.75,1.0
  static  size_t N = 5;
  static  Color C[5] = {
      Color(0.267004f, 0.004874f, 0.329415f),  // deep blue
      Color(0.190631f, 0.407061f, 0.556089f),  // blue‑cyan
      Color(0.208030f, 0.718701f, 0.472873f),  // green‑cyan
      Color(0.622255f, 0.783682f, 0.266941f),  // yellow‑green
      Color(0.993248f, 0.906157f, 0.143936f)   // yellow
  };

  // 3) find which segment we’re in
  float ft = t * (N - 1);              // scale to [0, N-1]
  size_t i = std::min((size_t)std::floor(ft), N - 2);
  float  u = ft - float(i);            // local [0,1] within segment

  // 4) lerp between C[i] and C[i+1]
  return Color(
      (1 - u) * C[i].r     + u * C[i+1].r,
      (1 - u) * C[i].g     + u * C[i+1].g,
      (1 - u) * C[i].b     + u * C[i+1].b
  );
}


inline void Scene::traceRayHeatmap(Ray &r, vector<float> &radiance, vector<float> &hits) const
{
  shared_ptr<BaseObject> closest = nullptr;

  auto t = hit(closest, r);
  r.box_tests /= 2;

  if(t < 0 || !closest)
  {
    hits.push_back(r.box_tests);
    radiance.push_back(0);
    return;
  }

  auto intersection_point = r.at(t);

  hits.push_back(r.box_tests);
  radiance.push_back(computePhongRadiance(intersection_point, closest));
  if (r.box_tests > max_hits)
  {
    max_hits = r.box_tests;
  }
}

void Scene::render_heatmap(const char *filename, int width, int height, int &box_tests, int &leaf_tests, int &n_test_max)
{
  vector<float> radiance;
  vector<float> hits;

  Framebuffer frame_buffer(width, height);
  long unsigned int total_leaf = 0;
  long unsigned int total_box = 0;

  // #pragma omp parallel for collapse(2)
  for(auto i = 0; i < height; i++)
  {
    for(auto j = 0; j < width; j++)
    {
      auto shooting_dir = camera.pixelToWorldSpace(i,j);
      
      Ray r(camera.origin, shooting_dir);

      traceRayHeatmap(r, radiance, hits);
      total_box += r.box_tests;
      total_leaf += r.leaf_tests;
    }
  }
  box_tests = total_box;
  leaf_tests = total_leaf;
  int k =0;
  for(auto i = 0; i < height; i++)
  {
    for(auto j = 0; j < width; j++)
    {
      float gamma = 1.1f;                    // try 1.5 → 3.0
      float color_t = hits[k]/max_hits;
      float t_corr = std::pow(color_t, gamma);
      // t_corr = std::clamp(t_corr, 0.0f, 1.0f);
      Color final_color = (rainbow(t_corr));
      if(radiance[k] == 0) final_color = BACKGROUND_COLOR;
      // final_color = final_color * radiance[k];
      frame_buffer.set(i,j,final_color);
      k++;
      
    }
  }
  n_test_max = max_hits;
  writeToPPM(filename, frame_buffer, width, height);
}

