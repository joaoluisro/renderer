#include <algorithm>

#include "math/triangle.h"

#define EPS 1e-8

Triangle::Triangle(
  Vector3D v0, 
  Vector3D v1, 
  Vector3D v2,
  Vector3D n0,
  Vector3D n1,
  Vector3D n2,
  Color color)
    : v0(v0), v1(v1), v2(v2), n0(n0), n1(n1), n2(n2), color(color)
{
   v0v1 = v1 - v0;
   v0v2 = v2 - v0;
   area = v0v1.cross(v0v2).length();
   area2_inv = 1.0f/(area * area);
   centroid = (v0 + v1 + v2)* (1.0/3.0);
   auto max_x = std::max({v0.x,v1.x,v2.x});
   auto max_y = std::max({v0.y,v1.y,v2.y});
   auto max_z = std::max({v0.z,v1.z,v2.z});

   max_bound = Vector3D(max_x, max_y, max_z);

   auto min_x = std::min({v0.x,v1.x,v2.x});
   auto min_y = std::min({v0.y,v1.y,v2.y});
   auto min_z = std::min({v0.z,v1.z,v2.z});
   min_bound = Vector3D(min_x, min_y, min_z);

}

Triangle::~Triangle()
{
}

inline float Triangle::intersects(const Ray& r) const
{
  auto N = n0;
  // Check if the ray and plane are parallel
  float n_dot_r = N.dot(r.direction);
  if (fabs(n_dot_r) < EPS) // Almost 0
  {
      return -1;
  }

  float d = -N.dot(v0);
  
  float t = -(N.dot(r.origin) + d) / n_dot_r;
  
  // Check if the triangle is behind the ray
  if (t < 0)
  {
      return -1;
  }

  // Muller-Trombore
  Vector3D P = r.origin + t * r.direction;

  Vector3D Ne;

  Vector3D v0p = P - v0;
  Ne = v0v1.cross(v0p);
  if (N.dot(Ne) < 0) return -1;

  Vector3D v2v1 = v2 - v1;
  Vector3D v1p = P - v1;
  Ne = v2v1.cross(v1p);
  if (N.dot(Ne) < 0) return -1;

  Vector3D v2v0 = v0 - v2;
  Vector3D v2p = P - v2;
  Ne = v2v0.cross(v2p);
  if (N.dot(Ne) < 0) return -1;

  return t;
}

Color Triangle::get_color() const
{
  return Color(color.r,color.g,color.b);
}

Vector3D Triangle::getCentroid() const{
    return centroid;
}

Vector3D Triangle::get_normal(const Vector3D& at) const {
    return n0;

  // auto x0 = v1 - at;
  // auto x1 = v2 - at;
  // auto x2 = v0 - at;
  // auto N = n0;
  // auto w0 = x0.cross( v2 - at).dot(N) * area2_inv;
  // auto w1 = x1.cross( v0 - at).dot(N) * area2_inv;
  // auto w2 = x2.cross( v1 - at).dot(N) * area2_inv;

  // return n0 * w0 + n1 * w1 + n2 * w2;
}

Vector3D Triangle::max() const {

    return max_bound;

}

Vector3D Triangle::min() const {
    return min_bound;

}

bool Triangle::isOutOfBounds(const Vector3D& mx,const Vector3D& mn) const
{
  auto v_min = min();
  if(v_min.x < mn.x ||v_min.y < mn.y ||v_min.z < mn.z ) return true;
  auto v_max = max();
  if(v_max.x > mx.x ||v_max.y > mx.y ||v_max.z > mx.z ) return true;
  return false;
}

Vector3D Triangle::generateUniform() const
{
    static RandomNumberGenerator rng;
    float e1 = rng.generate();
    float e2 = rng.generate();
    float sqrte1 = sqrt(e1);
    float u = 1 - sqrte1;
    float v = sqrte1 * (1 - e2);
    float w = sqrte1 * e2;
    return (v0 * u) + (v1 * v) + (v2 * w);
}

float Triangle::getArea() const
{
    return area;
}
