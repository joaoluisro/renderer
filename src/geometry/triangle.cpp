#include <algorithm>

#include "geometry/triangle.h"

#define EPS 1e-8

Triangle::Triangle(
  Vector3D v0, 
  Vector3D v1, 
  Vector3D v2,
  Vector3D n0,
  Vector3D n1,
  Vector3D n2,
  Color color,
  Material mat
)
{
  this->v0 = v0;
  this->v1 = v1;
  this->v2 = v2;
  // if(fabs((n0 - n1).length()) < EPS)
  // {
  //   this->n0 = (v0 - v1).cross(v0 - v2);
  // }
  // else
  // {
    this->n0 = n0;
    this->n1 = n1;
    this->n2 = n2;
  // }
  this->color = color;
  this->mat = mat;
}

Triangle::~Triangle()
{
}

float Triangle::intersects(const Ray& r) const
{
  Vector3D v0v1 = v1 - v0;
  Vector3D v0v2 = v2 - v0;
  Vector3D N = v0v1.cross(v0v2); // N

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

Vector3D Triangle::centroid() const{
  return((v0 + v1 + v2)* (1.0/3.0));
}

Vector3D Triangle::get_normal(const Vector3D& at) const {
  Vector3D  e0   = v1 - v0;
  Vector3D  e1   = v2 - v0;
  Vector3D  N    = e0.cross(e1);

  auto x0 = v1 - at;
  auto x1 = v2 - at;
  auto x2 = v0 - at;
  float area2 = N.length();

  auto w0 = x0.cross( v2 - at).dot(N) / (area2*area2);
  auto w1 = x1.cross( v0 - at).dot(N) / (area2*area2);
  auto w2 = x2.cross( v1 - at).dot(N) / (area2*area2);

  // interpolates
  auto normal = n0 * w0 + n1 * w1 + n2 * w2;
  return normal;
}

Material Triangle::material()
{
  return mat;
}

Vector3D Triangle::max() const {

  auto max_x = std::max({v0.x,v1.x,v2.x});
  auto max_y = std::max({v0.y,v1.y,v2.y});
  auto max_z = std::max({v0.z,v1.z,v2.z});

  return Vector3D(max_x, max_y, max_z);
}

Vector3D Triangle::min() const {

  auto min_x = std::min({v0.x,v1.x,v2.x});
  auto min_y = std::min({v0.y,v1.y,v2.y});
  auto min_z = std::min({v0.z,v1.z,v2.z});
  return Vector3D(min_x, min_y, min_z);
}

bool Triangle::isOutOfBounds(const Vector3D& mx,const Vector3D& mn) const
{
  auto v_min = min();
  if(v_min.x < mn.x ||v_min.y < mn.y ||v_min.z < mn.z ) return true; 
  auto v_max = max();
  if(v_max.x > mx.x ||v_max.y > mx.y ||v_max.z > mx.z ) return true; 
  return false; 
}
