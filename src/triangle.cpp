#include "triangle.h"

#define EPS 1e-8f
Triangle::Triangle(
  Vector3D v0, 
  Vector3D v1, 
  Vector3D v2,
  Vector3D n0,
  Vector3D n1,
  Vector3D n2,
  Color color
)
{
  this->v0 = v0;
  this->v1 = v1;
  this->v2 = v2;
  if(fabs((n0 - n1).length()) < EPS)
  {
    this->n0 = (v0 - v1).cross(v0 - v2);
  }
  else
  {
    this->n0 = n0;
    this->n1 = n1;
    this->n2 = n2;
  }
  this->color = color;
}

Triangle::~Triangle()
{
}

float Triangle::intersects(Ray &r) const 
{
  // Compute the plane's normal
  Vector3D v0v1 = v1 - v0;
  Vector3D v0v2 = v2 - v0;
  // No need to normalize
  Vector3D N = v0v1.cross(v0v2); // N

  // Step 1: Finding P
  float kEpsilon = 1e-6;
  // Check if the ray and plane are parallel
  float NdotRayDirection = N.dot(r.direction);
  if (fabs(NdotRayDirection) < kEpsilon) // Almost 0
      return -1; // They are parallel, so they don't intersect!

  // Compute d parameter using equation 2
  float d = -N.dot(v0);
  
  // Compute t (equation 3)
  float t = -(N.dot(r.origin) + d) / NdotRayDirection;
  
  // Check if the triangle is behind the ray
  if (t < 0) return -1; // The triangle is behind

  // Compute the intersection point using equation 1
  Vector3D P = r.origin + t * r.direction;

  // Step 2: Inside-Outside Test
  Vector3D Ne; // Vector perpendicular to triangle's plane

  // Test sidedness of P w.r.t. edge v0v1
  Vector3D v0p = P - v0;
  Ne = v0v1.cross(v0p);
  if (N.dot(Ne) < 0) return -1; // P is on the right side

  // Test sidedness of P w.r.t. edge v2v1
  Vector3D v2v1 = v2 - v1;
  Vector3D v1p = P - v1;
  Ne = v2v1.cross(v1p);
  if (N.dot(Ne) < 0) return -1; // P is on the right side

  // Test sidedness of P w.r.t. edge v2v0
  Vector3D v2v0 = v0 - v2; 
  Vector3D v2p = P - v2;
  Ne = v2v0.cross(v2p);
  if (N.dot(Ne) < 0) return -1; // P is on the right side

  return t; // The ray hits the triangle
}

Color Triangle::get_color() const
{
  return color;
}

void Triangle::translate(Vector3D &to)
{
  v0 = v0 + to;
  v1 = v1 + to;
  v2 = v2 + to;
}

Vector3D Triangle::centroid() const{
  return((v0 + v1 + v2)* (1.0f/3.0f));
}

Vector3D Triangle::get_normal(Vector3D &at) const {
  // Compute full‐triangle normal
  Vector3D  e0   = v1 - v0;
  Vector3D  e1   = v2 - v0;
  Vector3D  N    = e0.cross(e1);

  auto x0 = v1 - at;
  auto x1 = v2 - at;
  auto x2 = v0 - at;
  float area2 = N.length();           // twice the triangle area

  auto w0 = x0.cross( v2 - at).dot(N) / (area2*area2);
  // area opposite v1 is area(P,v2,v0)
  auto w1 = x1.cross( v0 - at).dot(N) / (area2*area2);
  // area opposite v2 is area(P,v0,v1)
  auto w2 = x2.cross( v1 - at).dot(N) / (area2*area2);

  auto normal = n0 * w0 + n1 * w1 + n2 * w2;
  return normal.normalized();
}