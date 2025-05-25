#include "sphere.h"

Sphere::~Sphere()
{
}

bool Sphere::intersects(Ray &r, Vector3D &p) const
{
  // Vector from sphere this->origin to r origin
  Vector3D L = r.origin - this->origin;

  // Quadratic coefficients A t^2 + B t + C = 0
  float A = r.direction.dot(r.direction);
  float B = 2.0f * L.dot(r.direction);
  float C = L.dot(L) - radius*radius;

  // Discriminant
  float disc = B*B - 4.0f*A*C;
  if (disc < 0.0f) {
      // no real roots ⇒ no intersection
      return false;
  }

  // Compute the two intersection distances along the r
  float sqrt_disc = std::sqrt(disc);
  float inv2A   = 0.5f / A;
  float t0 = (-B - sqrt_disc) * inv2A;
  float t1 = (-B + sqrt_disc) * inv2A;
  p = r.at(t0);
  // If either t0 or t1 is ≥ 0, the sphere is hit in front of the r origin
  return(true);
}

Vector3D Sphere::get_normal(Vector3D &at) const
{
  return Vector3D(at - origin) * (1/radius);
}

Color Sphere::get_color() const
{
  return color;
}

void Sphere::translate(Vector3D &to) 
{

}
Vector3D Sphere::centroid() const
{}
