#include "geometry/sphere.h"

Sphere::~Sphere()
{
}

double Sphere::intersects(Ray &r) const
{
  // Vector from sphere this->origin to r origin
  Vector3D L = r.origin - this->origin;

  // Quadratic coefficients A t^2 + B t + C = 0
  double A = r.direction.dot(r.direction);
  double B = 2.0f * L.dot(r.direction);
  double C = L.dot(L) - radius*radius;

  // Discriminant
  double disc = B*B - 4.0f*A*C;
  if (disc < 0.0f) {
      // no real roots, no intersection
      return -1;
  }

  // Compute the two intersection distances along the r
  double sqrt_disc = std::sqrt(disc);
  double inv2A   = 0.5f / A;
  double t0 = (-B - sqrt_disc) * inv2A;
  double t1 = (-B + sqrt_disc) * inv2A;
  // If either t0 or t1 is >= 0, the sphere is hit in front of the r origin
  return(t0);
}

Vector3D Sphere::get_normal(Vector3D &at) const
{
  return Vector3D(at - origin) * (1/radius);
}

Color Sphere::get_color() const
{
  return color;
}

Material Sphere::material()
{
  return mat;
}

Vector3D Sphere::centroid() const
{
  return this->origin;
}
Vector3D Sphere::max() const
{
  return this->origin + radius;
}
Vector3D Sphere::min() const
{
  return this->origin + (-radius);
}

bool Sphere::isOutOfBounds(Vector3D &mx, Vector3D &mn) const
{
  return false; 
}
