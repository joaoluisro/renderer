#include "math/sphere.h"

Sphere::~Sphere()
{
}

float Sphere::intersects(const Ray& r) const
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
      // no real roots, no intersection
      return -1;
  }

  // Compute the two intersection distances along the r
  float sqrt_disc = std::sqrt(disc);
  float inv2A   = 0.5f / A;
  float t0 = (-B - sqrt_disc) * inv2A;
  float t1 = (-B + sqrt_disc) * inv2A;
  // If either t0 or t1 is >= 0, the sphere is hit in front of the r origin
  return(t0);
}

Vector3D Sphere::get_normal(const Vector3D& at) const
{
  return Vector3D(at - origin) * (1/radius);
}

Color Sphere::get_color() const
{
  return color;
}

Vector3D Sphere::getCentroid() const
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

bool Sphere::isOutOfBounds(const Vector3D& mx, const Vector3D& mn) const
{
  return false; 
}

Vector3D Sphere::generateUniform() const
{
    return Vector3D(0,0,0);
}

float Sphere::getArea() const
{
    return 0.0f;
}
