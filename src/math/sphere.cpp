#include "math/sphere.h"

Sphere::~Sphere()
{
}

float Sphere::intersects(const Ray& r) const
{
  // Vector from sphere this->origin to r origin
  vec3 L = r.origin - this->origin;

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

vec3 Sphere::get_normal(const vec3& at) const
{
  return vec3(at - origin) * (1/radius);
}

Radiance Sphere::get_color() const
{
  return color;
}

vec3 Sphere::getCentroid() const
{
  return this->origin;
}
vec3 Sphere::max() const
{
  return this->origin + radius;
}
vec3 Sphere::min() const
{
  return this->origin + (-radius);
}

bool Sphere::isOutOfBounds(const vec3& mx, const vec3& mn) const
{
  return false; 
}

vec3 Sphere::generateUniform() const
{
    return vec3(0,0,0);
}

float Sphere::getArea() const
{
    return 0.0f;
}
