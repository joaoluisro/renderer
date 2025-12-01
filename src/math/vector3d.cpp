#include "math/vector3d.h"

vec3::vec3(float x, float y, float z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

vec3::vec3()
{
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

vec3::~vec3()
{
}

float vec3::operator[](int axis) const
{
  if(axis == 0) return x;
  if(axis == 1) return y;
  return z;
}


vec3 vec3::operator+(vec3 const &other) const
{
  return vec3(x + other.x, y + other.y, z + other.z);
}

vec3 vec3::operator-(vec3 const &other) const
{
  return vec3(x - other.x, y - other.y, z - other.z);
}

vec3 vec3::operator*(float lambda) const
{
  return vec3(lambda * x , lambda * y , lambda * z);
}

vec3 vec3::operator+(float lambda) const
{
  return vec3(lambda + x , lambda + y , lambda + z);
}

vec3 operator*(float s, const vec3& v) {
  return v * s;
}

float vec3::dot(vec3 const &other) const
{
  return (x * other.x + y * other.y + z * other.z);
}

vec3 vec3::cross(vec3 const &other) const
{
  return vec3(
     y * other.z - z * other.y,
     z * other.x - x * other.z, 
     x * other.y - y * other.x);
}

float vec3::length() const
{
  return sqrt(x * x + y * y + z*z);
}

vec3 vec3::normalized() const
{
  float len = this->length();
  return vec3(this->x/len, this->y/len, this->z/len);
}

void vec3::normalize() 
{
  float len = this->length();
  x = x/len;
  y = y/len; 
  z = z/len;
}

void vec3::info()
{
  std::cout << "[" << x << "," << y << "," << z << "]\n";
}

vec3 vec3::to_blender() const
{
  return vec3(x,z,-y);
}

vec3 vec3::reflect(const vec3& normal) const
{
    return *this - 2*(this->dot(normal))*normal;
}
