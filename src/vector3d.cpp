#include "vector3d.h"

Vector3D::Vector3D(float x, float y, float z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

Vector3D::Vector3D()
{
  x = 0.0f;
  y = 0.0f;
  z = 0.0f;
}

Vector3D::~Vector3D()
{
}

Vector3D Vector3D::operator+(Vector3D const &other) const
{
  return Vector3D(x + other.x, y + other.y, z + other.z);
}

Vector3D Vector3D::operator-(Vector3D const &other) const
{
  return Vector3D(x - other.x, y - other.y, z - other.z);
}

Vector3D Vector3D::operator*(float lambda) const
{
  return Vector3D(lambda * x , lambda * y , lambda * z);
}

Vector3D operator*(float s, const Vector3D& v) {
  return v * s;
}

float Vector3D::dot(Vector3D const &other) const
{
  return (x * other.x + y * other.y + z * other.z);
}

Vector3D Vector3D::cross(Vector3D const &other) const
{
  return Vector3D(
     y * other.z - z * other.y,
     z * other.x - x * other.z, 
     x * other.y - y * other.x);
}

float Vector3D::length() const
{
  return sqrt(x * x + y * y + z*z);
}

Vector3D Vector3D::normalized() const
{
  float len = this->length();
  return Vector3D(this->x/len, this->y/len, this->z/len);
}

void Vector3D::normalize() {
  float len = this->length();
  x = x/len;
  y = y/len; 
  z = z/len;
}

void Vector3D::info(){
  std::cout << "[" << x << "," << y << "," << z << "]\n";
}