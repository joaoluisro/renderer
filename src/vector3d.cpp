#include "vector3d.h"

Vector3D::Vector3D(float x, float y, float z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

Vector3D::Vector3D()
{
  this->x = 0.0f;
  this->y = 0.0f;
  this->z = 0.0f;
}

Vector3D::~Vector3D()
{
}

Vector3D Vector3D::operator+(Vector3D const &other) const
{
  return Vector3D(this->x + other.x, this->y + other.y, this->z + other.z);
}

Vector3D Vector3D::operator-(Vector3D const &other) const
{
  return Vector3D(this->x - other.x, this->y - other.y, this->z - other.z);
}

Vector3D Vector3D::operator*(float lambda) const
{
  return Vector3D(lambda * this->x , lambda * this->y , lambda * this->z);
}

Vector3D operator*(float s, const Vector3D& v) {
  return v * s;
}

float Vector3D::dot(Vector3D const &other) const
{
  return (this->x * other.x + this->y * other.y + this->z * other.z);
}

Vector3D Vector3D::cross(Vector3D const &other) const
{
  return Vector3D(this->y * other.z - this->z * other.y,
     this->z * other.x - this->x * other.z, 
     this->x * other.y - this->y * other.x);
}

float Vector3D::length() const
{
  return sqrt(this->x * this->x + this->y * this->y + this->z*this->z);
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
  std::cout << "[" <<this->x << "," << this->y << "," << this->z << "]\n";
}