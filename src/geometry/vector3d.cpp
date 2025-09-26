#include "geometry/vector3d.h"

Vector3D::Vector3D(double x, double y, double z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

Vector3D::Vector3D()
{
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

Vector3D::~Vector3D()
{
}

double Vector3D::operator[](int axis) const
{
  if(axis == 0) return x;
  if(axis == 1) return y;
  return z;
}


Vector3D Vector3D::operator+(Vector3D const &other) const
{
  return Vector3D(x + other.x, y + other.y, z + other.z);
}

Vector3D Vector3D::operator-(Vector3D const &other) const
{
  return Vector3D(x - other.x, y - other.y, z - other.z);
}

Vector3D Vector3D::operator*(double lambda) const
{
  return Vector3D(lambda * x , lambda * y , lambda * z);
}

Vector3D Vector3D::operator+(double lambda) const
{
  return Vector3D(lambda + x , lambda + y , lambda + z);
}

Vector3D operator*(double s, const Vector3D& v) {
  return v * s;
}

double Vector3D::dot(Vector3D const &other) const
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

double Vector3D::length() const
{
  return sqrt(x * x + y * y + z*z);
}

Vector3D Vector3D::normalized() const
{
  double len = this->length();
  return Vector3D(this->x/len, this->y/len, this->z/len);
}

void Vector3D::normalize() 
{
  double len = this->length();
  x = x/len;
  y = y/len; 
  z = z/len;
}

void Vector3D::info()
{
  std::cout << "[" << x << "," << y << "," << z << "]\n";
}

Vector3D Vector3D::to_blender() const
{
  return Vector3D(x,z,-y);
}
