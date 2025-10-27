#ifndef VECTOR_3D_HEADER
#define VECTOR_3D_HEADER

#include <math.h>
#include <iostream>
class Vector3D
{
public:
  double x,y,z;

public:
  Vector3D();
  Vector3D(double x, double y, double z);

  ~Vector3D();
  
  Vector3D& operator=(Vector3D const&) = default;

  Vector3D operator+(Vector3D const& other) const;

  Vector3D operator-(Vector3D const& other) const;
  
  double operator[](int axis) const;

  double dot(Vector3D const& other) const;
  
  Vector3D cross(Vector3D const& other) const;
  
  Vector3D operator*(double lambda) const;
  Vector3D operator+(double lambda) const;

  friend Vector3D operator*(double s, const Vector3D& v);

  friend Vector3D operator/(double s, const Vector3D& v);

  double length() const;
  
  Vector3D normalized() const;
  void normalize();
  void info();
  Vector3D to_blender() const;
};

#endif
