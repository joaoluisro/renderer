#ifndef VECTOR_3D_HEADER
#define VECTOR_3D_HEADER

#include <math.h>
#include <iostream>

class Vector3D
{
public:
  float x,y,z;

public:
  Vector3D();
  Vector3D(float x, float y, float z);

  ~Vector3D();
  
  Vector3D& operator=(Vector3D const&) = default;

  Vector3D operator+(Vector3D const& other) const;
  
  Vector3D operator-(Vector3D const& other) const;
  
  float dot(Vector3D const& other) const;
  
  Vector3D cross(Vector3D const& other) const;
  
  Vector3D operator*(float lambda) const;

  friend Vector3D operator*(float s, const Vector3D& v);

  friend Vector3D operator/(float s, const Vector3D& v);

  float length() const;
  
  Vector3D normalized() const;
  void normalize();
  void info();
};

#endif