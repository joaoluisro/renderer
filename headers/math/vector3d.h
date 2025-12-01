#ifndef VECTOR_3D_HEADER
#define VECTOR_3D_HEADER

#include <math.h>
#include <iostream>
class vec3
{
public:
  float x,y,z;

public:
  vec3();
  vec3(float x, float y, float z);

  ~vec3();
  
  vec3& operator=(vec3 const&) = default;

  vec3 operator+(vec3 const& other) const;

  vec3 operator-(vec3 const& other) const;
  
  float operator[](int axis) const;

  float dot(vec3 const& other) const;
  
  vec3 cross(vec3 const& other) const;
  
  vec3 operator*(float lambda) const;
  vec3 operator+(float lambda) const;

  friend vec3 operator*(float s, const vec3& v);

  friend vec3 operator/(float s, const vec3& v);
  vec3 reflect(const vec3 &normal) const;

  float length() const;
  
  vec3 normalized() const;
  void normalize();
  void info();
  vec3 to_blender() const;
};

#endif
