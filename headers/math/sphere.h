#pragma once
#ifndef SPHERE_HEADER
#define SPHERE_HEADER

#include "math/face.h"

class Sphere : public Face{
  public:
    Sphere(vec3 &o, float r, const Radiance& c) :
    origin(o),radius(r), color(c){}
    ~Sphere();

    float intersects(const Ray& r) const override;
    vec3 get_normal(const vec3& at) const override;
    Radiance get_color() const override;
    vec3 getCentroid() const override;
    vec3 max() const override;
    vec3 min() const override;
    bool isOutOfBounds(const vec3& mx,const vec3& mn) const override;
    vec3 generateUniform() const override;
    float getArea() const override;
  private:
    float radius;
    vec3 origin;
    Radiance color;
};

#endif
