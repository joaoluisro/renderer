#pragma once
#ifndef SPHERE_HEADER
#define SPHERE_HEADER

#include "geometry/face.h"

class Sphere : public Face{
  public:
    Sphere(vec3 &o, float r, const Spectrum& c) :
    origin(o),radius(r), color(c){}
    ~Sphere();

    float intersects(const Ray& r) const override;
    vec3 getNormal(const vec3& at) const override;
    Spectrum getColor() const override;
    vec3 getCentroid() const override;
    vec3 max() const override;
    vec3 min() const override;
    bool isOutOfBounds(const vec3& mx,const vec3& mn) const override;
    vec3 generateUniform() const override;
    float getArea() const override;
  private:
    float radius;
    vec3 origin;
    Spectrum color;
};

#endif
