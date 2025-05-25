#pragma once
#ifndef SPHERE_HEADER
#define SPHERE_HEADER

#include "baseObject.h"

class Sphere : public BaseObject{
  public:
    Sphere(Vector3D &o, float r, const Color& c) : 
    origin(o),radius(r), color(c){}
    ~Sphere();

    bool intersects(Ray &r, Vector3D &p) const override;
    Vector3D get_normal(Vector3D &at) const override;
    Color get_color() const override;
    void translate(Vector3D &to) override;
    Vector3D centroid() const override; 
  private:
    float radius;
    Vector3D origin;
    Color color;
};

#endif