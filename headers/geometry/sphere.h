#pragma once
#ifndef SPHERE_HEADER
#define SPHERE_HEADER

#include "geometry/baseObject.h"

class Sphere : public BaseObject{
  public:
    Sphere(Vector3D &o, double r, const Color& c) : 
    origin(o),radius(r), color(c){}
    ~Sphere();

    double intersects(Ray r) const override;
    Vector3D get_normal(Vector3D &at) const override;
    Color get_color() const override;
    Material material() override;
    Vector3D centroid() const override; 
    Vector3D max() const override;
    Vector3D min() const override;
    bool isOutOfBounds(Vector3D &mx, Vector3D &mn) const override;

  private:
    double radius;
    Vector3D origin;
    Color color;
    Material mat;
};

#endif
