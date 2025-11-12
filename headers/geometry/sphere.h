#pragma once
#ifndef SPHERE_HEADER
#define SPHERE_HEADER

#include "geometry/face.h"

class Sphere : public Face{
  public:
    Sphere(Vector3D &o, float r, const Color& c) : 
    origin(o),radius(r), color(c){}
    ~Sphere();

    float intersects(const Ray& r) const override;
    Vector3D get_normal(const Vector3D& at) const override;
    Color get_color() const override;
    Vector3D centroid() const override; 
    Vector3D max() const override;
    Vector3D min() const override;
    bool isOutOfBounds(const Vector3D& mx,const Vector3D& mn) const override;
    Vector3D generateUniform(float e1, float e2) const override;
    float getArea() const override;
  private:
    float radius;
    Vector3D origin;
    Color color;
};

#endif
