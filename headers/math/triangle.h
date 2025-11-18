#pragma once

#ifndef TRIANGLE_HEADER
#define TRIANGLE_HEADER

#include "math/face.h"

class Triangle : public Face{
  public:
    Triangle(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D n0,Vector3D n1, Vector3D n2, Color color);
    Triangle(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D n, Color color);

    ~Triangle();

    float intersects(const Ray& r) const override;
    Vector3D get_normal(const Vector3D& at) const override;
    Color get_color() const override;
    Vector3D centroid() const override; 
    Vector3D max() const override;
    Vector3D min() const override;
    bool isOutOfBounds(const Vector3D& mx, const Vector3D& mn) const override;
    Vector3D generateUniform(float e1, float e2) const override;
    float getArea() const override;

  private:
    Vector3D v0,v1,v2,n0,n1,n2;
    Color color;
    Vector3D normal;
    bool is_mirror;
};

#endif
