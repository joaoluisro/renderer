#pragma once

#ifndef TRIANGLE_HEADER
#define TRIANGLE_HEADER

#include "geometry/baseObject.h"

class Triangle : public BaseObject{
  public:
    Triangle(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D n0,Vector3D n1, Vector3D n2, Color color, Material mat);
    Triangle(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D n, Color color);

    ~Triangle();

    double intersects(Ray &r) const override;
    Vector3D get_normal(Vector3D &at) const override;
    Color get_color() const override;
    Vector3D centroid() const override; 
    Material material() override;
    Vector3D max() const override;
    Vector3D min() const override;
    bool isOutOfBounds(Vector3D &mx, Vector3D &mn) const override;

  private:
    Vector3D v0,v1,v2,n0,n1,n2;
    Color color;
    Vector3D normal;
    bool is_mirror;
    Material mat;
};

#endif