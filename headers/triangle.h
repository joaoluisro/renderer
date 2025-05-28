#pragma once

#ifndef TRIANGLE_HEADER
#define TRIANGLE_HEADER

#include "baseObject.h"

class Triangle : public BaseObject{
  public:
    Triangle(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D n0,Vector3D n1, Vector3D n2, Color color);
    ~Triangle();

    float intersects(Ray &r) const override;
    Vector3D get_normal(Vector3D &at) const override;
    Color get_color() const override;
    void translate(Vector3D &to) override;
    Vector3D centroid() const override; 
  private:
    Vector3D v0,v1,v2,n0,n1,n2;
    Color color;
    Vector3D normal;
};

#endif