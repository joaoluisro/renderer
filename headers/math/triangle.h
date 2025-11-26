#pragma once

#ifndef TRIANGLE_HEADER
#define TRIANGLE_HEADER

#include "math/face.h"
#include "math/randomnumbergenerator.h"

class Triangle : public Face{
  public:
    Triangle(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D n0,Vector3D n1, Vector3D n2, Color color);
    Triangle(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D n, Color color);

    ~Triangle();

    inline float intersects(const Ray& r) const override;
    Vector3D get_normal(const Vector3D& at) const override;
    Color get_color() const override;
    Vector3D getCentroid() const override;
    Vector3D max() const override;
    Vector3D min() const override;
    bool isOutOfBounds(const Vector3D& mx, const Vector3D& mn) const override;
    Vector3D generateUniform() const override;
    float getArea() const override;

  public:
    Vector3D v0,v1,v2,n0,n1,n2,v0v1,v0v2,centroid,max_bound,min_bound;
    Color color;
    float area, area2_inv;
};

#endif
