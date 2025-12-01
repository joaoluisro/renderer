#pragma once

#ifndef TRIANGLE_HEADER
#define TRIANGLE_HEADER

#include "math/face.h"
#include "math/randomnumbergenerator.h"

class Triangle : public Face{
  public:
    Triangle(vec3 v1, vec3 v2, vec3 v3, vec3 n0,vec3 n1, vec3 n2, Radiance color);
    Triangle(vec3 v1, vec3 v2, vec3 v3, vec3 n, Radiance color);

    ~Triangle();

    inline float intersects(const Ray& r) const override;
    vec3 get_normal(const vec3& at) const override;
    Radiance get_color() const override;
    vec3 getCentroid() const override;
    vec3 max() const override;
    vec3 min() const override;
    bool isOutOfBounds(const vec3& mx, const vec3& mn) const override;
    vec3 generateUniform() const override;
    float getArea() const override;

  public:
    vec3 v0,v1,v2,n0,n1,n2,v0v1,v0v2,centroid,max_bound,min_bound;
    Radiance color;
    float area, area2_inv;
};

#endif
