#include "triangle.h"

Triangle::Triangle(
  Vector3D v0, 
  Vector3D v1, 
  Vector3D v2,
  Vector3D n0,
  Vector3D n1,
  Vector3D n2,
  Color color
)
{
  this->v0 = v0;
  this->v1 = v1;
  this->v2 = v2;
  this->n0 = n0;
  this->n1 = n1;
  this->n2 = n2;
  this->color = color;
}

Triangle::~Triangle()
{
}

bool Triangle::intersects(Ray &r, Vector3D &p) const 
{
  const float EPS = 1e-8f;
  Vector3D e1 = v1 - v0;
  Vector3D e2 = v2 - v0;
  Vector3D P  = r.direction.cross(e2);
  float det  = e1.dot(P);

  if (fabs(det) < EPS) return false;
  float invDet = 1.0f / det;

  Vector3D T = r.origin - v0;
  float u = T.dot(P) * invDet;
  if (u < 0.0f || u > 1.0f) return false;

  Vector3D Q = T.cross(e1);
  float v = r.direction.dot(Q) * invDet;
  if (v < 0.0f || u + v > 1.0f) return false;

  float t = e2.dot(Q) * invDet;
  if (t > EPS) {
      p = r.at(t);
      return true;
  }
  return false;
}


Color Triangle::get_color() const
{
  return color;
}

void Triangle::translate(Vector3D &to)
{
  v0 = v0 + to;
  v1 = v1 + to;
  v2 = v2 + to;
}

Vector3D Triangle::centroid() const{
  return((v0 + v1 + v2)* (1.0f/3.0f));
}

Vector3D Triangle::get_normal(Vector3D &at) const {
  return n1;
}