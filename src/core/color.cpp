#include "core/color.h"
#include <iostream>
Radiance::Radiance()
{
  r=0;
  g=0;
  b=0;
}

Radiance::Radiance(float r, float g, float b){
  this->r = r;
  this->g = g;
  this->b = b;
}
Radiance::~Radiance(){

}
Radiance Radiance::operator+(const Radiance& other) const
{
    return Radiance(other.r+r, other.g+g, other.b+b);
}
Radiance Radiance::operator-(const Radiance& other) const
{
    return Radiance(other.r-r, other.g-g, other.b-b);
}
Radiance Radiance::operator*(Radiance& other)
{
    return Radiance(other.r*r, other.g*g, other.b*b);
}
Radiance& Radiance::operator*=(float lambda)
{
  r *= lambda;
  g *= lambda;
  b *= lambda;
  return *this;
}

Radiance Radiance::operator*(float lambda) const
{
  return Radiance(r*lambda, g*lambda, b*lambda);
}

Radiance Radiance::operator/(float lambda) const
{
    return Radiance(r/lambda, g/lambda, b/lambda);
}


Radiance& Radiance::operator+=(const Radiance &other)
{
  r += other.r;
  g += other.g;
  b += other.b;
  return *this;
}

void Radiance::info()
{
    std::cout << r << " " << g << " " << b << "\n";
}
