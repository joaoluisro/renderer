#include "color.h"

Color::Color()
{
  r=0;
  g=0;
  b=0;
}

Color::Color(double r, double g, double b){
  this->r = r;
  this->g = g;
  this->b = b;
}
Color::~Color(){

}
Color Color::operator+(const Color& other) const
{
    return Color(other.r+r, other.g+g, other.b+b);
}
Color Color::operator-(const Color& other) const
{
    return Color(other.r-r, other.g-g, other.b-b);
}
Color Color::operator*(const Color& other) const
{
    return Color(other.r*r, other.g*g, other.b*b);
}

Color& Color::operator*=(float lambda)
{
  r *= lambda;
  g *= lambda;
  b *= lambda;
  return *this;
}

Color Color::clamp(const Color& c)
{
  return Color((c.r / (1 + c.r)), (c.g / (1 + c.g)), (c.b / (1 + c.b)));
}