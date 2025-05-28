#include "color.h"

Color::Color()
{
  r=0;
  g=0;
  b=0;
}

Color::Color(float r, float g, float b){
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
Color Color::operator*(Color& other)
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

Color Color::operator*(float lambda) const
{
  return Color(r*lambda, g*lambda, b*lambda);
}

Color& Color::operator+=(const Color &other)
{
  r += other.r;
  g += other.g;
  b += other.b;
  return *this;
}