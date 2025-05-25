#ifndef COLOR_HEADER
#define COLOR_HEADER

#include "vector3d.h"
#include <math.h>
class Color
{
  public:
    Color();
    Color(double r, double g, double b); 
    ~Color();
    Color operator+(const Color& other) const;
    Color operator-(const Color& other) const;
    Color operator*(const Color& other) const;
    Color& operator*=(float lambda);
    static Color clamp(const Color& c);
  public:
    double r,g,b;
};

#endif