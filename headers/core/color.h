#ifndef COLOR_HEADER
#define COLOR_HEADER
class Color
{
  public:
    Color();
    Color(double r, double g, double b); 
    ~Color();
    Color operator+(const Color& other) const;
    Color operator-(const Color& other) const;
    Color operator*(Color& other);

    Color& operator*=(double lambda);
    Color operator*(double lambda) const;

    Color& operator+=(const Color &other);
  public:
    double r,g,b;
};

#endif