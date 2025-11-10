#ifndef COLOR_HEADER
#define COLOR_HEADER
class Color
{
  public:
    Color();
    Color(float r, float g, float b);
    ~Color();
    Color operator+(const Color& other) const;
    Color operator-(const Color& other) const;
    Color operator*(Color& other);

    Color& operator*=(float lambda);
    Color operator*(float lambda) const;

    Color& operator+=(const Color &other);
    void info();
  public:
    float r,g,b;
};

#endif
