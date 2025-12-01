#ifndef COLOR_HEADER
#define COLOR_HEADER
class Radiance
{
  public:
    Radiance();
    Radiance(float r, float g, float b);
    ~Radiance();
    Radiance operator+(const Radiance& other) const;
    Radiance operator-(const Radiance& other) const;
    Radiance operator*(Radiance& other);

    Radiance& operator*=(float lambda);
    Radiance operator*(float lambda) const;
    Radiance operator/(float lambda) const;

    Radiance& operator+=(const Radiance &other);
    void info();
  public:
    float r,g,b;
};

#endif
