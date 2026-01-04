#ifndef SPECTRUM_HEADER
#define SPECTRUM_HEADER

#pragma once

#include <iostream>

struct Spectrum {
    float r, g, b;

    Spectrum() : r(0.0f), g(0.0f), b(0.0f) {}
    Spectrum(float rr, float gg, float bb) : r(rr), g(gg), b(bb) {}

    Spectrum operator+(const Spectrum& other) const {
        return Spectrum(r + other.r, g + other.g, b + other.b);
    }

    Spectrum operator-(const Spectrum& other) const {
        return Spectrum(r - other.r, g - other.g, b - other.b);
    }

    Spectrum operator*(const Spectrum& other) const {
        return Spectrum(r * other.r, g * other.g, b * other.b);
    }

    Spectrum& operator+=(const Spectrum& other) {
        r += other.r;
        g += other.g;
        b += other.b;
        return *this;
    }

    Spectrum& operator*=(float lambda) {
        r *= lambda;
        g *= lambda;
        b *= lambda;
        return *this;
    }

    Spectrum operator*(float lambda) const {
        return Spectrum(r * lambda, g * lambda, b * lambda);
    }

    Spectrum operator/(float lambda) const {
        const float inv = 1.0f / lambda;
        return Spectrum(r * inv, g * inv, b * inv);
    }

    void info() const {
        std::cout << "Spectrum(" << r << ", " << g << ", " << b << ")\n";
    }
};



#endif
