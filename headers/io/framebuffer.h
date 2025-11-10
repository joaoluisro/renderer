#pragma once 
#ifndef FRAMEBUFFER_HEADER
#define FRAMEBUFFER_HEADER

#include <vector>

#include "core/color.h"

struct Pixel { float r, g, b; };

class FrameBuffer{
  public:
    FrameBuffer(int width, int height): width(width), height(height), data(width*height) {}
    ~FrameBuffer();
    const Pixel& at(int i, int j) const;
    void clamp();
    void set(int i, int j, Color &c);
    void writeToPPM(const char* filename, int width, int height);

  private: 
    int width, height;
    std::vector<Pixel> data;
};

#endif
