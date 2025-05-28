#pragma once 
#ifndef FRAMEBUFFER_HEADER
#define FRAMEBUFFER_HEADER

#include <vector>
#include "color.h"
struct Pixel { float r, g, b; };

class Framebuffer 
{
  int w, h;
  std::vector<Pixel> data;
public:
  Framebuffer(int width, int height)
    : w(width), h(height), data(width*height) {}

  Pixel& at(int x, int y) 
  {
    return data[y*w + x];
  }
  void clamp()
  {
    float max_r = 0.0f;
    float max_g = 0.0f;
    float max_b = 0.0f;

    for (const auto &px : data) {
      if (px.r > max_r) max_r = px.r;
      if (px.g > max_g) max_g = px.g;
      if (px.b > max_b) max_b = px.b;
    }
    for (auto &px : data) {
      px.r = (px.r/max_r) * 255.0f;
      px.g = (px.g/max_g) * 255.0f;
      px.b = (px.b/max_b) * 255.0f;
    }
  }
  void set(int i, int j, Color &c)
  {
    this->at(i,j).r = c.r;
    this->at(i,j).g = c.g;
    this->at(i,j).b = c.b;
  }
};

#endif