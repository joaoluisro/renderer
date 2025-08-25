#pragma once 
#ifndef FRAMEBUFFER_HEADER
#define FRAMEBUFFER_HEADER

#include <vector>

#include "core/camera.h"

struct Pixel { double r, g, b; };

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
    double max_r = 0.0;
    double max_g = 0.0;
    double max_b = 0.0;

    for (const auto &px : data) {
      if (px.r > max_r) max_r = px.r;
      if (px.g > max_g) max_g = px.g;
      if (px.b > max_b) max_b = px.b;
    }

    for (auto &px : data) {
      px.r = (px.r*(1 + (px.r/(max_r * max_r)))/(1 + px.r)) * 255.0;
      px.g = (px.g*(1 + (px.g/(max_g * max_g)))/(1 + px.g)) * 255.0;
      px.b = (px.b*(1 + (px.b/(max_b * max_b)))/(1 + px.b)) * 255.0;
    }
    // for (auto &px : data) {
      // px.r = (px.r/(1 + px.r)) * 255.0;
      // px.g = (px.g/(1 + px.g)) * 255.0;
      // px.b = (px.b/(1 + px.b)) * 255.0;
    // }
  }
  void set(int i, int j, Color &c)
  {
    this->at(i,j).r = c.r;
    this->at(i,j).g = c.g;
    this->at(i,j).b = c.b;
  }
};

#endif