#include "io/framebuffer.h"

FrameBuffer::~FrameBuffer()
{
}

const Pixel &FrameBuffer::at(int i, int j) const
{
  return data[i*width + j];
}

void FrameBuffer::clamp()
{
  float max_r = 0.0;
  float max_g = 0.0;
  float max_b = 0.0;

  for (const auto &px : data)
  {
    if (px.r > max_r) max_r = px.r;
    if (px.g > max_g) max_g = px.g;
    if (px.b > max_b) max_b = px.b;
  }

  for (auto &px : data) 
  {
    px.r = (px.r*(1 + (px.r/(max_r * max_r)))/(1 + px.r)) * 255.0;
    px.g = (px.g*(1 + (px.g/(max_g * max_g)))/(1 + px.g)) * 255.0;
    px.b = (px.b*(1 + (px.b/(max_b * max_b)))/(1 + px.b)) * 255.0;
  }
}

void FrameBuffer::set(int i, int j, Color &c)
{
  data[i*width + j].r = c.r;
  data[i*width + j].g = c.g;
  data[i*width + j].b = c.b;
}
