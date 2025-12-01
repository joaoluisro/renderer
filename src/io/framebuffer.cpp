#include "io/framebuffer.h"

#define cimg_display 0  // (no X11 needed)
#include "CImg-3.5.4/CImg.h"
using namespace cimg_library;

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
  // for (auto &px : data)
  // {
  //     px.r = (px.r/(1 + px.r)) * 255.0;
  //     px.g = (px.g/(1 + px.g)) * 255.0;
  //     px.b = (px.b/(1 + px.b)) * 255.0;
  // }
  // for (auto &px : data)
  // {
  //     px.r = (px.r/max_r) * 255.0;
  //     px.g = (px.g/max_g) * 255.0;
  //     px.b = (px.b/max_b) * 255.0;
  // }
}

Radiance FrameBuffer::getColor(int i, int j) const
{
    auto p = this->at(i,j);
    return Radiance(p.r, p.g, p.b);
}

void FrameBuffer::set(int i, int j, Radiance &c)
{
  data[i*width + j].r = c.r;
  data[i*width + j].g = c.g;
  data[i*width + j].b = c.b;
}

void FrameBuffer::writeToPPM(const char* filename, int width, int height)
{
    CImg<unsigned char> image(width, height, 1, 3, 0.5);
    for(auto i = 0; i < height; i++)
    {
        for(auto j = 0; j < width; j++)
        {
            image(i,j,0) = this->at(i,j).r;
            image(i,j,1) = this->at(i,j).g;
            image(i,j,2) = this->at(i,j).b;
        }
    }
    image.save(filename);
}

