# Physically Based Renderer

A renderer written in C++ using CImg and Tinyobjloader to render photo-realistic scenes through ray tracing.

## Building
```
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

After creating a build directory, use `ninja` to compile the source code

```
cd build && ninja
```

## Usage

```
./renderer [.obj path] [width] [height] [BVH param] [BVH type]
```

- BVH param dictates either the primitive/leaf in Median/Midpoint schemes or the bin count in SAH.
- BVH types available are: `median`, `midpoint` and `sah`.

Example: 

```
./renderer lucy.obj 1024 1024 16 sah
```