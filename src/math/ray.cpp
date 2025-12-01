
#include "math/ray.h"

Ray::Ray(const vec3 &origin, const vec3 &direction) : origin(origin), direction(direction)
{
    invDir.x = 1.0f / direction.x;
    invDir.y = 1.0f / direction.y;
    invDir.z = 1.0f / direction.z;

    sign[0] = invDir.x < 0.0f;
    sign[1] = invDir.y < 0.0f;
    sign[2] = invDir.z < 0.0f;
}

Ray::~Ray()
{
}


