#ifndef RANDOMNUMBERGENERATOR_H
#define RANDOMNUMBERGENERATOR_H

#include <random>
#include "pcg-cpp-0.98/include/pcg_random.hpp"

class RandomNumberGenerator
{
public:
    RandomNumberGenerator();
    float generate();
public:
    pcg32 rng;
    std::uniform_real_distribution<float> dist;
};

#endif // RANDOMNUMBERGENERATOR_H
