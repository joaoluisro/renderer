#include "math/randomnumbergenerator.h"

RandomNumberGenerator::RandomNumberGenerator()
{
    pcg32 rng(pcg_extras::seed_seq_from<std::random_device>{});
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    this->rng = rng;
    this->dist = dist;
}

float RandomNumberGenerator::generate()
{
    return this->dist(this->rng);
}
