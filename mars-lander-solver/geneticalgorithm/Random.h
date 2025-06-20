#pragma once

#include <random>


namespace geneticalgorithm
{
  class Random
  {
    //std::random_device rd;
    std::mt19937 gen;

  public:
    Random(unsigned long seed) : gen{ seed } { gen.seed(seed); }

    auto&
    get() { return gen; }
  };
}