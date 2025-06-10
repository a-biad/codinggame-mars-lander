#pragma once

#include "Random.h"

#define CGEXPORT 1

namespace geneticalgorithm
{
  namespace Util
  {
    size_t
    tournamentSelection(int k, size_t n, Random& random);
  };
} // geneticalgorithm