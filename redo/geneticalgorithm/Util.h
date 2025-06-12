#pragma once

#include "Random.h"

#define CGEXPORT 0

namespace geneticalgorithm
{
  namespace Util
  {
    size_t
    tournamentSelection(int k, size_t n, Random& random);
  };
} // geneticalgorithm