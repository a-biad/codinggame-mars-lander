#pragma once

#include "Random.h"

namespace geneticalgorithm
{
  namespace Util
  {
    size_t
    tournamentSelection(int k, size_t n, Random& random);
  };
} // geneticalgorithm