//
// Created by ahmed on 28/05/25.
//

#include "Util.h"

namespace geneticalgorithm
{
  size_t
  Util::tournamentSelection(int k, size_t n, Random& random)
  {
    std::uniform_int_distribution r{ 0UL, n };
    if(k <= 0)
      return 0;
    size_t best{ r(random.get()) };
    for(int i{ 0 }; i < k; i++) {
      size_t idx{ r(random.get()) };
      best = std::min(idx, best);
    }
    return best;
  }
} // geneticalgorithm