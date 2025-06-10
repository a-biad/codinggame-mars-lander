#pragma once

#include "Pt2f.h"

#include <optional>

namespace simulation
{
  struct Collision
  {
    bool hasCollided;
    std::optional<Pt2f> intersection;
    std::optional<size_t> idxLIntersection;

    Collision() : hasCollided(false) {}
  };

} // simulation