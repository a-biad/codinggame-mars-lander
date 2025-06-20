//
// Created by ahmed on 25/05/25.
//


#include <cmath>

#include "ShipControl.h"
#include "Constants.h"

using namespace simulation;


Pt2f
ShipControl::acceleration() const
{
  return {
    -static_cast<Float>(power) * std::sin(static_cast<Float>(angle) / 180._f * MATH_PI),
    G + static_cast<Float>(power) * std::cos(
      static_cast<Float>(angle) / 180._f * MATH_PI) };
}

std::ostream&
simulation::operator<<(std::ostream& os, const ShipControl& sc)
{
  os << sc.angle << " " << sc.power;
  return os;
}