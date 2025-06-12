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
  return { float(-(float)power * sinf((float)angle / 180.f * MATH_PI)),
           float(G + (float)power * cosf((float)angle / 180.f * MATH_PI)) };
}

std::ostream&
simulation::operator<<(std::ostream& os, const ShipControl& sc)
{
  os << sc.angle << " " << sc.power;
  return os;
}