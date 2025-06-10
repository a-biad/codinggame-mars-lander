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
  return { float(-(float)power * std::sinf((float)angle / 180.f * PI)),
           float(G + (float)power * std::cosf((float)angle / 180.f * PI)) };
}

std::ostream&
simulation::operator<<(std::ostream& os, const ShipControl& sc)
{
  os << sc.angle << " " << sc.power;
  return os;
}