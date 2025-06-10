#pragma once

#include <ostream>

#include "Pt2f.h"


namespace simulation
{
  struct ShipControl
  {
    int angle;
    int power;

    Pt2f acceleration() const;


    friend std::ostream& operator<<(std::ostream& os, const ShipControl& sc);
  };
}