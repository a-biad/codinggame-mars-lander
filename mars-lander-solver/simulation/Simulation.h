#pragma once

#include "Land.h"
#include "ShipControl.h"

namespace simulation
{
  struct State
  {
    ShipControl sc;
    Pt2f velocity;
    Pt2f position;
    int fuel;
    int t;
  };

  class Simulation
  {
    const Land* land_;

    State state_;

    bool done_;
    Collision collision_;

  public:
    /**
     * The variation of angle and power to be applied for next step
     */
    ShipControl diffSC;

    Simulation() : done_(true) {}

    Simulation(const Simulation&) = default;

    Simulation(const Land& land,
               const ShipControl& currentSC,
               const Pt2f& position,
               int fuel)
      : land_(&land), state_{ currentSC, { 0, 0 }, position, fuel, 0 }, done_(false) {}

    Simulation(const Land& land, std::istream& s) : land_(&land) { fetchFromIstream(s); }

    void fetchFromIstream(std::istream& s);

    void step();

    Simulation&
    targetAngle(int angle)
    {
      diffSC.angle = angle - state_.sc.angle;
      return *this;
    }

    Simulation&
    targetPower(int power)
    {
      diffSC.power = power - state_.sc.power;
      return *this;
    }

    Simulation&
    diffAngle(int diffangle)
    {
      diffSC.angle = diffangle;
      return *this;
    }

    Simulation&
    diffPower(int diffpower)
    {
      diffSC.power = diffpower;
      return *this;
    }

    bool
    isDone() const { return done_; }

    bool
    hasLanded() const;

    const ShipControl&
    currentSC() const { return state_.sc; }

    const Collision&
    collision() const { return collision_; }

    int
    t() const { return state_.t; }

    const Pt2f&
    velocity() const { return state_.velocity; }

    const Pt2f&
    position() const { return state_.position; }

    int
    fuel() const { return state_.fuel; }

    State const&
    state() { return state_; }

    const Land*
    land() const { return land_; }
  };
} // simulation