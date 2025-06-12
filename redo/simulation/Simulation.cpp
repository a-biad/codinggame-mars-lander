#include <istream>
#include <algorithm>

#include "Simulation.h"

namespace simulation
{
  void
  Simulation::fetchFromIstream(std::istream& s)
  {
    done_ = false;
    s >> state_.position.x >> state_.position.y >> state_.velocity.x >> state_.velocity.y
      >> state_.fuel >> state_.sc.angle >> state_.sc.power;
    s.ignore();
  }


  void
  Simulation::step()
  {
    if(!done_) {
      Pt2f posprev{ state_.position };

      state_.sc.angle += std::clamp(diffSC.angle, -15, +15);
      state_.sc.angle = std::clamp(state_.sc.angle, -90, +90);
      if(state_.fuel <= 0) { state_.sc.power = 0; }
      else {
        state_.sc.power += std::clamp(diffSC.power, -1, +1);
        state_.sc.power = std::clamp(state_.sc.power, 0, 4);
      }

      state_.fuel -= state_.sc.power;
      state_.velocity += state_.sc.acceleration();
      state_.position += state_.velocity;

      collision_ = land_->checkCollision(posprev, state_.position);
      if(collision_.hasCollided)
        done_ = true;

      state_.t++;
    }
  }

  bool
  Simulation::hasLanded() const
  {
    return collision_.hasCollided
           && collision_.idxLIntersection.value() == land_->idxLTarget()
           && collision_.intersection->x > land_->targetL().x + 25.f
           && collision_.intersection->x < land_->targetR().x - 25.f
           && state_.sc.angle == 0 && std::abs(state_.velocity.x) < 18.f
           && std::abs(state_.velocity.y) < 38.f;
  }
} // simulation