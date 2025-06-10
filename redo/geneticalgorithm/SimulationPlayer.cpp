#include "SimulationPlayer.h"

namespace geneticalgorithm
{
  int
  SimulationPlayer::diffAngleToRound()
  {
    int angle{ simulation_.currentSC().angle };
    int modangle{ angle % 15 };
    modangle = angle > 0 ? -modangle : modangle;
    return modangle;
  }

  void
  SimulationPlayer::step()
  {
    auto anglediff{ diffAngleToRound() };
    if(anglediff != 0) { simulation_.diffAngle(anglediff).diffPower(+1); }
    else {
      if(itchr_ != NUM_ALLELE) {
        simulation_.diffAngle(chromosome_->eAng()[itchr_]
                                ? (chromosome_->dAng()[itchr_] ? +15 : -15)
                                : 0)
                   .diffPower(chromosome_->dPow()[itchr_] ? +1 : -1);
        itchr_++;
      }
      else {
        simulation_.diffAngle(0).diffPower(0);
      }
    }
    simulation_.step();
  }
} // geneticalgorithm