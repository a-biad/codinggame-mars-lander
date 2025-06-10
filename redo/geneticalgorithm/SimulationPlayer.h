#pragma once

#include "../simulation/Simulation.h"
#include "Chromosome.h"

namespace geneticalgorithm
{
  class SimulationPlayer
  {
    simulation::Simulation simulation_;
    const Chromosome* chromosome_;
    size_t itchr_;

    /**
     * @return The difference of angle to apply so that it's a multiple of 15
     */
    int diffAngleToRound();

  public:
    SimulationPlayer(simulation::Simulation const& simulation,
                     const Chromosome& chromosome)
      : simulation_(simulation), chromosome_(&chromosome), itchr_(0) {}


    void step();

    const simulation::Simulation&
    simulation() { return simulation_; }

    bool
    isDone() const { return simulation_.isDone(); }

    size_t
    numAllelesConsumed() const { return itchr_; }
  };
} // geneticalgorithm