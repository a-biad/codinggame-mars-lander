#pragma once

#include <optional>
#include <queue>
#include <vector>

#include "../simulation/Simulation.h"
#include "Chromosome.h"
#include "DoubleBuffer.h"
#include "GeneticAlgorithmTrace.h"
#include "Parameters.h"
#include "Random.h"

namespace geneticalgorithm
{
  class GeneticAlgorithm
  {
    const Parameters parameters_;
    simulation::Simulation simulation_;
    Random random_;

    size_t generation_;

    struct Population
    {
      std::vector<Chromosome> chr;
      std::vector<Chromosome*> sortptr;
    };

    DoubleBuffer<Population> population_;

    GeneticAlgorithmTrace* gatrace_;

    Chromosome best;

    std::queue<simulation::ShipControl> scQueue_;

    void
    stepSimultationWithBest();

    void
    sortScoreAndUpdateBest();

    void
    addCurrentGenToTrace();

  public:
    GeneticAlgorithm(const Parameters& parameters,
                     const simulation::Simulation& simulation,
                     GeneticAlgorithmTrace* gatrace = nullptr);

    void
    execute(std::optional<size_t> geninc = std::nullopt);

    void
    flushSimulationWithBest();

    std::optional<simulation::ShipControl>
    getSc();

    const simulation::Simulation&
    simulation() const { return simulation_; }

    bool
    isDone() const
    {
      return simulation_.isDone() || (
               best.result().isEvaluated && best.result().hasLanded);
    }
  }; // geneticalgorithm
};