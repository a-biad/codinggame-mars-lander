#pragma once


#include <optional>
#include <vector>

#include "../simulation/Simulation.h"
#include "Chromosome.h"
#include "Parameters.h"
#include "SimulationPlayer.h"

#include <algorithm>

namespace geneticalgorithm
{
  class GeneticAlgorithmTrace
  {
    // struct ChromosomeResultTemp
    // {
    //   Chromosome chr;
    //   float score;
    //   simulation::Collision collision;
    //   std::vector<const simulation::Simulation> traj;
    //   bool isElite;
    // };

    struct Generation
    {
      const simulation::Simulation simulationBegin;
      std::vector<Chromosome> chr;
    };

    const Parameters parameters_;
    std::vector<Generation> generations_;
    long genCursor_;
    long chrCursor_;

  public:
    GeneticAlgorithmTrace(Parameters const& parameters);

    /** Add chromosome to the list. Higher score comes first */
    void
    addChromosome(Chromosome const& chr);

    void
    addGeneration(simulation::Simulation const& s);

    void
    changeGenerationCursor(long diff, bool loop = true);

    void
    changeChromosomeCursor(long diff, bool loop = false);

    void
    setGenerationCursor(long genCursor);

    void
    setChromosomeCursor(long chrCursor);

    auto const&
    getAllGenerations() const { return generations_; }

    const Generation&
    generation() const { return generations_[genCursor_]; }

    const Chromosome&
    chromosome() const { return generations_[genCursor_].chr[chrCursor_]; }

    const simulation::Land&
    land() const { return *generations_[0].simulationBegin.land(); }

    size_t
    genCursor() const { return genCursor_; }

    size_t
    chrCursor() const { return chrCursor_; }

    const Parameters&
    parameters() { return parameters_; }
  };
} // geneticalgorithm