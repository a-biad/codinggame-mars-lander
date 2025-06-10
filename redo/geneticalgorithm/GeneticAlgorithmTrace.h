#pragma once


#include <optional>
#include <vector>

#include "../simulation/Simulation.h"
#include "Chromosome.h"
#include "Parameters.h"
#include "SimulationPlayer.h"

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
    GeneticAlgorithmTrace(Parameters const& parameters)
      : parameters_(parameters), genCursor_(0), chrCursor_(0) {}

    /** Add chromosome to the list. Higher score comes first */
    void
    addChromosome(Chromosome const& chr) { (generations_.end() - 1)->chr.push_back(chr); }

    void
    addGeneration(simulation::Simulation const& s)
    {
      generations_.push_back({ s, {} });
      (generations_.end() - 1)->chr.reserve(parameters_.numChromosome);
    }

    void
    changeGeneration(long diff)
    {
      genCursor_ += diff;
      genCursor_ %= (long)generations_.size();
      if(genCursor_ < 0)
        genCursor_ += (long)generations_.size();
      changeChromosome(0);
    }

    void
    changeChromosome(long diff)
    {
      chrCursor_ += diff;
      chrCursor_ %= (long)generations_[genCursor_].chr.size();
      if(chrCursor_ < 0)
        chrCursor_ += (long)generations_[genCursor_].chr.size();
    }

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