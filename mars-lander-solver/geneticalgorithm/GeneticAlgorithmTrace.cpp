//
// Created by ahmed on 29/05/25.
//

#include "GeneticAlgorithmTrace.h"

namespace geneticalgorithm
{
  GeneticAlgorithmTrace::GeneticAlgorithmTrace(Parameters const& parameters):
    parameters_(parameters), genCursor_(0), chrCursor_(0) {}

  void
  GeneticAlgorithmTrace::addChromosome(Chromosome const& chr)
  {
    (generations_.end() - 1)->chr.push_back(chr);
  }

  void
  GeneticAlgorithmTrace::addGeneration(simulation::Simulation const& s)
  {
    generations_.push_back({ s, {} });
    (generations_.end() - 1)->chr.reserve(parameters_.numChromosome);
  }

  void
  GeneticAlgorithmTrace::changeGenerationCursor(long diff, bool loop)
  {
    const long genMax{ (long)generations_.size() };
    genCursor_ += diff;
    if(!loop) { genCursor_ = std::clamp(genCursor_, 0L, genMax - 1); }
    else {
      genCursor_ %= genMax;
      if(genCursor_ < 0)
        genCursor_ += genMax;
    }
    changeChromosomeCursor(0); // Make sure the chrCursor has a valid value
  }

  void
  GeneticAlgorithmTrace::changeChromosomeCursor(long diff, bool loop)
  {
    const long chrMax{ (long)generations_[genCursor_].chr.size() };
    chrCursor_ += diff;
    if(!loop) { chrCursor_ = std::clamp(chrCursor_, 0L, chrMax - 1); }
    else {
      chrCursor_ %= chrMax;
      if(chrCursor_ < 0)
        chrCursor_ += chrMax;
    }
  }

  void
  GeneticAlgorithmTrace::setGenerationCursor(long genCursor)
  {
    const long genMax{ (long)generations_.size() };
    genCursor_ = std::clamp(genCursor, 0L, genMax - 1);
    changeChromosomeCursor(0);
  }

  void
  GeneticAlgorithmTrace::setChromosomeCursor(long chrCursor)
  {
    const long chrMax{ (long)generations_[genCursor_].chr.size() };
    chrCursor_ = std::clamp(chrCursor, 0L, chrMax - 1);
  }
} // geneticalgorithm