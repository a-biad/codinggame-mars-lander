#pragma once


#include <cstdint>
#include <cassert>
#include <bitset>
#include <optional>
#include <ostream>

#include "../simulation/Simulation.h"
#include "Random.h"

#include <iomanip>

static constexpr size_t NUM_ALLELE{ 128 };

using BitSet = std::bitset<NUM_ALLELE>;

namespace geneticalgorithm
{
  class Chromosome
  {
    struct EndData
    {
      Pt2f distance;
      float projectedDistance;
      Pt2f velocity;
      float angle;
      float duration;
      float fuelLeft;

      friend std::ostream&
      operator<<(std::ostream& os, const EndData& ed)
      {
        os //
          << "Dist: " << std::setw(6) << int(ed.projectedDistance) //
          << " Speed: " << std::setw(6) << int(ed.velocity.x) //
          << " " << std::setw(6) << int(ed.velocity.y) //
          << " Angle: " << std::setw(6) << ed.angle;
        return os;
      }
    };

    struct ChromosomeResultTrace
    {
      std::vector<simulation::Simulation> traj;
      bool isElite;
      EndData number;
      EndData score;
    };

    class Result
    {
    public:
      bool isEvaluated{ false };
      float score;
      bool hasLanded;
      std::optional<ChromosomeResultTrace> trace;
    };

    BitSet eAng_;
    BitSet dAng_;
    BitSet dPow_;
    Result result_;

    static BitSet
    randombs(Random& rand);

  public:
    void
    randomize(Random& rand);

    void
    mutate(Random& rand, float pMut);

    static void
    crossover(Random& rand,
              const Chromosome& p1,
              const Chromosome& p2,
              Chromosome& c1,
              Chromosome& c2);

    void
    timeShift(size_t t, Random& rand);

    const Result& evaluate(const simulation::Simulation& simulation, bool trace = false);

    BitSet const&
    eAng() const { return eAng_; }

    BitSet const&
    dAng() const { return dAng_; }

    BitSet const&
    dPow() const { return dPow_; }

    Result const&
    result() const { return result_; }
  };
}