#include "Chromosome.h"

#include "SimulationPlayer.h"
#include "../simulation/Constants.h"

namespace geneticalgorithm
{
  BitSet
  Chromosome::randombs(Random& rand)
  {
    using Assigner = uint64_t;
    static constexpr auto SZBS{ NUM_ALLELE };
    static constexpr auto SZAS{ sizeof(Assigner) * 8 };
    static constexpr auto NUMIT{ SZBS / SZAS };
    static constexpr auto MXAS{ std::numeric_limits<Assigner>::max() };

    static std::uniform_int_distribution<Assigner> rand0{ Assigner{ 0 }, MXAS };

    BitSet ret{ rand0(rand.get()) };

    for(auto i{ NUMIT }; i != 0; i--) {
      ret <<= SZAS;
      ret |= rand0(rand.get());
    }
    return ret;
  }

  void
  Chromosome::randomize(Random& rand)
  {
    eAng_ = randombs(rand);
    dAng_ = randombs(rand);
    dPow_ = randombs(rand);
  }

  void
  Chromosome::mutate(Random& rand, float pMut)
  {
    std::bernoulli_distribution trial(pMut);
    for(auto& bs : { &eAng_, &dAng_, &dPow_ }) {
      for(size_t i{ 0 }; i != NUM_ALLELE; i++) {
        if(trial(rand.get()))
          bs->flip(i);
      }
    }
    result_.isEvaluated = false;
  }

  void
  Chromosome::crossover(Random& rand,
                        const Chromosome& p1,
                        const Chromosome& p2,
                        Chromosome& c1,
                        Chromosome& c2)
  {
    static std::uniform_int_distribution<size_t> pt0{ 1, NUM_ALLELE - 2 };
    size_t pt{ pt0(rand.get()) };
    BitSet fil{ BitSet{ 0 }.flip() << pt };
    c1.eAng_ = p1.eAng_ & fil | p2.eAng_ & ~fil;
    c2.eAng_ = p2.eAng_ & fil | p1.eAng_ & ~fil;
    c1.dAng_ = p1.dAng_ & fil | p2.dAng_ & ~fil;
    c2.dAng_ = p2.dAng_ & fil | p1.dAng_ & ~fil;
    c1.dPow_ = p1.dPow_ & fil | p2.dPow_ & ~fil;
    c2.dPow_ = p2.dPow_ & fil | p1.dPow_ & ~fil;
    c1.result_.isEvaluated = false;
    c2.result_.isEvaluated = false;
  }

  void
  Chromosome::timeShift(size_t t, Random& rand)
  {
    static std::bernoulli_distribution b{ 0.5 };
    eAng_ >>= t;
    dAng_ >>= t;
    dPow_ >>= t;
    for(size_t i{ NUM_ALLELE - t }; i < NUM_ALLELE; i++) {
      eAng_[i] = b(rand.get());
      dAng_[i] = b(rand.get());
      dPow_[i] = b(rand.get());
    }
    if(result_.isEvaluated && result_.trace.has_value()) {
      auto& traj{ result_.trace->traj };
      traj.erase(traj.begin(), traj.begin() + t);
    }
  }

  const Chromosome::Result&
  Chromosome::evaluate(const simulation::Simulation& simulation, const bool trace)
  {
    if(result_.isEvaluated)
      return result_;
    result_.isEvaluated = true;

    // Simulation
    SimulationPlayer simul{ simulation, *this };
    if(trace) { result_.trace = ChromosomeResultTrace{}; }
    while(!simul.isDone()) {
      if(trace) { result_.trace->traj.push_back(simul.simulation()); }
      simul.step();
    }
    if(trace) { result_.trace->traj.push_back(simul.simulation()); }

    // Score calculation
    result_.score = 0.f;
    const simulation::Simulation& resultSimu{ simul.simulation() };
    float projectedDistance{
      simul.simulation().land()->projectedDistance(resultSimu.collision()) };
    if(trace) {
      result_.trace->number.distance =
        resultSimu.position() - resultSimu.land()->targetL();
      result_.trace->number.projectedDistance = projectedDistance;
      result_.trace->number.velocity = resultSimu.velocity();
      result_.trace->number.angle = resultSimu.currentSC().angle;
      result_.trace->number.duration = resultSimu.t();
      result_.trace->number.fuelLeft = resultSimu.fuel();
    };

    EndData scoreDetails = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };

    // Distance penalty
    if(projectedDistance < 500) {
      // Decreased penalty if already on target
      scoreDetails.projectedDistance = -projectedDistance / 100.f;
    }
    else { scoreDetails.projectedDistance = -projectedDistance; }
    result_.score += scoreDetails.projectedDistance;
    // Speed penalty
    scoreDetails.velocity = {
      -std::max(simulation::LANDING_SPEED_LIMIT.x - 5._f,
                std::abs(resultSimu.velocity().x)),
      -std::max(simulation::LANDING_SPEED_LIMIT.y - 5._f,
                std::abs(resultSimu.velocity().y)) };
    result_.score += scoreDetails.velocity.x + scoreDetails.velocity.y;
    // Angle penalty
    scoreDetails.angle = -(std::abs(resultSimu.currentSC().angle) / 90.f);
    result_.score += scoreDetails.angle;
    // Landing bonus
    result_.hasLanded = resultSimu.hasLanded();
    if(result_.hasLanded)
      result_.score += 10000.f;

    if(trace) { result_.trace->score = scoreDetails; }

    return result_;
  }
}