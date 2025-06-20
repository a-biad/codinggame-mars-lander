#include <algorithm>
#include <iomanip>
#include <iostream>

#include "GeneticAlgorithm.h"
#include "SimulationPlayer.h"
#include "Util.h"

namespace geneticalgorithm {
  void
  GeneticAlgorithm::stepSimultationWithBest() {
    if(!simulation_.isDone()) {
      SimulationPlayer sp{ simulation_, best };
      sp.step();
      simulation_ = sp.simulation();
      scQueue_.push(sp.simulation().currentSC());
      for(auto& chr : population_.curr().chr) {
        chr.timeShift(sp.numAllelesConsumed(), random_);
      }
      best.timeShift(sp.numAllelesConsumed(), random_);
    }
  }

  void
  GeneticAlgorithm::sortScoreAndUpdateBest() {
    // using ES = ElemScore;
    // auto sortOp = [](const ES& a, const ES& b) { return a.res.score > b.res.score; };
    // std::sort(score_.begin(), score_.end(), sortOp);
    // if(score_[0].res.score > best.scr) {
    //   best.scr = score_[0].res.score;
    //   best.chr = *score_[0].chrptr;
    //   std::cerr << "-- Generation : " << std::setw(6) << generation_
    //     << " | Score : " << std::setw(10) << best.scr << "\n";
    // }

    auto sortLambda = [](const Chromosome* a, const Chromosome* b) {
      return a->result().score > b->result().score;
    };
    std::sort(population_.curr().sortptr.begin(),
              population_.curr().sortptr.end(),
              sortLambda);

    if(!best.result().isEvaluated || population_.curr().sortptr[0]->result().score >
       best.result().score) {
      best = *population_.curr().sortptr[0];
      std::cerr << "-- Generation : " << std::setw(6) << generation_
        << " | Score : " << std::setw(10) << best.result().score << "\n";
    }
  }

  void
  GeneticAlgorithm::addCurrentGenToTrace() {
    if(gatrace_ != nullptr && parameters_.trace) {
      gatrace_->addGeneration(simulation_);
      for(auto ptr : population_.curr().sortptr) { gatrace_->addChromosome(*ptr); }
    }
  }

  GeneticAlgorithm::GeneticAlgorithm(const Parameters& parameters,
    const simulation::Simulation& simulation,
    GeneticAlgorithmTrace* gatrace): parameters_(parameters), simulation_(simulation), gatrace_(gatrace),
                                     random_(parameters.seed),
                                     generation_(0) {
    population_.curr().chr.reserve(parameters.numChromosome);
    population_.curr().sortptr.reserve(parameters.numChromosome);
    population_.prev().chr.reserve(parameters.numChromosome);
    population_.curr().sortptr.reserve(parameters.numChromosome);
  }

  void
  GeneticAlgorithm::execute(std::optional<size_t> geninc) {
    if(this->isDone()) { return; }

    size_t genlast{ geninc.has_value()
                      ? generation_ + geninc.value()
                      : parameters_.maxGeneration };
    if(generation_ == 0) {
      generation_++;
      for(size_t ichr{ 0 }; ichr != parameters_.numChromosome; ichr++) {
        population_.curr().chr.emplace_back();
        population_.curr().sortptr.push_back(&population_.curr().chr[ichr]);
        population_.prev().chr.emplace_back();
        population_.prev().sortptr.push_back(&population_.prev().chr[ichr]);

        auto& chr{ population_.curr().chr[ichr] };
        chr.randomize(random_);
        //score_.push_back({ chr.evaluate(simulation_), &chr });
        chr.evaluate(simulation_, parameters_.trace);
      }
      sortScoreAndUpdateBest();
      addCurrentGenToTrace();
    }
    for(; generation_ != genlast && !this->isDone(); generation_++) {
      population_.flip();
      // Copy elites
      for(size_t ichr{ 0 }; ichr != parameters_.numElite; ichr++) {
        //population_.curr().chr[ichr] = *score_[ichr].chrptr;
        population_.curr().chr[ichr] = *population_.prev().sortptr[ichr];
        //score_[ichr].chrptr = &population_.curr()[ichr];
      }
      // Selection and crossover
      assert(parameters_.numElite % 2 == 0);
      assert(parameters_.numChromosome % 2 == 0);
      for(size_t ichr = parameters_.numElite; ichr < parameters_.numChromosome;
          ichr += 2) {
        Chromosome& c1{ population_.curr().chr[ichr] };
        Chromosome& c2{ population_.curr().chr[ichr + 1] };
        const Chromosome* p[2]{ nullptr };
        while(p[0] == p[1]) {
          for(size_t ip{ 0 }; ip != 2; ip++) {
            size_t idxscr{ Util::tournamentSelection(
              parameters_.selectionK,
              parameters_.numChromosome,
              random_) };
            p[ip] = population_.prev().sortptr[idxscr];
          }
        }
        Chromosome::crossover(random_, *p[0], *p[1], c1, c2);
      }
      // Mutation and evaluation for non-elites
      for(size_t ichr{ parameters_.numElite }; ichr != parameters_.numChromosome;
          ++ichr) {
        Chromosome& chr{ population_.curr().chr[ichr] };
        chr.mutate(random_, parameters_.mutationP);
        chr.evaluate(simulation_, parameters_.trace);
      }
      // Sort scores and check for best chrmosome
      sortScoreAndUpdateBest();
      addCurrentGenToTrace();
    }
  }

  void
  GeneticAlgorithm::flushSimulationWithBest() {
    if(!simulation_.isDone()) {
      SimulationPlayer sp{ simulation_, best };
      while(!sp.simulation().isDone()) {
        sp.step();
        scQueue_.push(sp.simulation().currentSC());
      }
      simulation_ = sp.simulation();
    }
  }

  std::optional<simulation::ShipControl>
  GeneticAlgorithm::getSc() {
    if(scQueue_.empty()) { stepSimultationWithBest(); }
    if(scQueue_.empty()) { return std::nullopt; }
    auto ret{ scQueue_.front() };
    scQueue_.pop();
    return ret;
  }
} // geneticalgorithm