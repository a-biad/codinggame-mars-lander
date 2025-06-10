#pragma once

#define CODING_GAME 1

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <vector>

#if !CODING_GAME
#include "utildraw.hpp"
#include <raylib.h>
#endif

static constexpr float PIF{
  3.1415926535897932384626433832795028841971693993751058209749445923078164062f
};

using std::abs;
using std::cerr;
using std::cout;
using std::max;
using std::min;
using std::ostream;
using std::to_string;
using std::vector;

struct Random
{
  Random() : rd{}, gen{ rd() } {}
  std::random_device rd;
  std::mt19937 gen;
  void setSeed(int seed) { gen.seed(seed); };
};

// Random walk integer
int walkAlgorithm(double b, Random& rand, int v, int vmin = -10000, int vmax = 10000)
{
  b = std::min(b, 0.999);

  std::bernoulli_distribution flip0{ b };
  // Returns true with probability b
  auto flip{ [&rand, &flip0]() { return flip0(rand.gen); } };

  static std::bernoulli_distribution n0{ 0.5 };
  static const int n1[]{ -1, 1 };
  static auto n{
    [&]() { return n1[n0(rand.gen)]; } // Return -1 or 1
  };

  static auto walk{ [&]() {
    do {
      int nr{ n() };
      // Check if new value is withing bounds
      if(v + nr <= vmax && v + nr >= vmin)
        v = v + nr;
      else if(v - nr <= vmax && v - nr >= vmin)
        v = v - nr;
    } while(flip());
    return v;
  } };

  return walk();
}

struct Pt2f
{
  float x;
  float y;
};
inline Pt2f operator+(const Pt2f& a, const Pt2f& b) { return { a.x + b.x, a.y + b.y }; }
inline Pt2f operator-(const Pt2f& a, const Pt2f& b) { return { a.x - b.x, a.y - b.y }; }
inline Pt2f operator-(const Pt2f& a) { return { -a.x, -a.y }; }
inline Pt2f operator*(const Pt2f& a, float b) { return { a.x * b, a.y * b }; }
inline Pt2f operator*(float a, const Pt2f& b) { return b * a; }
inline Pt2f operator/(const Pt2f& a, float b) { return { a.x / b, a.y / b }; }
inline Pt2f& operator+=(Pt2f& a, const Pt2f& b)
{
  a.x += b.x;
  a.y += b.y;
  return a;
};

float dist(const Pt2f& vec) { return sqrt(vec.x * vec.x + vec.y * vec.y); }

float dist(const Pt2f& a, const Pt2f& b) { return dist(a - b); }

static constexpr float G{ -3.711 };
static constexpr Pt2f BOUNDARIES{ 7000.f, 3000.f };

bool ccw(const Pt2f& a, const Pt2f& b, const Pt2f& c)
{
  return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
}

// https://stackoverflow.com/a/565282
struct CollisionData
{
  bool hasCollided{ false };
  Pt2f intersection;
  size_t idxL;
};
CollisionData checkCollision(const Pt2f& p1, const Pt2f p2, const vector<Pt2f>& land)
{
  auto cross{ [](const Pt2f& a, const Pt2f& b) { return a.x * b.y - a.y * b.x; } };
  CollisionData ret;
  ret.hasCollided = false;
  ret.intersection = BOUNDARIES;
  ret.idxL = land.size() - 1;

  Pt2f r{ p2 - p1 };
  for(size_t i{ 1 }; i < land.size(); i++) {
    const Pt2f& l1{ land[i - 1] };
    const Pt2f& l2{ land[i] };
    Pt2f s{ l2 - l1 };
    float denum{ cross(r, s) };
    float uNum{ cross((l1 - p1), r) };
    float tNum{ cross((l1 - p1), s) };
    if(denum != 0) {
      float t, u;
      if(0 <= (t = tNum / denum) && t <= 1 //
         && 0 <= (u = uNum / denum) && u <= 1) {
        ret.hasCollided = true;
        ret.intersection = p1 + t * r;
        ret.idxL = i - 1;
        return ret;
      }
      else {
        // Not parallel and no intersection
      }
    }
    else {
      // parallel
    }
  }
  return ret;
}

class ShipControl
{
public:
  int angle;
  int power;

  ShipControl roundAngle() const { return { int(roundf(angle / 15.f) * 15), power }; }

  Pt2f acceleration() const
  {
    return { float(-power * sin(angle / 180.f * PIF)),
             float(G + power * cos(angle / 180.f * PIF)) };
  }

  friend ostream& operator<<(ostream& os, const ShipControl& sc)
  {
    os << sc.angle << " " << sc.power;
    return os;
  }
};

class Input
{
public:
  int startTime{ 0 };
  vector<Pt2f> land;
  size_t idxLTarget;
  Pt2f targetLeft() const { return land[idxLTarget]; };
  Pt2f targetRight() const { return land[idxLTarget + 1]; };
  Pt2f position;
  Pt2f velocity{ 0.f, 0.f };
  int fuel;
  ShipControl sc;
};

float
projectedDistance(const CollisionData& c, const vector<Pt2f>& land, size_t idxLTarget)
{
  float ret{ 0.f };
  if(c.idxL == idxLTarget)
    return 0.f;

  int dir; // Direction of the target
  size_t idxLi;
  size_t idxLiend;
  if(c.idxL < idxLTarget) {
    dir = 1;
    idxLi = c.idxL + 1;
    idxLiend = idxLTarget;
    ret += dist(c.intersection, land[c.idxL + 1]);
  }
  else {
    dir = -1;
    idxLi = c.idxL - 1;
    idxLiend = idxLTarget;
    ret += dist(c.intersection, land[c.idxL]);
  }
  for(; idxLi != idxLiend; idxLi += dir) {
    ret += dist(land[idxLi], land[idxLi + 1]);
  }

  return ret;
}

void projectedDistanceTest(const Input& input)
{
  bool error{ false };
  float distPrev{ 0.f };
  for(int x{ -600 }; x < BOUNDARIES.x + 600; x += 100) {
    Pt2f p1{ float(x), 3000.f };
    Pt2f p2{ float(x), -1.f };
    auto cd{ checkCollision(p1, p2, input.land) };
    if(true || cd.hasCollided) {
      float dist{ projectedDistance(cd, input.land, input.idxLTarget) };
      cerr << std::setw(5) << x << "\t"                      //
           << std::setw(5) << cd.idxL << "\t"                //
           << std::setw(5) << int(cd.intersection.x) << "\t" //
           << std::setw(5) << int(cd.intersection.y) << "\t" //
           << std::setw(5) << int(dist) << "\n";
      distPrev = dist;
    }
  }
  if(error) {
    throw std::runtime_error("Error in checking collision");
  }
}

namespace GA
{
  enum class MutationStrategy
  {
    GAUSSIAN_DISTRIBUTION,
    BITFLIP,
  };

  enum class CrossoverStrategy
  {
    LINE_RECOMBINATION,
    ONE_POINT
  };

  struct Parameters
  {
    bool trace{ true };
    int seed{ 1234 };
    size_t numAlleles{ 128 };
    float weightDist{ 1.f };
    float weightSpeedX{ 1 / 50.f };
    float weightSpeedY{ 1 / 50.f };
    float weightAngle{ 1 / 90.f };
    size_t numPhenotypes{ 128 };
    int selectionK{ 10 };
    CrossoverStrategy crossoverStrategy{ CrossoverStrategy::ONE_POINT };
    bool crossoverPointBeforeLastChromosome{ false };
    float crossoverP{ 0.25 };
    MutationStrategy mutationStrategy{ MutationStrategy::BITFLIP };
    // double mutationB{ 0.01 };
    // float mutationSigma{ 15.f }; // 2 sigma = 15 deg rotation (95%)
    float mutationP{ 0.02f };
    size_t numElite{ 16 };
    size_t genMax{ 1024 };
  };

  struct Allele
  {
    bool eAng;
    bool dAng;
    bool dPow;

    ShipControl apply(const ShipControl& actual, int t) const
    {
      if(t == 0) {
        return actual.roundAngle();
      }
      ShipControl target;
      if(eAng) {
        target.angle = !dAng ? max(actual.angle - 15, -90) //
                             : min(actual.angle + 15, +90);
      }
      else {
        target.angle = actual.angle;
      }
      target.power = !dPow ? max(actual.power - 1, 0) //
                           : min(actual.power + 1, 4);
      return target;
    }
  };

  struct _TrajectoryElem
  {
    Pt2f p;
    ShipControl sc;
  };

  using Trajectory = vector<_TrajectoryElem>;

  struct EndData
  {
    Pt2f dist;
    float projectedDist;
    Pt2f speed;
    float angle;
    float duration;
    float fuelUsed;
    float fuelLeft;

    friend ostream& operator<<(ostream& os, const EndData& ed)
    {
      os << "Dist: " << int(ed.projectedDist) << " Speed: " << int(ed.speed.x) << " "
         << int(ed.speed.y) << " Angle: " << ed.angle << "Time " << int(ed.duration);
      return os;
    }
  };

  struct Result
  {
    bool evaluated{ false };
    size_t lastAll;
    float score;
    bool hasLanded;
    struct Meta
    {
      Trajectory trajectory;
      EndData details;
      EndData scoreDetails;
      CollisionData collisionData;
    } m;

    void sumToScore()
    {
      auto& d{ m.scoreDetails };
      score = d.dist.x + d.dist.y + d.projectedDist + d.speed.x + d.speed.y + d.angle
              + d.duration + d.fuelUsed + d.fuelLeft;
    };
  };

  class Generation;

  class Chromosome
  {
  public:
    struct Meta
    {
      const Chromosome* parent[2]{ nullptr };
    } m;

    vector<Allele> al;
    Result res;

    Chromosome(int gen, int id, const Parameters& param)
        : m{ { nullptr } }, al{ param.numAlleles }
    {}
    Chromosome() : m{ { nullptr } } {}

    void generateRandom(Random& rand,
                        vector<Allele>::iterator albeg,
                        vector<Allele>::iterator alend)
    {
      static std::bernoulli_distribution b{ 0.5 };
      for(auto& al{ albeg }; al != alend; al++) {
        al->eAng = b(rand.gen);
        al->dAng = b(rand.gen);
        al->dPow = b(rand.gen);
      }
    }

    void simulate(const Input& input, const Parameters& param)
    {
      if(res.evaluated)
        return;
      int t{ input.startTime };
      Pt2f position{ input.position };
      Pt2f velocity{ input.velocity };
      int fuel{ input.fuel };
      ShipControl actual{ input.sc };
      // First step: round angle to closest multiple of 15
      ShipControl target;
      auto step{ al.begin() };
      const auto stepend{ al.end() };

      // Stop conditions
      CollisionData collisionData;
      bool hasLeftBoundaries{ false };
      bool hasExhaustedFuel{ false };

      while(!collisionData.hasCollided && !hasLeftBoundaries && !hasExhaustedFuel) {
        // Apply changes from phenotype
        target = step->apply(actual, t);
        if(t != 0)
          step++;
        // Power control
        actual.power += min(max(-1, target.power - actual.power), 1);
        actual.power = min(max(0, actual.power), 4);
        // Angle control
        actual.angle += min(max(-15, target.angle - actual.angle), 15);
        actual.angle = max(min(90, actual.angle), -90);
        // Calculate next step
        Pt2f previousPosistion{ position };
        fuel -= actual.power;
        velocity += actual.acceleration();
        position += velocity;
        // Save trajectory
        if(param.trace) {
          res.m.trajectory.push_back({ position, actual });
        }
        // Check for termination conditions
        collisionData = checkCollision(previousPosistion, position, input.land);
        if(position.x < 0 || position.x > BOUNDARIES.x || position.y < 0
           || position.y > BOUNDARIES.y) {
          hasLeftBoundaries = true;
        }
        if(fuel <= 0) {
          hasExhaustedFuel = true;
        }
        t++;
      } // Simulation end

      if(!collisionData.hasCollided) {
        collisionData.idxL = input.land.size() - 1;
        collisionData.intersection = { 3000, 7000 };
      }
      auto& det{ res.m.details };
      det.dist = (input.targetLeft() + input.targetRight()) / 2.f - position;
      det.projectedDist = projectedDistance(collisionData, input.land, input.idxLTarget);
      det.speed = velocity;
      det.angle = actual.angle;
      det.duration = t;
      det.fuelUsed = input.fuel - fuel;
      det.fuelLeft = fuel;
      res.m.collisionData = collisionData;

      // Score evaluation
      auto& scr{ res.m.scoreDetails };
      scr.dist = { 0.f, 0.f };
      scr.projectedDist = 1.f - abs(det.projectedDist) * param.weightDist;
      scr.speed = -Pt2f{ abs(det.speed.x), abs(det.speed.y) };
      scr.angle = -abs(det.angle) / 90.f;
      scr.duration = 0.f;
      scr.fuelUsed = 0.f;
      scr.fuelLeft = 0.f;
      res.sumToScore();
      res.hasLanded = collisionData.idxL == input.idxLTarget && abs(velocity.x) < 20.f
                      && abs(velocity.y) < 40.f && abs(actual.angle) <= 15;
      res.lastAll = step - this->al.begin();

      res.evaluated = true;
    }

    Chromosome mutate(const Parameters& param, Random& rand)
    {
      Chromosome ret;
      ret.m = this->m;
      ret.al.resize(this->al.size());
      ret.res.evaluated = false;
      switch(param.mutationStrategy) {
      case MutationStrategy::GAUSSIAN_DISTRIBUTION:
        throw std::runtime_error("Gaussian Distribution unsupported");
      // case MutationStrategy::GAUSSIAN_DISTRIBUTION:
      //   std::normal_distribution dAngle{ 0.f, param.mutationSigma };
      //   auto normRand{ [&] { return int(dAngle(rand.gen)); } };
      //   for(size_t idx{ 0 }; idx < ret.al.size(); idx++) {
      //     do {
      //       ret.al[idx].angle = this->al[idx].angle + normRand();
      //     } while(ret.al[idx].angle < -90 && ret.al[idx].angle > 90);
      //     ret.al[idx].power
      //       = walkAlgorithm(param.mutationB, rand, ret.al[idx].power, 0, 4);
      //   }
      //   break;
      case MutationStrategy::BITFLIP:
        std::bernoulli_distribution flip{ param.mutationP };
        for(size_t idx{ 0 }; idx < this->al.size(); idx++) {
          ret.al[idx].eAng = this->al[idx].eAng ^ flip(rand.gen);
          ret.al[idx].dAng = this->al[idx].dAng ^ flip(rand.gen);
          ret.al[idx].dPow = this->al[idx].dPow ^ flip(rand.gen);
        }
        break;
      } // switch
      return ret;
    } // mutate()

    friend std::ostream& operator<<(std::ostream& os, const Chromosome& chr)
    {
      os << chr.res.m.details.duration << "\n";
      for(auto& sc : chr.al) {
        os << sc.eAng << ", ";
      }
      os << "\n";
      for(auto& sc : chr.al) {
        os << sc.dAng << ", ";
      }
      os << "\n";
      for(auto& sc : chr.al) {
        os << sc.dPow << ", ";
      }
      return os;
    } // operator<<()
  }; // class Chromosome

  void crossover(Parameters& param,
                 Random& rand,
                 const Chromosome& p1,
                 const Chromosome& p2,
                 Chromosome* c1,
                 Chromosome* c2)
  {
    if(c1 == nullptr && c2 == nullptr) {
      return;
    }
    if(c1 != nullptr) {
      c1->m.parent[0] = &p1;
      c1->m.parent[1] = &p2;
    }
    if(c2 != nullptr) {
      c2->m.parent[0] = &p2;
      c2->m.parent[1] = &p1;
    }
    switch(param.crossoverStrategy) {
    case CrossoverStrategy::LINE_RECOMBINATION:
      throw std::runtime_error("Line recombination unsupported");
      break;
    // case CrossoverStrategy::LINE_RECOMBINATION:
    //   static std::uniform_real_distribution linParam{ -param.crossoverP,
    //                                                   1 + param.crossoverP };
    //   static auto bound{ [](const ShipControl& sc) {
    //     return ShipControl{ max(-90, min(sc.angle, 90)), max(0, min(sc.power, 4)) };
    //   } };
    //
    //   float alph{ linParam(rand.gen) };
    //   float beta{ linParam(rand.gen) };
    //
    //   if(c1 != nullptr) {
    //     for(size_t idx{ 0 }; idx < p1.al.size(); idx++) {
    //       c1->al[idx]
    //         = bound({ int(alph * p1.al[idx].angle + (1 - alph) * p2.al[idx].angle),
    //                   int(alph * p1.al[idx].power + (1 - alph) * p2.al[idx].power) });
    //     }
    //   }
    //   if(c2 != nullptr) {
    //     for(size_t idx{ 0 }; idx < p1.al.size(); idx++) {
    //       c2->al[idx]
    //         = bound({ int(beta * p2.al[idx].angle + (1 - beta) * p1.al[idx].angle),
    //                   int(beta * p2.al[idx].power + (1 - beta) * p1.al[idx].power) });
    //     }
    //   }
    //   break; // LINE_RECOMBINATION
    case CrossoverStrategy::ONE_POINT:
      size_t ptMax{ param.numAlleles - 2 };
      if(param.crossoverPointBeforeLastChromosome) {
        ptMax = min(ptMax, min(p1.res.lastAll, p2.res.lastAll));
      }
      std::uniform_int_distribution<size_t> pt0{ 1, ptMax };
      size_t pt = pt0(rand.gen);
      if(c1 != nullptr) {
        std::copy_n(p1.al.begin(), pt, c1->al.begin());
        std::copy(p2.al.begin() + pt, p2.al.end(), c1->al.begin() + pt);
      }
      if(c2 != nullptr) {
        std::copy_n(p2.al.begin(), pt, c2->al.begin());
        std::copy(p1.al.begin() + pt, p1.al.end(), c2->al.begin() + pt);
      }
      break;

    } // switch
  } // crossover()

  using Population = vector<Chromosome>;

  Population::const_iterator tournamentSelection(const Parameters& param,
                                                 Random& rand,
                                                 const Population::const_iterator beg,
                                                 const Population::const_iterator end)
  {
    std::uniform_int_distribution r{ 0L, (long)(end - beg) - 1 };
    int k{ param.selectionK };
    if(k <= 0)
      return beg;
    Population::const_iterator best{ end };
    for(int i{ 0 }; i < k; i++) {
      Population::const_iterator idx{ beg + r(rand.gen) };
      if(best == end || idx->res.score > best->res.score) {
        best = idx;
      }
    }
    return best;
  } // tournamentSelection()

  class Generation
  {
    std::shared_ptr<Random> rand;

  public:
    Population chr;
    int num;

    Generation() {}

    Generation(const Generation& g) : chr{ g.chr }, rand{ g.rand }, num{ g.num } {}

    Generation(int num, const Parameters& params, std::shared_ptr<Random>& rand)
        : num{ num }, chr{ params.numPhenotypes, Chromosome(num, -1, params) }, rand(rand)
    {}

    void initialize()
    {
      for(auto& c : chr) {
        c.generateRandom(*rand, c.al.begin(), c.al.end());
        // static std::uniform_int_distribution randangle{ -180, 180 };
        // static std::uniform_int_distribution randpower(0, 4);
        static std::bernoulli_distribution b{ 0.5 };
        for(auto& sc : c.al) {
          sc.eAng = b(rand->gen);
          sc.dAng = b(rand->gen);
          sc.dPow = b(rand->gen);
        }
      }
      // !! First hint for input 4
      // std::fill(chr[0].al.begin(), chr[0].al.begin() + 7, (Allele){ true, true, true });
      // std::fill(chr[0].al.begin() + 7, chr[0].al.end(), (Allele){ false, true, true });
    }

    void evaluate(const Input& input, const Parameters& params)
    {
      for(auto& c : chr) {
        c.simulate(input, params);
      }
      sort(chr.begin(), chr.end(), [](const Chromosome& a, const Chromosome& b) {
        return a.res.score > b.res.score;
      });
    }

    void mutate(const Parameters& params)
    {
      for_each(chr.begin() + params.numElite, chr.end(), [&](Chromosome& c) {
        c = c.mutate(params, *rand);
      });
    }
  }; // class Generation

  class GeneticAlgorithmTrace
  {
  public:
    vector<Generation> generations;
    vector<Chromosome> bests;

    GeneticAlgorithmTrace() {};
  };

  class GeneticAlgorithm
  {
  private:
    Generation gcurr;

  public:
    Parameters param;
    const Input input;
    std::shared_ptr<Random> rand;

    size_t gen{ 0 };
    Input inputCurr;
    Chromosome best;
    bool done{ false };

    std::shared_ptr<GeneticAlgorithmTrace> gatrace;

    GeneticAlgorithm(const Input& input, const Parameters& param = Parameters())
        : input(input), param(param), rand(std::make_shared<Random>()), inputCurr(input)
    {
      rand->setSeed(param.seed);
      if(param.trace) {
        gatrace = std::make_shared<GeneticAlgorithmTrace>();
      }
      best.res.score = -INFINITY;
    }

    void select_and_crossover(const Generation& prev,
                              Population::iterator destbeg,
                              Population::iterator destend)
    {
      auto c{ destbeg };
      if(distance(destbeg, destend) % 2 != 0) {
        auto parent1{ tournamentSelection(
          param, *rand, prev.chr.cbegin(), prev.chr.cend()) };
        auto parent2{ tournamentSelection(
          param, *rand, prev.chr.cbegin(), prev.chr.cend()) };
        crossover(param, *rand, *parent1, *parent2, &(*c), nullptr);
        ++c;
      }
      for(; c < destend; c += 2) {
        auto parent1{ tournamentSelection(
          param, *rand, prev.chr.cbegin(), prev.chr.cend()) };
        auto parent2{ tournamentSelection(
          param, *rand, prev.chr.cbegin(), prev.chr.cend()) };
        crossover(param, *rand, *parent1, *parent2, &(*c), &*(c + 1));
      }
    }

    void execute(int geninc = -1)
    {
      size_t genlast{ geninc == -1 ? param.genMax : min(param.genMax, gen + geninc) };
      for(; gen != genlast && !best.res.hasLanded; gen++) {
        Generation gnext(gen, param, rand);
        if(gen == 0) {
          gnext.initialize();
        }
        else {
          // Copy elites and crossover to current generation
          copy(gcurr.chr.begin(), gcurr.chr.begin() + param.numElite, gnext.chr.begin());
          select_and_crossover(
            gcurr, gnext.chr.begin() + param.numElite, gnext.chr.end());
          gnext.mutate(param);
        }

        gnext.evaluate(inputCurr, param);
        auto& currbest = gnext.chr[0];
        bool isNewBest{ gnext.chr[0].res.score > best.res.score };
        if(isNewBest) {
          best = gnext.chr[0];
          cout << "Gen: " << gen << " score: " << best.res.score << "\n";
        }
        if(param.trace) {
          gatrace->generations.push_back(gnext);
          if(isNewBest)
            gatrace->bests.push_back(best);
        }
        gcurr = std::move(gnext);
      }

      cout << "-- Best chromosome: \n";
      cout << best << "\n";
      if(best.res.hasLanded) {
        done = true;
        cout << "-- Succesful landing\n";
      }
      else if(gen == param.genMax) {
        done = true;
        cout << "-- Maximum generation reached\n";
      }
      else {
        cout << "-- Waiting for next call ( " << param.genMax - gen
             << " generations left )\n";
      }
      cout << "Gen: " << gen - 1 << " score: " << best.res.score << "\n";
    } // execute()

    void changeInput(const Input& inputNext)
    {
      int dt = inputNext.startTime - inputCurr.startTime;
      inputCurr = inputNext;
      if(dt <= 0)
        return;
      for(auto& c : gcurr.chr) {
        c.al.erase(c.al.begin(), c.al.begin() + dt);
        c.al.resize(param.numAlleles);
        c.generateRandom(*rand, c.al.end() - dt, c.al.end());
      }
    }
  }; // class GeneticAlgorithm

} // namespace Genetic Algorithm

#if !CODING_GAME

namespace GAVisualizer
{
  using namespace GA;

  class Visualiser
  {
    const Input* input_;
    const Parameters* parameters_;

  public:
    size_t gen;
    size_t pheno;
    GeneticAlgorithmTrace* ga;
    static constexpr float SCALE{ 1366.f / 7000.f };

    bool drawGeneration{ true };
    bool drawParents{ false };

    Visualiser(GeneticAlgorithmTrace* ga) : ga(ga)
    {
      gen = 0;
      pheno = 0;
      SetTraceLogLevel(LOG_WARNING);
      InitWindow(1366.f, 700.f,
                 "Mars lander"); // Initialize window and OpenGL context
    }

    void setInput(const Input* input) { input_ = input; }
    void setParameters(const Parameters* parameters) { parameters_ = parameters; }

    void printCurrentGenInfo()
    {
      cout << "Gen " << gen << " : ";
      for(auto& p : ga->generations[gen].chr) {
        cout << p.res.score << " ";
      }
      cout << "\n";
    }

    void eventCheck()
    {
      if(IsKeyPressed(KEY_Z)) {
        drawGeneration = !drawGeneration;
      }
      if(IsKeyPressed(KEY_X)) {
        drawParents = !drawParents;
      }

      static const int INCREMENT{ 20 };
      if(IsKeyPressedRepeat(KEY_PAGE_DOWN) || IsKeyPressed(KEY_PAGE_DOWN)) {
        if(gen >= INCREMENT) {
          gen -= INCREMENT;
        }
        else {
          gen = ga->generations.size() - 1;
        }
      }
      if(IsKeyPressedRepeat(KEY_PAGE_UP) || IsKeyPressed(KEY_PAGE_UP)) {
        if(gen <= ga->generations.size() - 1 - INCREMENT) {
          gen += INCREMENT;
        }
        else {
          gen = 0;
        }
      }

      if(IsKeyPressedRepeat(KEY_LEFT) || IsKeyPressed(KEY_LEFT)) {
        if(gen != 0) {
          gen--;
        }
        else {
          gen = ga->generations.size() - 1;
        }
      }
      if(IsKeyPressedRepeat(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT)) {
        if(gen != ga->generations.size() - 1) {
          gen++;
        }
        else {
          gen = 0;
        }
      }
      if(IsKeyPressedRepeat(KEY_DOWN) || IsKeyPressed(KEY_DOWN)) {
        if(pheno != 0) {
          pheno--;
        }
      }
      if(IsKeyPressedRepeat(KEY_UP) || IsKeyPressed(KEY_UP)) {
        if(pheno != ga->generations[0].chr.size() - 1) {
          pheno++;
        }
      }
    }
    void drawTrajectory(Trajectory::const_iterator beg,
                        Trajectory::const_iterator end,
                        const Color& color,
                        bool simple = false)
    {
      for(auto t{ beg }; t != end; t++) {
        if(!simple)
          drawRocket(
            t->p.x * SCALE, t->p.y * SCALE, t->sc.angle, t->sc.power, color, 2.f);
        else
          drawRocketSimple(
            t->p.x * SCALE, t->p.y * SCALE, t->sc.angle, t->sc.power, color, 6.f);
      }
    }

    void drawChromosome(const Chromosome& chr, const Color& color, int x, int y)
    {
      for(size_t a{ 0 }; a < chr.al.size(); a++) {
        auto& sc{ chr.al[a] };
        Color drawingColor = a <= chr.res.m.details.duration ? color : BLACK;
        DrawText(to_string(sc.eAng).c_str(), x + a * 20, y, 15, drawingColor);
        DrawText(to_string(sc.dAng).c_str(), x + a * 20, y + 10, 15, drawingColor);
        DrawText(to_string(sc.dPow).c_str(), x + a * 20, y + 20, 15, drawingColor);
      }
    }

    void render()
    {
      BeginDrawing();
      ClearBackground(GRAY);

      // Draw land
      for(auto it{ input_->land.begin() + 1 }; it != input_->land.end(); it++) {
        auto pit = it - 1;
        DrawLine(pit->x * SCALE, pit->y * SCALE, it->x * SCALE, it->y * SCALE, RED);
      }
      // Current generation
      DrawText(("Gen: " + to_string(gen)).c_str(), 30, 400, 30, RED);

      if(drawGeneration) {
        auto beg{ ga->generations[gen].chr.rbegin() };
        auto end{ ga->generations[gen].chr.rend() };
        for(auto p{ beg }; p != end; p++) {
          Color drawingColor;
          auto idx{ &(*p) - &(ga->generations[gen].chr[0]) };
          if(idx < parameters_->numElite) {
            drawingColor = GREEN;
          }
          else {
            drawingColor = ORANGE;
          }
          // Draw ship trajectory
          drawTrajectory(
            p->res.m.trajectory.begin(), p->res.m.trajectory.end(), drawingColor, true);
        }
      }

      auto& selectedPheno{ ga->generations[gen].chr[pheno] };
      // Draw the parents trajectory and chromosome
      if(drawParents) {
        for(size_t p{ 0 }; p < 2; p++) {
          Color drawingColor{ p == 0 ? BLUE : SKYBLUE };
          if(selectedPheno.m.parent[p] != nullptr) {
            auto& t{ selectedPheno.m.parent[p]->res.m.trajectory };
            drawTrajectory(t.begin(), t.end(), drawingColor);
            drawChromosome(*selectedPheno.m.parent[p], drawingColor, 150, 500 + 50 * p);
          }
        }
      }
      // Draw the selected pheno trajectory, chromosome, and score
      drawTrajectory(selectedPheno.res.m.trajectory.begin(),
                     selectedPheno.res.m.trajectory.end(),
                     PINK);
      std::stringstream tempstream;
      tempstream << std::setprecision(3) << selectedPheno.res.score << " "
                 << selectedPheno.res.m.scoreDetails;

      auto phenoScoreString{ "Pheno: " + to_string(pheno) + " " + tempstream.str() };
      DrawText(phenoScoreString.c_str(), 150, 400, 20, PINK);
      drawChromosome(selectedPheno, PINK, 150, 450);

      EndDrawing();
      WaitTime(1.0 / 24.0);
    }

    void mainLoop()
    {
      while(!WindowShouldClose()) {
        eventCheck();
        render();
      }
    }

    ~Visualiser()
    {
      CloseWindow(); // Close window and unload OpenGL context
    }
  };
}

#endif#if !CODING_GAME
#include "GeneticAlgorithm.hpp"
#include "CmdArgs.hpp"
#endif

using std::cin;

void fetchFromInputStream(Input& input)
{
  int surface_n; // the number of points used to draw the surface of Mars.
  cin >> surface_n;
  cin.ignore();
  input.land.resize(surface_n + 3);

  for(int i = 0; i < surface_n; i++) {
    cin >> input.land[i].x >> input.land[i].y;
    cin.ignore();
    if(i > 0 && input.land[i].x - input.land[i - 1].x >= 1000
       && input.land[i - 1].y == input.land[i].y) {
      input.idxLTarget = i - 1;
    }
  }
  input.land[surface_n] = { BOUNDARIES.x, BOUNDARIES.y };
  input.land[surface_n + 1] = { 0, BOUNDARIES.y };
  input.land[surface_n + 2] = input.land[0];

  int x;
  int y;
  int h_speed; // the horizontal speed (in m/s), can be negative.
  int v_speed; // the vertical speed (in m/s), can be negative.
  int fuel;    // the quantity of remaining fuel in liters.
  int rotate;  // the rotation angle in degrees (-90 to 90).
  int power;   // the thrust power (0 to 4).
  cin >> x >> y >> h_speed >> v_speed >> fuel >> rotate >> power;
  cin.ignore();

  input.position = { static_cast<float>(x), static_cast<float>(y) };
  input.fuel = fuel;
  input.sc.power = power;
  input.sc.angle = rotate;
}

int main(int argc, char** argv)
{
  CmdArgs cmdArgs(argc, argv);
  if(cmdArgs.shouldExit() == true)
    return 0;
  auto options{ cmdArgs.getOptions() };

  Input input;
  fetchFromInputStream(input);

  GA::GeneticAlgorithm ga{ input };
  while(!ga.done) {
    ga.execute(32);
#if !CODING_GAME
    if(options.visualize) {
      GAVisualizer::Visualiser visualiser{ ga.gatrace.get() };
      visualiser.setInput(&input);
      visualiser.setParameters(&ga.param);
      visualiser.mainLoop();
    }
#endif
    auto& al{ ga.best.al[0] };
    input.sc = al.apply(input.sc, input.startTime);
    cout << input.sc << "\n";
    input.velocity += input.sc.acceleration();
    input.position += input.velocity;
    ga.changeInput(input);
  }

  return 0;
}