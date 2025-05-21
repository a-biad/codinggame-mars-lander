#pragma once

#include "GeneticAlgorithm.hpp"

#include <algorithm>
#include <iostream>
#include <random>
#include <sstream>
#include <vector>

#include <raylib.h>

#include <glm/glm.hpp>

#include "utildraw.hpp"

#include <iomanip>
#include <memory>

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

using Pt2f = glm::vec<2, float>;
using Pt2i = glm::vec<2, int>;

float dist(const Pt2f& vec) { return sqrt(vec.x * vec.x + vec.y * vec.y); }

float dist(const Pt2f& a, const Pt2f& b) { return dist(a - b); }

static constexpr float G{ -3.711 };
static constexpr Pt2f BOUNDARIES{ 7000.f, 3000.f };

bool ccw(const Pt2f& a, const Pt2f& b, const Pt2f& c)
{
  return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
}

/**
 * Check if a collision occured
 * p1: Position before a single turn
 * p2: Position after a single turn
 * return: Pointer to the "left" point of the land that was collided with, or nullptr if
 * no collision
 */
const Pt2f* collision(const Pt2f& p1, const Pt2f p2, const vector<Pt2f>& land)
{
  for(size_t i{ 1 }; i < land.size(); i++) {
    const Pt2f& l1{ land[i - 1] };
    const Pt2f& l2{ land[i] };
    if(ccw(p1, l1, l2) != ccw(p2, l1, l2) && ccw(p1, p2, l1) != ccw(p1, p2, l2))
      return &l1;
  }
  return nullptr;
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
  ret.intersection = { -1.f, -1.f };
  ret.idxL = land.size();

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
        ret.idxL = i;
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
  return {};
}

struct Input
{
  vector<Pt2f> land;
  size_t idxLTarget;
  Pt2f targetLeft() const { return land[idxLTarget]; };
  Pt2f targetRight() const { return land[idxLTarget + 1]; };
  Pt2f position;
  int fuel;
  int angle;
  int power;
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

namespace GA
{
  enum class MutationStrategy
  {
    GAUSSIAN_DISTRIBUTION
  };

  enum class CrossoverStrategy
  {
    LINE_RECOMBINATION
  };

  struct Parameters
  {
    int seed{ 1234 };
    size_t numAlleles{ 300 };
    float weightDist{ 1 / 10.f };
    float weightSpeedX{ 1 / 50.f };
    float weightSpeedY{ 1 / 50.f };
    float weightAngle{ 1 / 90.f };
    size_t numPhenotypes{ 100 };
    int selectionK{ 10 };
    CrossoverStrategy crossoverStrategy{ CrossoverStrategy::LINE_RECOMBINATION };
    float crossoverP{ 0.25 };
    MutationStrategy mutationStrategy{ MutationStrategy::GAUSSIAN_DISTRIBUTION };
    double mutationB{ 0.01 };
    float mutationSigma{ 15.f }; // 2 sigma = 15 deg rotation (95%)
    size_t numElite{ 10 };
    size_t genMax{ 1000 };
  };

  struct ShipControl
  {
    int angle;
    int power;
    Pt2f acceleration()
    {
      return { -power * sin(angle / 180.f * PI), G + power * cos(angle / 180.f * PI) };
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

    vector<ShipControl> sc;
    Result res;

    Chromosome(int gen, int id, const Parameters& param)
        : m{ { nullptr } }, sc{ param.numAlleles }
    {}
    Chromosome() : m{ { nullptr } } {}

    void simulate(const Input& input, const Parameters& param)
    {
      if(res.evaluated)
        return;
      int t{ 0 };
      Pt2f position{ input.position };
      Pt2f velocity{ 0, 0 };
      int fuel{ input.fuel };
      ShipControl target{ sc[0] };
      ShipControl actual{ input.angle, input.power };
      auto step{ sc.begin() };
      const auto stepend{ sc.end() };

      // Stop conditions
      CollisionData collisionData;
      bool hasLeftBoundaries{ false };
      bool hasExhaustedFuel{ false };

      while(!collisionData.hasCollided && !hasLeftBoundaries && !hasExhaustedFuel) {
        // Apply changes from phenotype
        t++;
        if(step != stepend) {
          target = *(step++);
        }
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
        res.m.trajectory.push_back({ position, actual });
        // Check for termination conditions
        collisionData = checkCollision(previousPosistion, position, input.land);
        if(position.x < 0 || position.x > BOUNDARIES.x || position.y < 0
           || position.y > BOUNDARIES.y) {
          hasLeftBoundaries = true;
        }
        if(fuel <= 0) {
          hasExhaustedFuel = true;
        }
      } // Simulation end
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
      scr.speed = Pt2f{ 1.f, 1.f } - Pt2f{ abs(det.speed.x), abs(det.speed.y) };
      scr.angle = abs(det.angle) / 90.f;
      scr.duration = 0.f;
      scr.fuelUsed = 0.f;
      scr.fuelLeft = 0.f;
      res.sumToScore();
      res.hasLanded = collisionData.idxL == input.idxLTarget && abs(velocity.x) < 20.f
                      && abs(velocity.y) < 40.f && abs(actual.angle) <= 15;

      res.evaluated = true;
    }

    Chromosome mutate(const Parameters& param, Random& rand)
    {
      Chromosome ret;
      switch(param.mutationStrategy) {
      case MutationStrategy::GAUSSIAN_DISTRIBUTION:
        ret.m = this->m;
        ret.sc.resize(this->sc.size());
        std::normal_distribution dAngle{ 0.f, param.mutationSigma };
        auto normRand{ [&] { return int(dAngle(rand.gen)); } };
        for(size_t idx{ 0 }; idx < ret.sc.size(); idx++) {
          do {
            ret.sc[idx].angle = this->sc[idx].angle + normRand();
          } while(ret.sc[idx].angle < -90 && ret.sc[idx].angle > 90);
          ret.sc[idx].power
            = walkAlgorithm(param.mutationB, rand, ret.sc[idx].power, 0, 4);
        }
        ret.res.evaluated = false;
      } // switch
      return ret;
    } // mutate()

    friend std::ostream& operator<<(std::ostream& os, const Chromosome& chr)
    {
      os << chr.res.m.details.duration << "\n";
      for(auto& sc : chr.sc) {
        os << sc.angle << ", ";
      }
      os << "\n";
      for(auto& sc : chr.sc) {
        os << sc.power << ", ";
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
    switch(param.crossoverStrategy) {
    case CrossoverStrategy::LINE_RECOMBINATION:
      static std::uniform_real_distribution linParam{ -param.crossoverP,
                                                      1 + param.crossoverP };
      static auto bound{ [](const ShipControl& sc) {
        return ShipControl{ max(-90, min(sc.angle, 90)), max(0, min(sc.power, 4)) };
      } };

      float alph{ linParam(rand.gen) };
      float beta{ linParam(rand.gen) };

      if(c1 != nullptr) {
        for(size_t idx{ 0 }; idx < p1.sc.size(); idx++) {
          c1->sc[idx]
            = bound({ int(alph * p1.sc[idx].angle + (1 - alph) * p2.sc[idx].angle),
                      int(alph * p1.sc[idx].power + (1 - alph) * p2.sc[idx].power) });
        }
        c1->m.parent[0] = &p1;
        c1->m.parent[1] = &p2;
      }
      if(c2 != nullptr) {
        for(size_t idx{ 0 }; idx < p1.sc.size(); idx++) {
          c2->sc[idx]
            = bound({ int(beta * p2.sc[idx].angle + (1 - beta) * p1.sc[idx].angle),
                      int(beta * p2.sc[idx].power + (1 - beta) * p1.sc[idx].power) });
        }
        c2->m.parent[0] = &p2;
        c2->m.parent[1] = &p1;
      }
      break; // LINE_RECOMBINATION
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
    std::shared_ptr<const Parameters> params;
    std::shared_ptr<Random> rand;

  public:
    Population chr;
    int num;

    Generation(const Generation& g)
        : chr{ g.chr }, params{ g.params }, rand{ g.rand }, num{ g.num }
    {}

    Generation(int num,
               const std::shared_ptr<Parameters>& params,
               std::shared_ptr<Random>& rand)
        : num{ num }, params{ params },
          chr{ params->numPhenotypes, Chromosome(num, -1, *params) }, rand(rand)
    {}

    void initialize()
    {
      for(auto& c : chr) {
        static std::uniform_int_distribution randangle{ -180, 180 };
        static std::uniform_int_distribution randpower(0, 4);
        for(auto& sc : c.sc) {
          sc.angle = randangle(rand->gen);
          sc.power = randpower(rand->gen);
        }
      }
    }

    void evaluate(const Input& input)
    {
      for(auto& c : chr) {
        c.simulate(input, *params);
      }
      sort(chr.begin(), chr.end(), [](const Chromosome& a, const Chromosome& b) {
        return a.res.score > b.res.score;
      });
    }

    void mutate()
    {
      for_each(chr.begin() + params->numElite, chr.end(), [&](Chromosome& c) {
        c = c.mutate(*params, *rand);
      });
    }
  }; // class Generation

  class GeneticAlgorithm
  {
  public:
    std::shared_ptr<Parameters> param;
    Input input;
    std::shared_ptr<Random> rand;
    vector<Generation> generations;
    vector<Chromosome> best;

    GeneticAlgorithm(const Input& input, const Parameters& param = Parameters())
        : input(input), param(std::make_shared<Parameters>(param)),
          rand(std::make_shared<Random>())
    {
      rand->setSeed(param.seed);
      generations.reserve(param.genMax);
      for(int i{ 0 }; i < param.genMax; i++) {
        generations.push_back(Generation{ i, this->param, this->rand });
      }
    }

    void select_and_crossover(const Generation& prev,
                              Population::iterator destbeg,
                              Population::iterator destend)
    {
      auto c{ destbeg };
      if(distance(destbeg, destend) % 2 != 0) {
        auto parent1{ tournamentSelection(
          *param, *rand, prev.chr.cbegin(), prev.chr.cend()) };
        auto parent2{ tournamentSelection(
          *param, *rand, prev.chr.cbegin(), prev.chr.cend()) };
        crossover(*param, *rand, *parent1, *parent2, &(*c), nullptr);
        ++c;
      }
      for(; c < destend; c += 2) {
        auto parent1{ tournamentSelection(
          *param, *rand, prev.chr.cbegin(), prev.chr.cend()) };
        auto parent2{ tournamentSelection(
          *param, *rand, prev.chr.cbegin(), prev.chr.cend()) };
        crossover(*param, *rand, *parent1, *parent2, &(*c), &*(c + 1));
      }
    }

    void execute()
    {
      size_t gen{ 0 };

      // Generate random chromosomes
      generations[gen].initialize();
      generations[gen].evaluate(input);

      best.push_back(generations[gen].chr[0]);
      gen++;
      for(; gen != param->genMax; gen++) {
        auto& genprev = generations[gen - 1];
        auto& gencurr = generations[gen];

        // Copy elites and crossover to current generation
        copy(genprev.chr.begin(),
             genprev.chr.begin() + param->numElite,
             gencurr.chr.begin());
        select_and_crossover(
          genprev, gencurr.chr.begin() + param->numElite, gencurr.chr.end());
        gencurr.mutate();
        gencurr.evaluate(input);
        if(gencurr.chr[0].res.score > best[best.size() - 1].res.score) {
          best.push_back(gencurr.chr[0]);
          cerr << "Gen: " << gen << " score: " << gencurr.chr[0].res.score << "\n";
        }
        if(gencurr.chr[0].res.hasLanded) {
          break;
        }
      }

      // generations.resize(gen);
      generations.erase(generations.begin() + gen, generations.end());
      auto& resultChromosome{ generations[gen - 1].chr[0] };

      if(resultChromosome.res.hasLanded) {
        cerr << "-- Succesful landing\n";
        cerr << resultChromosome;
      }
      else {
        cerr << "-- Maximum generation reached\n";
      }
      cerr << "Gen: " << gen - 1 << " score: " << resultChromosome.res.score << "\n";
    }
  };

} // namespace Genetic Algorithm

namespace GAVisualizer
{
  using namespace GA;

  class Visualiser
  {
  public:
    size_t gen;
    size_t pheno;
    GeneticAlgorithm* ga;
    static constexpr float SCALE{ 1000.f / 7000.f };

    bool drawGeneration{ true };
    bool drawParents{ false };

    Visualiser(GeneticAlgorithm* ga) : ga(ga)
    {
      gen = 0;
      pheno = 0;
      SetTraceLogLevel(LOG_WARNING);
      InitWindow(1366.f, 700.f,
                 "Mars lander"); // Initialize window and OpenGL context
    }

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
        if(pheno != ga->param->numPhenotypes - 1) {
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
            t->p.x * SCALE, t->p.y * SCALE, t->sc.angle, t->sc.power, color, 6.f);
        else
          drawRocketSimple(
            t->p.x * SCALE, t->p.y * SCALE, t->sc.angle, t->sc.power, color, 6.f);
      }
    }

    void drawChromosome(const Chromosome& chr, const Color& color, int x, int y)
    {
      for(size_t a{ 0 }; a < chr.sc.size(); a++) {
        auto& sc{ chr.sc[a] };
        Color drawingColor = a <= chr.res.m.details.duration ? color : BLACK;
        DrawText(to_string(sc.power).c_str(), x + a * 20, y, 15, drawingColor);
        DrawText(to_string(sc.angle).c_str(), x + a * 20, y + 20, 15, drawingColor);
      }
    }

    void render()
    {
      BeginDrawing();
      ClearBackground(GRAY);

      // Draw land
      for(auto it{ ga->input.land.begin() + 1 }; it != ga->input.land.end(); it++) {
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
          if(idx < ga->param->numElite) {
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