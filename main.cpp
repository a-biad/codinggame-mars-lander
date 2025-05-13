#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/program_options/detail/parsers.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>

#include <raylib.h>

using namespace std;

struct P
{
  float x;
  float y;
};

const float G{ -3.711 };
const P BOUNDARIES{ 7000, 3000 };

int ccw(const P& a, const P& b, const P& c)
{
  return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
}

/**
 * Check if a collision occured
 * p1: Position before a single turn
 * p2: Position after a single turn
 */
bool collision(P p1, P p2, const vector<P>& land)
{
  for(size_t i{ 1 }; i < land.size(); i++) {
    const P& l1{ land[i - 1] };
    const P& l2{ land[i] };
    if(ccw(p1, l1, l2) != ccw(p2, l1, l2) && ccw(p1, p2, l1) != ccw(p1, p2, l2))
      return true;
  }
  return false;
}

struct InitialData
{
  vector<P> land;
  int target_x1;
  int target_x2;
  int target_y;
  P position;
  int fuel;
  int power;
  int rotate;
};

class MarsLanderGeneticAlgorithm
{
  static const int NUM_ALLELES{ 20 };
  static const int NUM_PHENOTYPES{ 20 };
  static const int NUM_SELECT{ 10 };
  static const size_t NUM_ELITE{ 2 };
  static constexpr float MUTATION_P{ 0.1f };
  static const int SEED{ 4321 };
  // static constexpr float ELITISM_RATE{ 0.2f };
  static const size_t MAX_EPOCH{ 100000 };

  struct ShipControl
  {
    int t;
    int thrust;
    int angle;
  };

  struct Result
  {
    struct ScoreDetails
    {
      float distX;
      float distY;
      float landingSpeedX;
      float landingSpeedY;
      float landingRotation;
      float duration;
    } scoreDetails;

    string scoreString()
    {
      return "Score: " + to_string(score)                                //
             + " x: " + to_string(int(scoreDetails.distX))               //
             + " y: " + to_string(int(scoreDetails.distY))               //
             + " spdX: " + to_string(int(scoreDetails.landingSpeedX))    //
             + " spdY: " + to_string(int(scoreDetails.landingSpeedY))    //
             + " rot: " + to_string(int(scoreDetails.landingRotation))   //
             + " time: " + to_string(int(scoreDetails.duration * 100.f)) //
        ;
    }

    float score{ 0.f };
    bool hasLanded{ false };
    struct TrajectoryElem
    {
      P position;
      int angle;
      int thrust;
    };
    vector<TrajectoryElem> trajectory;
  };

  struct Chromosome
  {
    vector<ShipControl> sc{ NUM_ALLELES };
    size_t endSc;
    auto lastCommand() { return sc.begin() + endSc; }
  };

  struct Phenotype
  {
    int parentNum0, parentNum1;
    Chromosome chr;
    Result result;

    string chromosomeToString()
    {
      string ret;
      ret += chr.endSc;
      ret += "\n";
      for(auto sc{ chr.sc.begin() }; sc != chr.sc.end(); sc++) {
        ret += to_string(sc->t) + ",";
      }
      ret += "\n";
      for(auto sc{ chr.sc.begin() }; sc != chr.sc.end(); sc++) {
        ret += to_string(sc->thrust) + ",";
      }
      ret += "\n";
      for(auto sc{ chr.sc.begin() }; sc != chr.sc.end(); sc++) {
        ret += to_string(sc->angle) + ",";
      }
      ret += "\n";
      return ret;
    }
  };

  vector<vector<Phenotype>> population;
  vector<Phenotype> highscores;
  const InitialData initialData;

public:
  MarsLanderGeneticAlgorithm(const InitialData& initialData) : initialData(initialData)
  {
    srand(SEED);
  }

  void populationInitialize()
  {
    population.clear();
    population.push_back(vector<Phenotype>(NUM_PHENOTYPES));
    for(auto& p : population[0]) {
      int tPrev{ 0 };
      for(auto& a : p.chr.sc) {
        a.t = tPrev + rand() % 10;
        tPrev = { a.t };
        a.thrust = rand() % 5;
        a.angle = (rand() % 181) - 90;
      }
      p.chr.sc[0].t = 0;
      p.chr.endSc = NUM_ALLELES;
    }
  }

  // void testFunctions()
  // {
  //   populationInitialize();
  //   if(population.size() != 1)
  //     throw std::runtime_error("populationInitialize: population size != 1");
  //   for(auto& p : population[0]) {
  //     if(p.alleles[0].d_t != 0) {
  //       throw std::runtime_error("populationInitialize: alleles[0].d_t != 0");
  //     }
  //   }
  //
  //   populationSimulateAndSort(population[0]);
  // };

  void execute(bool visualize = false)
  {
    cerr << "Starting genetic algorithm" << endl;
    populationInitialize();
    populationSimulateAndSort(population[0]);
    highscores.push_back(population[0][0]);
    int gen{ 0 };
    while(population.size() < MAX_EPOCH) {
      gen++;
      population.push_back(vector<Phenotype>(NUM_PHENOTYPES));
      auto& prevPopulation{ *(population.end() - 2) };
      auto& currPopulation{ *(population.end() - 1) };
      // Elitist choice
      std::copy(prevPopulation.begin(),
                prevPopulation.begin() + NUM_ELITE,
                currPopulation.begin());
      for(auto it{ currPopulation.begin() }; it != currPopulation.begin() + NUM_ELITE;
          it++) {
        it->result.trajectory.clear();
      }
      populationCrossover(prevPopulation.begin(),
                          prevPopulation.begin() + NUM_SELECT,
                          currPopulation.begin() + NUM_ELITE,
                          currPopulation.end(),
                          CrossoverStrategy::NORMAL);
      populationMutation(currPopulation.begin() + NUM_ELITE,
                         currPopulation.end(),
                         MutationStrategy::UNIFORM);
      populationSimulateAndSort(currPopulation);
      if(currPopulation[0].result.score > (highscores.end() - 1)->result.score) {
        highscores.push_back(currPopulation[0]);
        cerr << "Gen: " << gen << " score: " << currPopulation[0].result.score << endl;
        if(currPopulation[0].result.hasLanded) {
          cerr << "Ending: Landing succesful" << endl;
          cout << currPopulation[0].chromosomeToString();
          break;
        }
      }
    }
    if(population.size() == MAX_EPOCH) {
      cerr << "Ending: Max epoch reached" << endl;
    }
    auto& currPopulation{ *(population.end() - 1) };
    cout << "Gen: " << gen << " score: " << currPopulation[0].result.score << endl;

    if(visualize) {
      Visualiser visualizer(population, initialData.land);
      visualizer.mainLoop();
    }
  }

  void populationSimulateAndSort(vector<Phenotype>& currentPopulation)
  {
    for(auto& p : currentPopulation) {
      simulate(initialData, p);
    }
    sort(currentPopulation.begin(),
         currentPopulation.end(),
         [](Phenotype& a, Phenotype& b) { return a.result.score > b.result.score; });
  }

  const Phenotype& tournament_selection(vector<Phenotype>::const_iterator beg,
                                        vector<Phenotype>::const_iterator end,
                                        int k)
  {
    if(k <= 0)
      return *beg;
    const Phenotype* best{ nullptr };
    for(int i{ 0 }; i < k; i++) {
      const Phenotype& ind{ *((rand() % (beg - end)) + beg) };
      if(best == nullptr || ind.result.score > best->result.score) {
        best = &ind;
      }
    }
    return *best;
  }

  enum class CrossoverStrategy
  {
    UNIFORM,
    NORMAL,
    MIDPOINT,
    MERGE
  };
  void populationCrossover(vector<Phenotype>::const_iterator fromBegin,
                           vector<Phenotype>::const_iterator fromEnd,
                           vector<Phenotype>::iterator destBegin,
                           vector<Phenotype>::iterator destEnd,
                           CrossoverStrategy strategy,
                           bool verbose = false)
  {
    for(auto p{ destBegin }; p != destEnd; p++) {
      const Phenotype *parent1{ nullptr }, *parent2{ nullptr };

      do {
        parent1
          = &tournament_selection(fromBegin, fromEnd, min(2, (int)(fromEnd - fromBegin)));
        parent2
          = &tournament_selection(fromBegin, fromEnd, min(2, (int)(fromEnd - fromBegin)));
      } while(parent1 == parent2);

      if(parent2->chr.endSc < parent1->chr.endSc) {
        swap(parent1, parent2);
      }

      p->parentNum0 = parent1 - &(*fromBegin);
      p->parentNum1 = parent2 - &(*fromBegin);

      if(verbose) {
        cerr << "Crossover between parent " << p->parentNum0 << " and " << p->parentNum1
             << "\n";
      }

      if(strategy == CrossoverStrategy::MERGE) {
        auto sc1{ parent1->chr.sc.begin() };
        auto sc1end{ parent1->chr.sc.end() };
        auto sc2{ parent2->chr.sc.begin() };
        auto sc2end{ parent2->chr.sc.end() };
        auto sc{ p->chr.sc.begin() };

        while(sc1 != sc1end && sc2 != sc2end && sc != p->chr.sc.end()) {
          if(sc1->t == sc2->t) { // Two commands at the same time: pick the mean
            sc->t = sc1->t;
            sc->thrust = (sc1->thrust + sc2->thrust) / 2;
            sc->angle = (sc1->angle + sc2->angle) / 2;
            sc1++;
            sc2++;
          }
          else if(sc1->t < sc2->t) {
            *sc = *(sc1++);
          }
          else {
            *sc = *(sc2++);
          }
          sc++;
        }
        while(sc1 != sc1end && sc != p->chr.sc.end()) {
          *(sc++) = *(sc1++);
        }
        while(sc2 != sc2end && sc != p->chr.sc.end()) {
          *(sc++) = *(sc2++);
        }
        if(sc != p->chr.sc.end()) {
          cout << "sc left to fill: " << p->chr.sc.end() - sc << "\n";
          cout << "sc1 left to iter: " << sc1end - sc1 << "\n";
          cout << "sc2 left to iter: " << sc2end - sc2 << "\n";
          throw std::runtime_error("Fatal error: Merge crossover unsuccesful");
        }
      }
      if(strategy == CrossoverStrategy::NORMAL) {
        size_t splitPointMax{ parent1->chr.endSc - 1 };
        const size_t splitPoint(rand() % (splitPointMax + 1));

        for(size_t it{ 0 }; it != splitPoint; it++) {
          p->chr.sc[it] = parent1->chr.sc[it];
        }
        for(size_t it{ splitPoint }; it != NUM_ALLELES; it++) {
          p->chr.sc[it] = parent2->chr.sc[it];
        }
        p->chr.endSc = parent2->chr.endSc;
      }
      else if(strategy == CrossoverStrategy::MIDPOINT) {
        for(size_t it{ 0 }; it != NUM_ALLELES; it++) {
          p->chr.sc[it].t = (parent1->chr.sc[it].t + parent2->chr.sc[it].t) / 2;
          p->chr.sc[it].thrust
            = (parent1->chr.sc[it].thrust + parent2->chr.sc[it].thrust) / 2;
          p->chr.sc[it].angle
            = (parent1->chr.sc[it].angle + parent2->chr.sc[it].angle) / 2;
        }
        p->chr.endSc = max(parent1->chr.endSc, parent2->chr.endSc);
      }
      else if(strategy == CrossoverStrategy::UNIFORM) {
        for(size_t it{ 0 }; it != NUM_ALLELES; it++) {
          p->chr.sc[it] = (rand() % 2) == 0 ? parent1->chr.sc[it] : parent2->chr.sc[it];
        }
        p->chr.endSc = max(parent1->chr.endSc, parent2->chr.endSc);
        sort(
          p->chr.sc.begin(), p->chr.sc.end(), [](auto& a, auto& b) { return a.t < b.t; });
      }
    }
  }

  enum class MutationStrategy
  {
    UNIFORM,
    DIFFERENTIAL
  };
  void populationMutation(vector<Phenotype>::iterator beg,
                          vector<Phenotype>::iterator end,
                          MutationStrategy strategy)
  {
    int numMut{ 0 };
    for(auto p{ beg }; p != end; p++) {
      if(float(rand()) / float(RAND_MAX) > MUTATION_P) {
        continue;
      }
      numMut++;

      size_t targetAllele;
      int targetValue;

      do {
        targetAllele = rand() % p->chr.endSc;
        targetValue = rand() % 3;
      } while(targetAllele == 0 && targetValue == 0); // allele[0].t remains identical

      int tPrev{ targetAllele == 0 ? 0 : p->chr.sc[targetAllele - 1].t };
      int& t{ p->chr.sc[targetAllele].t };
      int& angle{ p->chr.sc[targetAllele].angle };
      int& thrust{ p->chr.sc[targetAllele].thrust };

      switch(targetValue) {
      case 0:
        switch(strategy) {
        case MutationStrategy::UNIFORM: t = tPrev + rand() % 20; break;
        case MutationStrategy::DIFFERENTIAL:
          t += rand() % 30;
          t -= 2 * min(t - tPrev, 0);
          break;
        }

      case 1:
        switch(strategy) {
        case MutationStrategy::UNIFORM: angle = rand() % 181 - 90; break;
        case MutationStrategy::DIFFERENTIAL:
          angle += rand() % 20;
          angle -= 2 * min(angle - -180, 0);
          angle -= 2 * max(angle - 180, 0);
          break;
        }

      case 2:
        switch(strategy) {
        case MutationStrategy::UNIFORM: thrust = rand() % 5; break;
        case MutationStrategy::DIFFERENTIAL:
          thrust += rand() % 3 - 1;
          thrust -= 2 * min(thrust - 0, 0);
          thrust -= 2 * max(thrust - 4, 0);
          break;
        }
      }
    }
    // cerr << "Num mut: " << numMut << "\n";
  }

  static void simulate(const InitialData& initialData, Phenotype& pheno)
  {
    int t{ 0 };
    P position{ initialData.position };
    P velocity{ 0, 0 };
    int fuel{ initialData.fuel };
    int target_thrust{ pheno.chr.sc[0].thrust };
    int thrust{ initialData.power };
    int target_rotate{ pheno.chr.sc[0].angle };
    int rotate{ initialData.rotate };
    auto step{ pheno.chr.sc.begin() + 1 };
    const auto stepend{ pheno.chr.sc.end() };

    // Stop conditions
    bool hasCollided{ false };
    bool hasLeftBoundaries{ false };
    bool hasExhaustedFuel{ false };

    while(!(hasCollided || hasLeftBoundaries || hasExhaustedFuel)) {
      // Apply changes from phenotype
      t++;
      if(step != stepend && t >= step->t) {
        target_thrust = step->thrust;
        target_rotate = step->angle;
        step++;
      }
      // Thrust control
      thrust += min(max(-1, target_thrust - thrust), 1);
      thrust = min(max(0, thrust), 4);
      fuel -= thrust;
      // Rotate control
      rotate += min(max(-15, target_rotate - rotate), 15);
      rotate = max(min(90, rotate), -90);
      // Calculate next step
      P previousPosistion{ position };
      fuel -= thrust;
      P acceleration = { -thrust * sin(rotate / 180.f * 3.15159265f),
                         G + thrust * cos(rotate / 180.f * 3.14159265f) };
      velocity = { velocity.x + acceleration.x, velocity.y + acceleration.y };
      position = { position.x + velocity.x, position.y + velocity.y };
      // Save trajectory
      pheno.result.trajectory.push_back({ position, rotate, thrust });
      // Check for termination conditions
      if(collision(previousPosistion, position, initialData.land)) {
        hasCollided = true;
      }
      if(position.x < 0 || position.x > BOUNDARIES.x || position.y < 0
         || position.y > BOUNDARIES.y) {
        hasLeftBoundaries = true;
      }
      if(fuel <= 0) {
        hasExhaustedFuel = true;
      }
    }

    pheno.chr.endSc = step - pheno.chr.sc.begin();
    // Calculate results
    if(hasExhaustedFuel) {
      pheno.result.score -= 10000; // Wasted fuel
    }
    if(hasLeftBoundaries) {
      pheno.result.score -= 10000; // Wrong direction
    }
    // bool aboveTarget
    //   = position.x >= initialData.target_x1 && position.x <= initialData.target_x2;
    float penaltyLandingSpeedX = max(abs(velocity.x) - 20.f, 0.f);
    float penaltyLandingSpeedY = max(abs(velocity.y) - 40.f, 0.f);
    float penaltyLandingRotation = abs(rotate);
    float distY = abs(initialData.target_y - position.y);
    float distYThres{ 0.f };
    if(distY > 5.f || distY < -5.f)
      distYThres = abs(distY);
    float distX = max(
      0.f, max(initialData.target_x1 - position.x, position.x - initialData.target_x2));
    auto sigmoid = [](float x) { return x / (1.f + abs(x)); };
    auto& details{ pheno.result.scoreDetails };
    details.distX = -pow(distX, 2);
    details.distY = -distY;
    bool reachedLanding{ distX == 0 && distYThres == 0 };
    details.landingSpeedX = !reachedLanding ? -10000.f : -penaltyLandingSpeedX;
    details.landingSpeedY = !reachedLanding ? -10000.f : -penaltyLandingSpeedY;
    details.landingRotation = !reachedLanding ? -10000.f : -penaltyLandingRotation;
    details.duration = -t / 100.f;
    pheno.result.score = details.distX + details.distY + details.landingSpeedX
                         + details.landingSpeedY + details.landingRotation
                         + details.duration;
    if(reachedLanding && penaltyLandingSpeedX == 0.f && penaltyLandingSpeedY == 0.f
       && penaltyLandingRotation == 0.f) {
      pheno.result.hasLanded = true;
    }
    return;
  }

  class Visualiser
  {
  public:
    size_t gen;
    size_t pheno;
    vector<vector<Phenotype>>& population;
    const vector<P>& land;
    static constexpr float SCALE{ 1000.f / 7000.f };

    bool showOnlySelectedPheno{ false };

    Visualiser(vector<vector<Phenotype>>& population, const vector<P>& land)
        : population(population), land(land)
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
      for(auto& p : population[gen]) {
        cout << p.result.score << " ";
      }
      cout << "\n";
    }

    void eventCheck()
    {
      if(IsKeyPressed(KEY_SPACE)) {
        showOnlySelectedPheno = !showOnlySelectedPheno;
      }

      if(IsKeyPressedRepeat(KEY_LEFT) || IsKeyPressed(KEY_LEFT)) {
        if(gen != 0) {
          gen--;
        }
        else {
          gen = population.size() - 1;
        }
      }
      if(IsKeyPressedRepeat(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT)) {
        if(gen != population.size() - 1) {
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
        if(pheno != NUM_PHENOTYPES - 1) {
          pheno++;
        }
      }
    }

    void render()
    {
      BeginDrawing();
      ClearBackground(WHITE);

      for(auto it{ land.begin() + 1 }; it != land.end(); it++) {
        auto pit = it - 1;
        DrawLine(pit->x * SCALE, pit->y * SCALE, it->x * SCALE, it->y * SCALE, RED);
      }

      auto beg{ population[gen].begin() };
      for(auto p{ beg }; p != population[gen].end(); p++) {
        Color drawingColor;
        if(p - beg == pheno) {
          drawingColor = PINK;
        }
        else if(showOnlySelectedPheno) {
          continue;
        }
        else if(p - beg < NUM_ELITE) {
          drawingColor = GREEN;
        }
        else {
          drawingColor = ORANGE;
        }
        for(auto& t : p->result.trajectory) {
          int startPosX = t.position.x * SCALE;
          int startPosY = t.position.y * SCALE;
          // P acceleration = { -thrust * sin(rotate / 180.f * 3.15159265f),
          //                G + thrust * cos(rotate / 180.f / 3.14159265f) };
          int endPosX
            = startPosX
              + SCALE * 10 * (t.thrust + 1) * sin(t.angle / 180.f * 3.14159265f);
          int endPosY
            = startPosY - SCALE * 10 * (t.thrust + 1) * cos(t.angle / 180.f * 3.14159265);
          DrawLine(startPosX, startPosY, endPosX, endPosY, drawingColor);
        }
        auto& finalPosition{ (p->result.trajectory.end() - 1)->position };
        DrawText(to_string(int(p->result.score * 100)).c_str(),
                 finalPosition.x * SCALE,
                 finalPosition.y * SCALE,
                 15,
                 drawingColor);
        DrawText(("Pheno: " + to_string(pheno) + " "
                  + population[gen][pheno].result.scoreString())
                   .c_str(),
                 150,
                 400,
                 30,
                 PINK);
      }
      int t{ 0 };
      for(size_t a{ 0 }; a < NUM_ALLELES; a++) {
        auto& sc{ population[gen][pheno].chr.sc[a] };
        DrawText(to_string(sc.t).c_str(), 150 + a * 20, 430, 15, PINK);
        DrawText(to_string(sc.thrust).c_str(), 150 + a * 20, 450, 15, PINK);
        DrawText(to_string(sc.angle).c_str(), 150 + a * 20, 470, 15, PINK);
      }
      DrawText(("Parent 1: " + to_string(population[gen][pheno].parentNum0)).c_str(),
               150,
               500,
               30,
               PINK);
      DrawText(("Parent 2: " + to_string(population[gen][pheno].parentNum1)).c_str(),
               150,
               550,
               30,
               PINK);
      DrawText(("Gen: " + to_string(gen)).c_str(), 30, 400, 30, RED);

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
};

namespace po = boost::program_options;

class CmdArgs
{
  struct Options
  {
    bool help_{ false };
    bool visualize_{ false };
  } opt;

  bool shouldExit_{ false };

public:
  CmdArgs(int argc, char** argv) { parseOptions(argc, argv); }

  void parseOptions(int argc, char** argv)
  {
    po::options_description desc("OPTIONS");
    desc.add_options()("help", "Put help message in output stream and exit")(
      "visualize", "Open the UI to view the details");
    po::variables_map argmap;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), argmap);
    po::notify(argmap);

    if(argmap.count("help")) {
      opt.help_ = true;
      cout << desc << "\n";
      shouldExit_ = true;
    }
    if(argmap.count("visualize")) {
      opt.visualize_ = true;
    }
  }

  auto getVisualize() { return opt.visualize_; }
  auto isHelpEnabled() { return opt.help_; }
  bool shouldExit() { return shouldExit_; }
};

int main(int argc, char* argv[])
{
  CmdArgs cmdArgs(argc, argv);
  if(cmdArgs.shouldExit() == true)
    return 0;

  InitialData init;

  int surface_n; // the number of points used to draw the surface of Mars.
  cin >> surface_n;
  cin.ignore();
  init.land.resize(surface_n);

  for(int i = 0; i < surface_n; i++) {
    cin >> init.land[i].x >> init.land[i].y;
    cin.ignore();
    if(i > 0 && init.land[i].x - init.land[i - 1].x >= 1000
       && init.land[i - 1].y == init.land[i].y) {
      init.target_x1 = init.land[i - 1].x;
      init.target_x2 = init.land[i].x;
      init.target_y = init.land[i].y;
    }
  }

  int t{ 0 };
  bool firstLoop{ true };
  MarsLanderGeneticAlgorithm* algo{ nullptr };

  // game loop
  while(1) {
    int x;
    int y;
    int h_speed; // the horizontal speed (in m/s), can be negative.
    int v_speed; // the vertical speed (in m/s), can be negative.
    int fuel;    // the quantity of remaining fuel in liters.
    int rotate;  // the rotation angle in degrees (-90 to 90).
    int power;   // the thrust power (0 to 4).
    cin >> x >> y >> h_speed >> v_speed >> fuel >> rotate >> power;
    cin.ignore();

    if(firstLoop) {
      firstLoop = false;
      init.position = { (float)x, (float)y };
      init.fuel = fuel;
      init.power = power;
      init.rotate = rotate;

      algo = new MarsLanderGeneticAlgorithm(init);
      algo->execute(cmdArgs.getVisualize());
      delete algo;

      break;
    }

    cout << "-20 3" << endl;
    t++;
  }
}
