#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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
};

class MarsLanderGeneticAlgorithm
{
  static const int NUM_ALLELES{ 20 };
  static const int NUM_PHENOTYPES{ 200 };
  static const int SEED{ 1234 };
  static constexpr float SELECTION_RATE{ 0.8f };
  static const size_t SELECTION_NUMBER{ (size_t)(NUM_PHENOTYPES * SELECTION_RATE) };
  static const size_t MAX_EPOCH{ 1000 };

  struct Allele
  {
    int d_t;
    int d_thrust;
    int d_angle;
  };

  struct Result
  {
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

  struct Phenotype
  {
    vector<Allele> alleles{ NUM_ALLELES };
    Result result;
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
      for(auto& a : p.alleles) {
        a.d_t = rand() % 60;
        a.d_thrust = (rand() % 9) - 4;
        a.d_angle = (rand() % 181) - 90;
      }
    }
  }

  void execute()
  {
    cerr << "Starting genetic algorithm" << endl;
    populationInitialize();
    populationSimulateAndSort(population[0]);
    highscores.push_back(population[0][0]);
    int gen{ 1 };

    while(population.size() < MAX_EPOCH) {
      gen++;
      population.push_back(vector<Phenotype>(NUM_PHENOTYPES));
      auto& prevPopulation{ *(population.end() - 2) };
      auto& currPopulation{ *(population.end() - 1) };
      populationCrossover(prevPopulation, currPopulation);
      populationMutation(currPopulation);
      populationSimulateAndSort(currPopulation);
      if(currPopulation[0].result.score > (highscores.end() - 1)->result.score) {
        highscores.push_back(currPopulation[0]);
        cerr << "Gen: " << gen << " score: " << currPopulation[0].result.score << endl;
        if(currPopulation[0].result.hasLanded) {
          cerr << "Ending: Landing succesful" << endl;
          break;
        }
      }
    }
    if(population.size() == MAX_EPOCH) {
      cerr << "Ending: Max epoch reached" << endl;
    }
    auto& currPopulation{ *(population.end() - 1) };
    cerr << "Gen: " << gen << " score: " << currPopulation[0].result.score << endl;

    Visualiser visualizer(population, initialData.land);
    visualizer.mainLoop();
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

  enum StrategyCrossover
  {
    UNIFORM,
    NORMAL
  };
  void populationCrossover(const vector<Phenotype>& currentPopulation,
                           vector<Phenotype>& nextPopulation,
                           StrategyCrossover strategy = UNIFORM)
  {
    // auto& currentPopulation{ *(population.end() - 1) };    vector<int> v2 = v1;
    // population.push_back(vector<Phenotype>(NUM_PHENOTYPES));
    // auto& nextPopulation{ *(population.end() - 1) };

    nextPopulation.resize(NUM_PHENOTYPES);

    for(auto& p : nextPopulation) {
      vector<Phenotype>::const_iterator parent1{ currentPopulation.begin()
                                                 + (rand() % SELECTION_NUMBER) };
      vector<Phenotype>::const_iterator parent2{ currentPopulation.begin()
                                                 + (rand() % SELECTION_NUMBER) };

      if(strategy == NORMAL) {
        const size_t split_point(rand() % (NUM_ALLELES + 1));

        for(size_t it{ 0 }; it != split_point; it++) {
          p.alleles[it] = parent1->alleles[it];
        }
        for(size_t it{ split_point }; it != NUM_ALLELES; it++) {
          p.alleles[it] = parent2->alleles[it];
        }
      }
      else if(strategy == UNIFORM) {
        for(size_t it{ 0 }; it != NUM_ALLELES; it++) {
          p.alleles[it] = (rand() % 2) == 0 ? parent1->alleles[it] : parent2->alleles[it];
        }
      }
    }
  }

  void populationMutation(vector<Phenotype>& nextPopulation)
  {
    for(auto& p : nextPopulation) {
      size_t targetAllele(rand() % NUM_ALLELES);
      int& d_t{ p.alleles[targetAllele].d_t };
      int& d_angle{ p.alleles[targetAllele].d_angle };
      int& d_thrust{ p.alleles[targetAllele].d_thrust };

      int targetValue(rand() % 3);
      switch(targetValue) {
      case 0:
        d_t += rand() % 7 - 3;
        if(d_t < 0)
          d_t *= -1;
        break;
      case 1:
        d_angle += rand() % 11 - 5;
        if(d_angle < -180) {
          d_angle -= 2 * (d_angle + 180);
        }
        else if(d_angle > 180) {
          d_angle -= 2 * (d_angle - 180);
        }
        break;
      case 2:
        d_thrust += rand() % 3 - 1;
        if(d_thrust < -4) {
          d_thrust -= 2 * (d_thrust + 4);
        }
        else if(d_thrust > 4) {
          d_thrust -= 2 * (d_thrust - 4);
        }
        break;
      }
    }
  }

  /*
   * Returns the performance score of a phenotype
   */
  static void simulate(const InitialData& initialData, Phenotype& pheno)
  {
    int t{ 0 };
    P position{ initialData.position };
    P velocity{ 0, 0 };
    int fuel{ initialData.fuel };
    int target_thrust{ pheno.alleles[0].d_thrust };
    int thrust{ 0 };
    int target_rotate{ pheno.alleles[0].d_angle };
    int rotate{ 0 };
    const auto step{ pheno.alleles.begin() + 1 };
    const auto stepend{ pheno.alleles.end() };
    int tPreviousStep{ 0 };

    // Stop conditions
    bool hasCollided{ false };
    bool hasLeftBoundaries{ false };
    bool hasExhaustedFuel{ false };

    while(!(hasCollided || hasLeftBoundaries || hasExhaustedFuel)) {
      // Apply changes from phenotype
      t++;
      if(step != stepend && t - tPreviousStep >= step->d_t) {
        tPreviousStep = t;
        target_thrust += step->d_thrust;
        target_rotate += step->d_angle;
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
                         G + thrust * cos(rotate / 180.f / 3.14159265f) };
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

    // Calculate results
    // if(hasExhaustedFuel) {
    //   pheno.result.score -= 30; // Wasted fuel
    // }
    // if(hasLeftBoundaries) {
    //   pheno.result.score -= 100; // Wrong direction
    // }
    if(hasCollided) {
      if(position.x >= initialData.target_x1 && position.x <= initialData.target_x2
         && initialData.target_y - position.y <= 40 && abs(velocity.x) <= 20.f
         && abs(velocity.y) <= 40.f) {}
    }
    // bool aboveTarget
    //   = position.x >= initialData.target_x1 && position.x <= initialData.target_x2;
    float penaltyLandingSpeedX = max(abs(velocity.x) - 20.f, 0.f);
    float penaltyLandingSpeedY = max(abs(velocity.y) - 40.f, 0.f);
    float penaltyLandingRotation = abs(rotate);
    float distYtemp = initialData.target_y - position.y;
    float distY{ 0.f };
    if(distYtemp > 5.f || distYtemp < -5.f)
      distY = distYtemp;
    float distX = max(
      0.f, max(initialData.target_x1 - position.x, position.x - initialData.target_x2));
    auto sigmoid = [](float x) { return x / (1.f + abs(x)); };
    pheno.result.score -= sigmoid(distX) * 10;
    pheno.result.score -= sigmoid(abs(distY)) * 10;
    pheno.result.score += float(fuel) / float(initialData.fuel);
    if( distX == 0.f && distY == 0.f)
    pheno.result.score -= sigmoid(penaltyLandingSpeedX) + sigmoid(penaltyLandingSpeedY)
                          + sigmoid(penaltyLandingRotation);
    else 
      pheno.result.score -= 10;
    if(distX == 0.f && distY == 0.f && penaltyLandingSpeedX == 0.f
       && penaltyLandingSpeedY == 0.f && penaltyLandingRotation == 0.f) {
      pheno.result.hasLanded = true;
    }
    return;
  }

  class Visualiser
  {
  public:
    size_t gen;
    vector<vector<Phenotype>>& population;
    const vector<P>& land;
    static constexpr float SCALE{ 1000.f / 7000.f };

    Visualiser(vector<vector<Phenotype>>& population, const vector<P>& land)
        : population(population), land(land)
    {
      gen = 0;
      InitWindow(1366.f, 700.f,
                 "Mars lander"); // Initialize window and OpenGL context
    }

    void eventCheck()
    {
      if(IsKeyPressedRepeat(KEY_LEFT) || IsKeyPressed(KEY_LEFT)) {
        if(gen != 0)
          gen--;
      }
      if(IsKeyPressedRepeat(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT)) {
        gen = min(population.size() - 1, gen + 1);
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
        if(p - beg < SELECTION_NUMBER) {
          drawingColor = GREEN;
        }
        else {
          drawingColor = ORANGE;
        }
        for(auto& t : p->result.trajectory) {
          int startPosX = t.position.x * SCALE;
          int startPosY = t.position.y * SCALE;
          int endPosX = startPosX;
          int endPosY = startPosY - SCALE * 20;
          DrawLine(startPosX, startPosY, endPosX, endPosY, drawingColor);
        }
        auto& finalPosition{ (p->result.trajectory.end() - 1)->position };
        DrawText(to_string(int(p->result.score * 100)).c_str(),
                 finalPosition.x * SCALE,
                 finalPosition.y * SCALE,
                 15,
                 drawingColor);
      }

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

int main()
{
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

      algo = new MarsLanderGeneticAlgorithm(init);
      algo->execute();
      delete algo;

      break;
    }

    cout << "-20 3" << endl;
    t++;
  }
}
