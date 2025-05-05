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
  static const int NUM_ALLELES{ 5 };
  static const int NUM_PHENOTYPES{ 200 };
  static const int SEED{ 4321 };
  static constexpr float ELITISM_RATE{ 0.5f };
  static const size_t ELITISM_NUMBER{ (size_t)(NUM_PHENOTYPES * ELITISM_RATE) };
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
    struct ScoreDetails
    {
      float distX;
      float distY;
      float landingSpeedX;
      float landingSpeedY;
      float landingRotation;
      float fuel;
    } scoreDetails;

    string scoreString()
    {
      return "Score: " + to_string(score)                              //
             + " x: " + to_string(int(scoreDetails.distX))             //
             + " y: " + to_string(int(scoreDetails.distY))             //
             + " spdX: " + to_string(int(scoreDetails.landingSpeedX))  //
             + " spdY: " + to_string(int(scoreDetails.landingSpeedY))  //
             + " rot: " + to_string(int(scoreDetails.landingRotation)) //
             + " fuel: " + to_string(int(scoreDetails.fuel))           //
        ;
    }
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

    bool firstLoop{ true };

    while(population.size() < MAX_EPOCH) {
      gen++;
      population.push_back(vector<Phenotype>(NUM_PHENOTYPES));
      auto& prevPopulation{ *(population.end() - 2) };
      auto& currPopulation{ *(population.end() - 1) };
      // Elitist choice
      if(firstLoop) {
        cerr << "NUM_PHENOTYPES: " << NUM_PHENOTYPES << endl;
        cerr << "ELITISM_NUMBER: " << ELITISM_NUMBER << endl;
      }
      std::copy(prevPopulation.begin(),
                prevPopulation.begin() + ELITISM_NUMBER,
                currPopulation.begin());
      populationCrossover(currPopulation.begin(),
                          currPopulation.begin() + ELITISM_NUMBER,
                          currPopulation.begin() + ELITISM_NUMBER,
                          currPopulation.end());
      populationMutation(currPopulation.begin() + ELITISM_NUMBER, currPopulation.end());
      populationSimulateAndSort(currPopulation);
      if(currPopulation[0].result.score > (highscores.end() - 1)->result.score) {
        highscores.push_back(currPopulation[0]);
        cerr << "Gen: " << gen << " score: " << currPopulation[0].result.score << endl;
        if(currPopulation[0].result.hasLanded) {
          cerr << "Ending: Landing succesful" << endl;
          break;
        }
      }
      firstLoop = false;
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

  enum StrategyCrossover
  {
    UNIFORM,
    NORMAL
  };
  void populationCrossover(vector<Phenotype>::const_iterator fromBegin,
                           vector<Phenotype>::const_iterator fromEnd,
                           vector<Phenotype>::iterator destBegin,
                           vector<Phenotype>::iterator destEnd,
                           StrategyCrossover strategy = NORMAL)
  {
    for(auto p{ destBegin }; p != destEnd; p++) {
      auto& parent1{ tournament_selection(
        fromBegin, fromEnd, min(2, (int)(fromEnd - fromBegin))) };
      auto& parent2{ tournament_selection(
        fromBegin, fromEnd, min(2, (int)(fromEnd - fromBegin))) };

      if(strategy == NORMAL) {
        const size_t split_point(rand() % (NUM_ALLELES + 1));

        for(size_t it{ 0 }; it != split_point; it++) {
          p->alleles[it] = parent1.alleles[it];
        }
        for(size_t it{ split_point }; it != NUM_ALLELES; it++) {
          p->alleles[it] = parent2.alleles[it];
        }
      }
      else if(strategy == UNIFORM) {
        for(size_t it{ 0 }; it != NUM_ALLELES; it++) {
          p->alleles[it] = (rand() % 2) == 0 ? parent1.alleles[it] : parent2.alleles[it];
        }
      }
    }
  }

  void
  populationMutation(vector<Phenotype>::iterator beg, vector<Phenotype>::iterator end)
  {
    for(auto p{ beg }; p != end; p++) {
      size_t targetAllele(rand() % NUM_ALLELES);
      int& d_t{ p->alleles[targetAllele].d_t };
      int& d_angle{ p->alleles[targetAllele].d_angle };
      int& d_thrust{ p->alleles[targetAllele].d_thrust };

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
      distY = abs(distYtemp);
    float distX = max(
      0.f, max(initialData.target_x1 - position.x, position.x - initialData.target_x2));
    auto sigmoid = [](float x) { return x / (1.f + abs(x)); };
    auto& details{ pheno.result.scoreDetails };
    details.distX = -pow(distX / 100.f, 2);
    details.distY = -pow(distY / 100.f, 2);
    details.landingSpeedX = -pow(penaltyLandingSpeedX, 2);
    details.landingSpeedY = -pow(penaltyLandingSpeedY, 2);
    details.landingRotation = -pow(penaltyLandingRotation, 2);
    details.fuel = float(fuel) / float(initialData.fuel);
    pheno.result.score = details.distX + details.distY + details.landingSpeedX
                         + details.landingSpeedY + details.landingRotation + details.fuel;
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
    size_t pheno;
    vector<vector<Phenotype>>& population;
    const vector<P>& land;
    static constexpr float SCALE{ 1000.f / 7000.f };

    Visualiser(vector<vector<Phenotype>>& population, const vector<P>& land)
        : population(population), land(land)
    {
      gen = 0;
      pheno = 0;
      InitWindow(1366.f, 700.f,
                 "Mars lander"); // Initialize window and OpenGL context
    }

    void printCurrentGenInfo()
    {
      cerr << "Gen " << gen << " : ";
      for(auto& p : population[gen]) {
        cerr << p.result.score << " ";
      }
      cerr << endl;
    }

    void eventCheck()
    {
      if(IsKeyPressedRepeat(KEY_LEFT) || IsKeyPressed(KEY_LEFT)) {
        if(gen != 0) {
          gen--;
        }
      }
      if(IsKeyPressedRepeat(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT)) {
        if(gen != population.size() - 1) {
          gen++;
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
        else if(p - beg < ELITISM_NUMBER) {
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
        DrawText(("Pheno: " + to_string(pheno) + " "
                  + population[gen][pheno].result.scoreString())
                   .c_str(),
                 150,
                 400,
                 30,
                 PINK);
      }
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
