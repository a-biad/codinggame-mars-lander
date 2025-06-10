#if !CODING_GAME
#include "CmdArgs.hpp"
#include "GeneticAlgorithm.hpp"
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
  while(input.position.y > -10.f) {
    if(!ga.done) {
      ga.execute(500);
#if !CODING_GAME
      if(options.visualize) {
        GAVisualizer::Visualiser visualiser{ ga.gatrace.get() };
        visualiser.setInput(&input);
        visualiser.setParameters(&ga.param);
        visualiser.mainLoop();
      }
#endif
    }
    input.sc = ga.nextSc();
    cout << input.sc << "\n";
    input.velocity += input.sc.acceleration();
    input.position += input.velocity;
    ga.changeInput(input);
  }
  return 0;
}