#include <fstream>
#include <iostream>
#include <memory>

// https://github.com/badaix/popl
#include "external/popl.hpp"

#include "geneticalgorithm/GeneticAlgorithm.h"
#include "geneticalgorithm/Visualizer.h"

using namespace ::simulation;
using namespace ::geneticalgorithm;

struct OuterParameters
{
  bool helpAndExit{ false };
  std::vector<std::string> testcaseFiles{};
  int numGenPerIter{ 100 };
  bool infiniteExecution{ false };
  bool logSimulatedState{ true };
};

void run(std::istream& is,
         const Parameters& parameters,
         const OuterParameters& outerParameters)
{
  Land land;
  land.fetchFromIstream(is);

  std::unique_ptr<GeneticAlgorithm> genetic_algorithm;
  GeneticAlgorithmTrace gatrace(parameters);

  bool firstLoop{ true };

  while(true) {
    // Fetch line from input
    if(firstLoop) {
      firstLoop = false;
      genetic_algorithm = std::make_unique<GeneticAlgorithm>(
        parameters, Simulation{ land, is }, &gatrace);
    }
    else {
      int x;
      int y;
      int h_speed; // the horizontal speed (in m/s), can be negative.
      int v_speed; // the vertical speed (in m/s), can be negative.
      int fuel;    // the quantity of remaining fuel in liters.
      int rotate;  // the rotation angle in degrees (-90 to 90).
      int power;   // the thrust power (0 to 4).
      is >> x >> y >> h_speed >> v_speed >> fuel >> rotate >> power;
      is.ignore();
    }

    // Run step of GA
    if(!genetic_algorithm->simulation().isDone()) {
      genetic_algorithm->execute(outerParameters.numGenPerIter);
    }
    else {
      std::cerr << "-- Genetic Algorithm complete" << "\n";
    }

    // Output value given by GA
    if(auto sc{ genetic_algorithm->getSc() }; sc.has_value()) {
      std::cout << sc.value() << std::endl;
    }
    else {
      if(outerParameters.infiniteExecution) {
        std::cout << "0 0" << std::endl;
      }
      else {
        break;
      }
    }

    if(outerParameters.logSimulatedState) {
      const auto& s{ genetic_algorithm->simulation() };
      std::cerr << "Position: " << s.position().x << " " << s.position().y << "\n";
      std::cerr << "Velocity: " << s.velocity().x << " " << s.velocity().y << "\n";
      std::cerr << "Fuel: " << s.fuel() << "\n";
      std::cerr << "SC: " << s.currentSC() << "\n";
      std::cerr.flush();
    }
  }

  if(parameters.trace) {
    geneticalgorithm::Visualizer visualizer(gatrace);
    visualizer.show();
  }
}

void runFile(const std::string& inputFileName,
             const Parameters& parameters,
             const OuterParameters& outerParameters)
{
  std::ifstream is{ inputFileName };
  run(is, parameters, outerParameters);
}

void runAllFiles(const Parameters& parameters, const OuterParameters& outer_parameters)
{
  std::string fileNames[] = {
    "../input/input1.txt", "../input/input2.txt", "../input/input3.txt",
    "../input/input4.txt", "../input/input5.txt", "../input/input6.txt",
    "../input/input7.txt",
  };

  for(auto& fileName : fileNames) {
    runFile(fileName, parameters, outer_parameters);
  }
}

void runCG(const Parameters& parameters, const OuterParameters& outer_parameters)
{
  Parameters actualp{ parameters };
  actualp.trace = false;

  OuterParameters acutalop{ outer_parameters };
  acutalop.infiniteExecution = true;

  run(std::cin, actualp, acutalop);
}

void setParametersFromCommandLine(int argc,
                                  char** argv,
                                  Parameters& parameters,
                                  OuterParameters& outer_parameters)
{
  using popl::OptionParser, popl::Switch, popl::Value;

  OptionParser op("Allowed options");
  auto help_option = op.add<Switch>("h", "help", "produce help message");
  auto input_testcase_option
    = op.add<Value<std::string>>("t", "testcase", "a testcase file");
  op.parse(argc, argv);

  if(help_option->count() == 1) {
    outer_parameters.helpAndExit = true;
    std::cerr << op.help(popl::Attribute::expert) << "\n";
  }
  if(input_testcase_option->is_set()) {
    for(size_t n = 0; n < input_testcase_option->count(); ++n) {
      std::cerr << "testcase_option #" << n
                << " - value: " << input_testcase_option->value(n) << "\n";
    }
    if(input_testcase_option->count() == 1) {
      outer_parameters.testcaseFiles.push_back(input_testcase_option->value(0));
    }
  }
}

void runFromCommandLine(int argc,
                        char** argv,
                        const Parameters& parameters_default,
                        const OuterParameters& outer_parameters_default)
{
  Parameters parameters = parameters_default;
  OuterParameters outer_parameters = outer_parameters_default;
  setParametersFromCommandLine(argc, argv, parameters, outer_parameters);

  if(outer_parameters.helpAndExit) {
    return;
  }

  if(outer_parameters.testcaseFiles.empty()) {
    runFile("../input/input1.txt", parameters, outer_parameters);
    return;
  }
  for(auto& testcaseFile : outer_parameters.testcaseFiles) {
    runFile(testcaseFile, parameters, outer_parameters);
  }
}

int main(int argc, char** argv)
{
  OuterParameters outer_parameters;
  outer_parameters.numGenPerIter = 10;
  outer_parameters.infiniteExecution = false;
  outer_parameters.logSimulatedState = CGEXPORT;

  Parameters parameters{ defaultParameters() };
  parameters.trace = true;

  if constexpr(CGEXPORT) {
    //
    runCG(parameters, outer_parameters);
  }
  else {
    //
    runFromCommandLine(argc, argv, parameters, outer_parameters);
  }

  return 0;
}
