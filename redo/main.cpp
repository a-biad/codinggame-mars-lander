#include <iostream>
#include <fstream>
#include <memory>

#include "geneticalgorithm/GeneticAlgorithm.h"
#include "geneticalgorithm/Visualizer.h"

using namespace ::simulation;
using namespace ::geneticalgorithm;

struct OuterParameters
{
  int numGenPerIter{ 100 };
  bool infiniteExecution{ false };
};

void
run(std::istream& is,
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
      genetic_algorithm
        = std::make_unique<
          GeneticAlgorithm>(parameters, Simulation{ land, is }, &gatrace);
    }
    is.ignore();

    // Run step of GA
    if(!genetic_algorithm->simulation().isDone()) {
      genetic_algorithm->execute(outerParameters.numGenPerIter);
    }
    else { genetic_algorithm->flushSimulationWithBest(); }

    // Output value given by GA
    if(auto sc{ genetic_algorithm->getSc() }; sc.has_value()) {
      std::cout << sc.value() << std::endl;
    }
    else {
      if(outerParameters.infiniteExecution) { std::cout << "0 0" << std::endl; }
      else { break; }
    }
  }

  if(parameters.trace) {
    Visualizer visualizer(gatrace);
    visualizer.show();
  }
}

void
runFile(const std::string& inputFileName = std::string("../input/input1.txt"),
        const Parameters& parameters,
        const OuterParameters& outerParameters)
{
  std::ifstream is{ inputFileName };
  run(is, parameters, outerParameters);
}

void
runAllFiles(const Parameters& parameters, const OuterParameters& outer_parameters)
{
  std::string fileNames[] = {
    "../input/input1.txt",
    "../input/input2.txt",
    "../input/input3.txt",
    "../input/input4.txt",
    "../input/input5.txt"
  };

  for(auto& fileName : fileNames) {
    runFile(fileName, parameters, outer_parameters);
  }
}

void runCG(const Parameters& parameters)
{
  Parameters actualParams {parameters};
  actualParams.trace = false;

  OuterParameters outer_parameters;
  outer_parameters.numGenPerIter = 100;
  outer_parameters.infiniteExecution = true;

  run(std::cin, actualParams, outer_parameters);
}

int
main(int argc, char** argv)
{
  // std::string inputFileName{ "../input/input1.txt" };
  // if(argc > 1) { inputFileName = argv[1]; }
  //
  // Parameters parameters{ defaultParameters() };
  // parameters.trace = true;
  //
  // OuterParameters outer_parameters;
  // outer_parameters.numGenPerIter = 100;
  // outer_parameters.infiniteExecution = false;
  //
  // runFile(inputFileName, parameters, outer_parameters);

  runCG( defaultParameters() );

  return 0;
}