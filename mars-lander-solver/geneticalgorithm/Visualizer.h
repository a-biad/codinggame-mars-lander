#pragma once

#include "Util.h"

#if !CGEXPORT

#include <sstream>
#include <raylib.h>

#include "GeneticAlgorithmTrace.h"
#include "utildraw.h"

#include <filesystem>


namespace geneticalgorithm
{
  class Visualizer
  {
    GeneticAlgorithmTrace* gatrace_;
    bool drawGeneration_{ true };

    static constexpr float SCALE{ 1300.f / 7000.f };

    static void
    drawTrajectory(const Chromosome& chr,
                   const Color& color,
                   bool simple = false);

    void
    eventCheck();

    void
    screenshotEverything(std::filesystem::path const& outDirectory = "screenshot",
                         int increment = 1);

    void
    render(bool wait = true);

  public:
    Visualizer(GeneticAlgorithmTrace& gatrace): gatrace_(&gatrace) {}

    void
    show();
  };
} // geneticalgorithm


#else

#include "GeneticAlgorithmTrace.h"

namespace geneticalgorithm
{
  class Visualizer
  {
  public:
    Visualizer(GeneticAlgorithmTrace& gatrace) {}

    void show() {}
  };
}

#endif