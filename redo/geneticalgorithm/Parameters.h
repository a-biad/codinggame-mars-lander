#pragma once


namespace geneticalgorithm
{
  struct Parameters
  {
    bool trace;
    int seed;

    size_t maxGeneration;
    float mutationP;
    int selectionK;

    size_t numChromosome;
    size_t numElite;
  };

  inline Parameters defaultParameters() {
    Parameters ret;
    ret.trace = true;
    ret.seed = 1234;
    ret.maxGeneration = 30000;
    ret.mutationP = 0.025f;
    ret.selectionK = 10;
    ret.numChromosome = 128;
    ret.numElite = 16;
    return ret;
  }
}