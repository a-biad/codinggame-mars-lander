# codingame-mars-lander

A project to solve the [Mars Lander exercise](https://www.codingame.com/training/medium/mars-lander-episode-2) using a genetic algorithm.

## Build
Depends on Raylib

```bash
cd mars-lander-solver
cmake -B build -DCMAKE_BUILD_TYPE=Release # Or '=Debug`
cmake --build build
```

## Run
```bash
cd mars-lander-solver
# Run from the specified input file (the shape of the land, and the initial ship state)
build/mars-lander-solver -t ../input/input1.txt 
```

