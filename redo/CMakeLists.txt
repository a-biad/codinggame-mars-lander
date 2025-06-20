cmake_minimum_required(VERSION 3.31)
project(redo)

set(CMAKE_CXX_STANDARD 17)

add_executable(redo main.cpp
        simulation/Pt2f.cpp
        simulation/Pt2f.h
        simulation/Land.cpp
        simulation/Land.h
        simulation/Collision.cpp
        simulation/Collision.h
        simulation/Constants.h
        simulation/ShipControl.cpp
        simulation/ShipControl.h
        simulation/Simulation.cpp
        simulation/Simulation.h
        geneticalgorithm/Random.h
        geneticalgorithm/GeneticAlgorithm.cpp
        geneticalgorithm/GeneticAlgorithm.h
        geneticalgorithm/Chromosome.cpp
        geneticalgorithm/Chromosome.h
        geneticalgorithm/Parameters.h
        geneticalgorithm/SimulationPlayer.cpp
        geneticalgorithm/SimulationPlayer.h
        geneticalgorithm/DoubleBuffer.h
        geneticalgorithm/Util.cpp
        geneticalgorithm/Util.h
        geneticalgorithm/GeneticAlgorithmTrace.cpp
        geneticalgorithm/GeneticAlgorithmTrace.h
        geneticalgorithm/Visualizer.cpp
        geneticalgorithm/Visualizer.h
        simulation/Float.h
        geneticalgorithm/utildraw.cpp
)

target_link_libraries(redo PRIVATE raylib)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_custom_command(
        OUTPUT ${CMAKE_SOURCE_DIR}/export/export.cpp
        DEPENDS redo export-cg.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMAND python export-cg.py
)

add_custom_target(export-cg ALL
        DEPENDS export/export.cpp
)