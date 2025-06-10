main2: main2.cpp GeneticAlgorithm.hpp CmdArgs.hpp utildraw.hpp
	clang++ main2.cpp -o main2 -lraylib -ljsoncpp -lboost_program_options  --std=c++17 -O3

run: main2
	./main2 --visualize < input/input1.txt

mainCG.cpp: main2.cpp GeneticAlgorithm.hpp
	cat GeneticAlgorithm.hpp main2.cpp > mainCG.cpp
