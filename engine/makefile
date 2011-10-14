puzzle2:
	g++ -Wall -O3 -ffloat-store -ffast-math -msse -DNDEBUG -o puzzle2 puzzle2.cc 

test:	test.cc
	g++ -o test test.cc

test: algorithm.h hash.h heuristic.h problem.h random.h utils.h
puzzle2: puzzle2.cc algorithm.h hash.h heuristic.h problem.h random.h utils.h

