G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -DNDEBUG -I../engine

puzzle2:	puzzle2.o
		$(G++) $(CCFLAGS) -o puzzle2 puzzle2.cc

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

puzzle2:	puzzle2.cc
puzzle2:	../engine/algorithm.h ../engine/hash.h ../engine/heuristic.h ../engine/problem.h ../engine/random.h ../engine/utils.h

