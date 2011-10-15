G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -DNDEBUG -I../engine

puzzle2:	puzzle2.o
		$(G++) $(CCFLAGS) -o puzzle2 puzzle2.o

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

puzzle2.o:	puzzle2.cc
puzzle2.o:	../engine/algorithm.h ../engine/hash.h ../engine/heuristic.h
puzzle2.o:	../engine/parameters.h ../engine/problem.h ../engine/random.h
puzzle2.o:	../engine/utils.h

