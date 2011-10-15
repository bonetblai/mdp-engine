G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -DNDEBUG -I../engine

race2:		race2.o parsing.o
		$(G++) $(CCFLAGS) -o race2 race2.o parsing.o

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

race2.o:	race2.cc parsing.h
race2.o:	../engine/algorithm.h ../engine/evaluation.h ../engine/hash.h
race2.o:	../engine/heuristic.h ../engine/parameters.h ../engine/problem.h
race2.o:	../engine/random.h ../engine/utils.h

parsing.o:	parsing.h

