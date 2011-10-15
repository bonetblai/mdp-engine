G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -DNDEBUG -I../engine

race2:		race2.o parsing.o
		$(G++) $(CCFLAGS) -o race2 race2.o parsing.o

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

race2.o:	race2.cc parsing.h
race2.o:	../engine/algorithm.h ../engine/hash.h ../engine/heuristic.h
race2.o:	../engine/parameters.h ../engine/problem.h ../engine/random.h
race2.o:	../engine/utils.h

parsing.o:	parsing.h

