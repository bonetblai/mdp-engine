G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine
OBJS	=	puzzle2.o
TARGET	=	puzzle2

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

puzzle2.o:	puzzle2.cc
puzzle2.o:	../engine/algorithm.h ../engine/dispatcher.h ../engine/hash.h ../engine/heuristic.h
puzzle2.o:	../engine/mcts.h ../engine/parameters.h ../engine/problem.h ../engine/policy.h
puzzle2.o:	../engine/random.h ../engine/rollout.h ../engine/uct.h ../engine/utils.h

