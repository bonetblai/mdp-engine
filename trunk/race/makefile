G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine
OBJS	=	race2.o parsing.o
TARGET	=	race2

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

race2.o:	race2.cc parsing.h
race2.o:	../engine/algorithm.h ../engine/dispatcher.h ../engine/hash.h ../engine/heuristic.h
race2.o:	../engine/mcts.h ../engine/parameters.h ../engine/problem.h ../engine/policy.h
race2.o:	../engine/random.h ../engine/rollout.h ../engine/uct.h ../engine/utils.h

parsing.o:	parsing.h

