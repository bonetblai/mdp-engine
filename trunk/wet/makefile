G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine
OBJS	=	wet2.o
TARGET	=	wet2

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

wet2.o:		wet2.cc
wet2.o:		../engine/algorithm.h ../engine/dispatcher.h ../engine/hash.h ../engine/heuristic.h
wet2.o:		../engine/mcts.h ../engine/parameters.h ../engine/problem.h ../engine/policy.h
wet2.o:		../engine/random.h ../engine/rollout.h ../engine/uct.h ../engine/utils.h

