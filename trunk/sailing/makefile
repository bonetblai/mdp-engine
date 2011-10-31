G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine
OBJS	=	sailing.o
TARGET	=	sailing

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

sailing.o:	sailing.cc
sailing.o:	../engine/algorithm.h ../engine/dispatcher.h ../engine/hash.h ../engine/heuristic.h
sailing.o:	../engine/mcts.h ../engine/parameters.h ../engine/problem.h ../engine/policy.h
sailing.o:	../engine/random.h ../engine/rollout.h ../engine/uct.h ../engine/utils.h

