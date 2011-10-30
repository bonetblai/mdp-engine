G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine
OBJS	=	tree2.o
TARGET	=	tree2

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

tree2.o:	tree2.cc
tree2.o:	../engine/algorithm.h ../engine/dispatcher.h ../engine/hash.h ../engine/heuristic.h
tree2.o:	../engine/mcts.h ../engine/parameters.h ../engine/problem.h ../engine/policy.h
tree2.o:	../engine/random.h ../engine/rollout.h ../engine/uct.h ../engine/utils.h

