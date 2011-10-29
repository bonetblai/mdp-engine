G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine
OBJS	=	rect2.o
TARGET	=	rect2

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

rect2.o:	rect2.cc
rect2.o:	../engine/algorithm.h ../engine/dispatcher.h ../engine/hash.h ../engine/heuristic.h
rect2.o:	../engine/mcts.h ../engine/parameters.h ../engine/problem.h ../engine/policy.h
rect2.o:	../engine/random.h ../engine/rollout.h ../engine/uct.h ../engine/utils.h

