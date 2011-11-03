G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine
OBJS	=	main.o
TARGET	=	tree

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

main.o:		tree.h ../evaluation.h
main.o:		../engine/algorithm.h ../engine/dispatcher.h ../engine/hash.h ../engine/heuristic.h
main.o:		../engine/mcts.h ../engine/parameters.h ../engine/problem.h ../engine/policy.h
main.o:		../engine/random.h ../engine/rollout.h ../engine/uct.h ../engine/utils.h

