G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine
OBJS	=	main.o parsing.o
TARGET	=	race

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~

.cc.o:
		$(G++) $(CCFLAGS) -c $<

main.o:		main.cc parsing.h race.h
main.o:		../engine/algorithm.h ../engine/dispatcher.h ../engine/hash.h ../engine/heuristic.h
main.o:		../engine/mcts.h ../engine/parameters.h ../engine/problem.h ../engine/policy.h
main.o:		../engine/random.h ../engine/rollout.h ../engine/uct.h ../engine/utils.h ../engine/ao.h

parsing.o:	parsing.h

