G++	=	g++
CCFLAGS	=	-Wall -g -ffloat-store -ffast-math -msse -I../engine -std=c++0x #-DNDEBUG
OBJS	=	main.o
TARGET	=	wumpus

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(G++) $(CCFLAGS) -c $<

main.o:		main.cc base_policy.h wumpus.h agent.h wumpus_belief.h
main.o:		belief.h bin.h ordered_vector.h
main.o:		../evaluation.h
main.o:		../engine/problem.h ../engine/policy.h ../engine/rollout.h

