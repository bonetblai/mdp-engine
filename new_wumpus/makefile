G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse #-DNDEBUG
OBJS	=	main.o
TARGET	=	wumpus

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(G++) $(CCFLAGS) -c $<

main.o	:	main.cc agent.h wumpus_belief.h belief.h bin.h ordered_vector.h

