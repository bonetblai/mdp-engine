G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -std=c++0x #-DNDEBUG
OBJS	=	main.o
TARGET	=	mines

$(TARGET):	$(OBJS)
		$(G++) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(G++) $(CCFLAGS) -c $<

main.o	:	main.cc agent.h mines_belief.h belief.h cell_bin.h ordered_vector.h

