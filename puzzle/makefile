CXX	=	g++
CCFLAGS	=	-Wall -g -ffloat-store -ffast-math -msse -I../engine -DNDEBUG
OBJS	=	main.o
TARGET	=	puzzle

$(TARGET):	$(OBJS)
		$(CXX) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(CXX) $(CCFLAGS) -c $<

