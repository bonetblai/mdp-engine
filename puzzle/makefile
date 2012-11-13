CXX	=	g++
CCFLAGS	=	-Wall -g -ffloat-store -ffast-math -msse -I../engine -DNDEBUG
EXTRA	=	-std=c++0x
OBJS	=	main.o
TARGET	=	puzzle

$(TARGET):	$(OBJS)
		$(CXX) $(CCFLAGS) $(EXTRA) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(CXX) $(CCFLAGS) $(EXTRA) -c $<

