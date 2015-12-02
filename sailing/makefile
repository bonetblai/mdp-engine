CXX	=	g++
#CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine -DNDEBUG
CCFLAGS	=	-Wall -O3 -g -ffloat-store -ffast-math -msse -I../engine
EXTRA	=	-std=c++0x
OBJS	=	main.o
TARGET	=	sailing

$(TARGET):	$(OBJS)
		$(CXX) $(CCFLAGS) $(EXTRA) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(CXX) $(CCFLAGS) $(EXTRA) -c $<

