CXX	=	clang++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine #-DNDEBUG
OBJS	=	main.o
TARGET	=	wet

$(TARGET):	$(OBJS)
		$(CXX) $(CCFLAGS) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(CXX) $(CCFLAGS) -c $<

