GCCPATH	=	/opt/sfw/gcc-3.4.1
G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -DNDEBUG
LDFLAGS =	#-R $(GCCPATH)/lib

puzzle2:	puzzle2.cc ../common.h
	$(G++) $(CCFLAGS) -o puzzle2 puzzle2.cc $(LDFLAGS)

clean:
	rm -f *~
