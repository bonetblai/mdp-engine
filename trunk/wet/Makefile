GCCPATH	=	/opt/sfw/gcc-3.4.1
G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -DNDEBUG
LDFLAGS =	#-R $(GCCPATH)/lib

wet2:	wet2.cc ../common.h
	$(G++) $(CCFLAGS) -o wet2 wet2.cc $(LDFLAGS)

wet:	wet.cc
	$(G++) $(CCFLAGS) -o wet wet.cc $(LDFLAGS)

clean:
	rm -f *~
