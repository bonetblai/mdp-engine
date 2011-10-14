GCCPATH	=	/opt/sfw/gcc-3.4.1
G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -DNDEBUG
LDFLAGS =	#-R $(GCCPATH)/lib

rect2:	rect2.cc ../common.h
	$(G++) $(CCFLAGS) -o rect2 rect2.cc $(LDFLAGS)

rect:	rect.cc
	$(G++) $(CCFLAGS) -o rect rect.cc $(LDFLAGS)

clean:
	rm -f *~
