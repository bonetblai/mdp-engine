GCCPATH	=	/opt/sfw/gcc-3.4.1
G++	=	g++
CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -DNDEBUG
LDFLAGS =	#-R $(GCCPATH)/lib

tree2:	tree2.cc ../common.h
	$(G++) $(CCFLAGS) -o tree2 tree2.cc $(LDFLAGS)

tree:	tree.cc
	$(G++) $(CCFLAGS) -o tree tree.cc $(LDFLAGS)

clean:
	rm -f *~

