GCCPATH	=	/opt/sfw/gcc-3.4.1
G++	=	g++
CCFLAGS	=	-Wall -ffloat-store -ffast-math -msse -O3 -DNDEBUG
LDFLAGS =	#-R $(GCCPATH)/lib

race2:	race2.o parsing.o ../common.h
	$(G++) $(CCFLAGS) -o race2 race2.o parsing.o $(LDFLAGS)

race:	race.o parsing.o
	$(G++) $(CCFLAGS) -o race race.o parsing.o $(LDFLAGS)

.cc.o:
	$(G++) $(CCFLAGS) -c $<

clean:
	rm -fr *~

race2.o:	race2.cc ../common.h

