CXX	=	g++
#CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I../engine -DNDEBUG
CCFLAGS	=	-Wall -O3 -g -ffloat-store -ffast-math -msse -I../engine
EXTRA	=	-std=c++11
OBJS	=	main.o
TARGET	=	ctp3

$(TARGET):	$(OBJS)
		$(CXX) $(CCFLAGS) $(EXTRA) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(CXX) $(CCFLAGS) $(EXTRA) -c $<

$(OBJS):	ctp3.h graph.h
$(OBJS):	../engine/algorithm.h
$(OBJS):	../engine/aot.h
$(OBJS):	../engine/aot_gh.h
$(OBJS):	../engine/aot_path.h
$(OBJS):	../engine/base_policies.h
$(OBJS):	../engine/bdd_priority_queue.h
$(OBJS):	../engine/deprecated
$(OBJS):	../engine/dispatcher.h
$(OBJS):	../engine/hash.h
$(OBJS):	../engine/hdp.h
$(OBJS):	../engine/heuristic.h
$(OBJS):	../engine/improved_lao.h
$(OBJS):	../engine/ldfs.h
$(OBJS):	../engine/lrtdp.h
$(OBJS):	../engine/makefile
$(OBJS):	../engine/online_rtdp.h
$(OBJS):	../engine/pac.h
$(OBJS):	../engine/plain_check.h
$(OBJS):	../engine/policy.h
$(OBJS):	../engine/problem.h
$(OBJS):	../engine/random.h
$(OBJS):	../engine/rollout.h
$(OBJS):	../engine/simple_astar.h
$(OBJS):	../engine/uct.h
$(OBJS):	../engine/utils.h
$(OBJS):	../engine/value_iteration.h

