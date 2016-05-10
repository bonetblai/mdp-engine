BELIEF_TRACKING =   ~/software/github/belief-tracking/master

ENGINE	=	../../engine
CXX	=	g++
#CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I $(ENGINE) -I $(BELIEF_TRACKING)/engine -DNDEBUG
CCFLAGS	=	-Wall -O3 -g -ffloat-store -ffast-math -msse -I $(ENGINE) -I $(BELIEF_TRACKING)/engine
EXTRA	=	-std=c++11
OBJS	=	main.o
TARGET	=	rocksample

$(TARGET):	$(OBJS)
		$(CXX) $(CCFLAGS) $(EXTRA) -o $(TARGET) $(OBJS)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(CXX) $(CCFLAGS) $(EXTRA) -c $<

$(OBJS):	rocksample.h ../binary-search/bitmap.h
$(OBJS):	$(ENGINE)/algorithm.h
$(OBJS):	$(ENGINE)/aot.h
$(OBJS):	$(ENGINE)/aot_gh.h
$(OBJS):	$(ENGINE)/aot_path.h
$(OBJS):	$(ENGINE)/base_policies.h
$(OBJS):	$(ENGINE)/bdd_priority_queue.h
$(OBJS):	$(ENGINE)/deprecated
$(OBJS):	$(ENGINE)/dispatcher.h
$(OBJS):	$(ENGINE)/hash.h
$(OBJS):	$(ENGINE)/hdp.h
$(OBJS):	$(ENGINE)/heuristic.h
$(OBJS):	$(ENGINE)/improved_lao.h
$(OBJS):	$(ENGINE)/iw_base.h
$(OBJS):	$(ENGINE)/iw_bel.h
$(OBJS):	$(ENGINE)/iw_bel2.h
$(OBJS):	$(ENGINE)/ldfs.h
$(OBJS):	$(ENGINE)/lrtdp.h
$(OBJS):	$(ENGINE)/makefile
$(OBJS):	$(ENGINE)/online_rtdp.h
$(OBJS):	$(ENGINE)/plain_check.h
$(OBJS):	$(ENGINE)/policy.h
$(OBJS):	$(ENGINE)/pomdp.h
$(OBJS):	$(ENGINE)/problem.h
$(OBJS):	$(ENGINE)/random.h
$(OBJS):	$(ENGINE)/rollout.h
$(OBJS):	$(ENGINE)/simple_astar.h
$(OBJS):	$(ENGINE)/uct.h
$(OBJS):	$(ENGINE)/utils.h
$(OBJS):	$(ENGINE)/value_iteration.h

