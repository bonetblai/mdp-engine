BELIEF_TRACKING =   ~/software/github/belief-tracking/master

ENGINE	=	../../engine
CXX	=	g++
#CCFLAGS	=	-Wall -O3 -ffloat-store -ffast-math -msse -I $(ENGINE) -I $(BELIEF_TRACKING)/engine -DNDEBUG
CCFLAGS	=	-Wall -O3 -g -ffloat-store -ffast-math -msse -I $(ENGINE) -I $(BELIEF_TRACKING)/engine
EXTRA	=	-std=c++11
OBJS	=	main.o
TARGET	=	rocksample
TARGET2	=	rocksample2
OBJS2	=	main2.o

all:	rocksample rocksample2

$(TARGET):	$(OBJS)
		$(CXX) $(CCFLAGS) $(EXTRA) -o $(TARGET) $(OBJS)

$(TARGET2):	$(OBJS2)
		$(CXX) $(CCFLAGS) $(EXTRA) -o $(TARGET2) $(OBJS2)

clean:
		rm -f $(OBJS) $(TARGET) *~ core

.cc.o:
		$(CXX) $(CCFLAGS) $(EXTRA) -c $<

$(OBJS):	rocksample_original.h ../binary-search/bitmap.h
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
$(OBJS):	$(ENGINE)/iw_bel.h
$(OBJS):	$(ENGINE)/iw_bel2.h
$(OBJS):	$(ENGINE)/iw_bel3.h
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

$(OBJS2):	rocksample_original2.h ../binary-search/bitmap.h
$(OBJS2):	$(ENGINE)/algorithm.h
$(OBJS2):	$(ENGINE)/aot.h
$(OBJS2):	$(ENGINE)/aot_gh.h
$(OBJS2):	$(ENGINE)/aot_path.h
$(OBJS2):	$(ENGINE)/base_policies.h
$(OBJS2):	$(ENGINE)/bdd_priority_queue.h
$(OBJS2):	$(ENGINE)/deprecated
$(OBJS2):	$(ENGINE)/dispatcher.h
$(OBJS2):	$(ENGINE)/hash.h
$(OBJS2):	$(ENGINE)/hdp.h
$(OBJS2):	$(ENGINE)/heuristic.h
$(OBJS2):	$(ENGINE)/improved_lao.h
$(OBJS2):	$(ENGINE)/iw_bel.h
$(OBJS2):	$(ENGINE)/iw_bel2.h
$(OBJS2):	$(ENGINE)/iw_bel3.h
$(OBJS2):	$(ENGINE)/ldfs.h
$(OBJS2):	$(ENGINE)/lrtdp.h
$(OBJS2):	$(ENGINE)/makefile
$(OBJS2):	$(ENGINE)/online_rtdp.h
$(OBJS2):	$(ENGINE)/plain_check.h
$(OBJS2):	$(ENGINE)/policy.h
$(OBJS2):	$(ENGINE)/pomdp.h
$(OBJS2):	$(ENGINE)/problem.h
$(OBJS2):	$(ENGINE)/random.h
$(OBJS2):	$(ENGINE)/rollout.h
$(OBJS2):	$(ENGINE)/simple_astar.h
$(OBJS2):	$(ENGINE)/uct.h
$(OBJS2):	$(ENGINE)/utils.h
$(OBJS2):	$(ENGINE)/value_iteration.h

