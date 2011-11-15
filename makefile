PROBLEMS =	ctp puzzle race rect sailing tree wet

all:
	for p in $(PROBLEMS); do \
	    (cd $$p; make) \
        done

clean:
	for p in $(PROBLEMS); do \
	    (cd $$p; make clean) \
        done
