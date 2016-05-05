#!/bin/bash

if [ "$#" != "1" ]; then
    echo "usage: experiment.sh <filename>"
    exit 1
fi

filename=$1

#echo binary-search iw-bel2
#(cd binary-search; ./experiment.sh 550 10 iw-bel2 5 2 250 300 >> results/$filename)
#(cd binary-search; ./experiment.sh 550 10 iw-bel2 5 3 250 300 >> results/$filename)

#echo rocksample iw-bel2
#(cd rocksample; ./experiment.sh 10 10 1 10 iw-bel2 5 2 250 300 >> results/$filename)
#(cd rocksample; ./experiment.sh 10 10 1 10 iw-bel2 5 3 250 300 >> results/$filename)

echo rocksample_original iw-bel2
(cd rocksample_original; ./experiment.sh 10 10 10 iw-bel2 5 2 350 300 >> results/$filename)
(cd rocksample_original; ./experiment.sh 10 10 10 iw-bel2 5 3 350 300 >> results/$filename)

