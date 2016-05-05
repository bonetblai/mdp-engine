#!/bin/bash

echo binary-search iw-bel2
(cd binary-search; ./experiment.sh 80 10 iw-bel2 5 1 250 300 > results.txt)
(cd binary-search; ./experiment.sh 80 10 iw-bel2 5 2 250 300 >> results.txt)
echo binary-search iw-bel3
(cd binary-search; ./experiment.sh 80 10 iw-bel3 5 1 250 300 >> results.txt)
(cd binary-search; ./experiment.sh 80 10 iw-bel3 5 2 250 300 >> results.txt)

echo rocksample iw-bel2
(cd rocksample; ./experiment.sh 8 6 1 10 iw-bel2 5 1 250 300 > results.txt)
(cd rocksample; ./experiment.sh 8 6 1 10 iw-bel2 5 2 250 300 >> results.txt)
echo rocksample iw-bel3
(cd rocksample; ./experiment.sh 8 6 1 10 iw-bel2 3 1 250 300 >> results.txt)
(cd rocksample; ./experiment.sh 8 6 1 10 iw-bel2 3 2 250 300 >> results.txt)

echo rocksample_original iw-bel2
(cd rocksample_original; ./experiment.sh 8 6 10 iw-bel2 5 1 250 300 > results.txt)
(cd rocksample_original; ./experiment.sh 8 6 10 iw-bel2 5 2 250 300 >> results.txt)
echo rocksample_original iw-bel3
(cd rocksample_original; ./experiment.sh 8 6 10 iw-bel2 3 1 250 300 >> results.txt)
(cd rocksample_original; ./experiment.sh 8 6 10 iw-bel2 3 2 250 300 >> results.txt)

