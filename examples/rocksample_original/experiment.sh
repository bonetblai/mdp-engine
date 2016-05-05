#!/bin/bash

if [ "$#" != "8" ]; then
    echo "usage: experiment.sh <dim> <nrocks> <ntrials> <policy> <discretization-parameter> <prune-threshold> <max-expansions> <max-steps>"
    exit 1
fi

dim=$1
nrocks=$2
ntrials=$3

shift 3
policy=$1
dp=$2
pt=$3
max_expansions=$4
max_steps=$5

./rocksample2 --no-colors -r "policy=$policy(determinization=most-likely,stop-criterion=reward,dp=$dp,max-expansions=$max_expansions,random-ties=true,prune-threshold=$pt)" -t $ntrials -s $RANDOM --max-steps $max_steps $dim $dim $nrocks

