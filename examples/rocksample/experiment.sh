#!/bin/bash

if [ "$#" != "9" ]; then
    echo "usage: experiment.sh <dim> <nrocks> <max-antenna-height> <ntrials> <policy> <discretization-parameter> <prune-threshold> <max-expansions> <max-steps>"
    exit 1
fi

dim=$1
nrocks=$2
max_antenna_height=$3
ntrials=$4

shift 4
policy=$1
dp=$2
pt=$3
max_expansions=$4
max_steps=$5

./rocksample --no-colors -r "policy=$policy(determinization=most-likely,stop-criterion=reward,dp=$dp,max-expansions=$max_expansions,random-ties=true,prune-threshold=$pt)" -t $ntrials -s $RANDOM --max-steps $max_steps $dim $dim $nrocks $max_antenna_height

