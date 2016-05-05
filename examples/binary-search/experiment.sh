#!/bin/bash

if [ "$#" != "7" ]; then
    echo "usage: experiment.sh <dim> <ntrials> <policy> <discretization-parameter> <prune-threshold> <max-expansions> <max-steps>"
    exit 1
fi

dim=$1
ntrials=$2

shift 2
policy=$1
dp=$2
pt=$3
max_expansions=$4
max_steps=$5

./bs --no-colors -r "policy=$policy(determinization=most-likely,stop-criterion=reward,dp=$dp,max-expansions=$max_expansions,random-ties=true,prune-threshold=$pt)" -t $ntrials -s $RANDOM --max-steps $max_steps $dim

