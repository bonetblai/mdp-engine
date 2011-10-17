/*
 *  Copyright (C) 2011 Universidad Simon Bolivar
 * 
 *  Permission is hereby granted to distribute this software for
 *  non-commercial research purposes, provided that this copyright
 *  notice is included with any such distribution.
 *  
 *  THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 *  EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE
 *  SOFTWARE IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU
 *  ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 *  
 *  Blai Bonet, bonet@ldc.usb.ve
 *
 */

#ifndef POLICY_H
#define POLICY_H

#include "problem.h"
#include "heuristic.h"
#include "random.h"

#include <iostream>
#include <cassert>
#include <limits>
#include <vector>

//#define DEBUG

namespace Policy {

template<typename T> class policy_t {
  protected:
    const Problem::problem_t<T> &problem_;

  public:
    policy_t(const Problem::problem_t<T> &problem)
      : problem_(problem) {
    }
    virtual ~policy_t() { }
    virtual Problem::action_t operator()(const T &s) const = 0;
};

template<typename T> class improvement_t : public policy_t<T> {
  protected:
    const policy_t<T> &base_policy_;

  public:
    improvement_t(const Problem::problem_t<T> &problem, const policy_t<T> &base_policy)
      : policy_t<T>(problem), base_policy_(base_policy) {
    }
    virtual ~improvement_t() { }
};

template<typename T> class random_t : public policy_t<T> {
  public:
    random_t(const Problem::problem_t<T> &problem) : policy_t<T>(problem) { }
    virtual ~random_t() { }
    virtual Problem::action_t operator()(const T &s) const {
        return Random::uniform(policy_t<T>::problem_.number_actions());
    }
};

template<typename T> class hash_policy_t : public policy_t<T> {
  protected:
    const Problem::hash_t<T> &hash_;

  public:
    hash_policy_t(const Problem::problem_t<T> &problem, const Problem::hash_t<T> &hash)
      : policy_t<T>(problem), hash_(hash) {
    }
    virtual ~hash_policy_t() { }
    virtual Problem::action_t operator()(const T &s) const {
        std::pair<Problem::action_t, float> p = hash_.bestQValue(s);
        return p.first;
    }
};

template<typename T> class greedy_t : public policy_t<T> {
  protected:
    const Heuristic::heuristic_t<T> &heuristic_;

  public:
    greedy_t(const Problem::problem_t<T> &problem, const Heuristic::heuristic_t<T> &heuristic)
      : policy_t<T>(problem), heuristic_(heuristic) {
    }
    virtual ~greedy_t() { }
    virtual Problem::action_t operator()(const T &s) const {
        std::vector<std::pair<T, float> > outcomes;
        Problem::action_t best_action = Problem::noop;
        float best_value = std::numeric_limits<float>::max();
        for( Problem::action_t a = 0; a < policy_t<T>::problem_.number_actions(); ++a ) {
            float value = 0;
            policy_t<T>::problem_.next(s, a, outcomes);
            for( size_t i = 0, isz = outcomes.size(); i < isz; ++i ) {
                value += outcomes[i].second * heuristic_.value(outcomes[i].first);
            }
            value += policy_t<T>::problem_.cost(s, a);

            if( value < best_value ) {
                best_value = value;
                best_action = a;
            }
        }
        return best_action;
    }
};

// Policy evaluation

template<typename T>
float evaluation_trial(const Problem::problem_t<T> &problem, const Policy::policy_t<T> &policy, const T &s, unsigned max_depth) {
    T state = s;
    size_t steps = 1;
    float cost = 0;
    while( !problem.terminal(state) && (steps <= max_depth) ) {
        Problem::action_t action = policy(state);
        std::pair<T, bool> p = problem.sample(state, action);
        cost += policy_t<T>::problem.cost(state, action);
        state = p.first;
        ++steps;
    }
    return cost;
}

template<typename T>
float evaluation(const Problem::problem_t<T> &problem, const Policy::policy_t<T> &policy, const T &s, unsigned number_trials, unsigned max_depth) {
    float value = 0;
    for( unsigned i = 0; i < number_trials; ++i ) {
        value += evaluation_trial(problem, policy, s, max_depth);
    }
    return value / number_trials;
}

}; // namespace Policy

#undef DEBUG

#endif

