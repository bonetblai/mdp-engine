/*
 *  Copyright (c) 2011-2016 Universidad Simon Bolivar
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
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cassert>
#include <limits>
#include <vector>
#include <math.h>

//#define DEBUG

namespace Online {

extern unsigned g_seed;

namespace Policy {

// Abstract class that defines the interface for action-selection polcicies
template<typename T> class policy_t {
  protected:
    const Problem::problem_t<T> &problem_;
    unsigned seed_;

    mutable unsigned decisions_;

  public:
    policy_t(const Problem::problem_t<T> &problem)
      : problem_(problem), seed_(g_seed), decisions_(0) {
    }
    virtual ~policy_t() { }
    virtual policy_t<T>* clone() const = 0;
    virtual std::string name() const = 0;
    virtual Problem::action_t operator()(const T &s) const = 0;
    virtual void reset_stats() const = 0;
    virtual void print_stats(std::ostream &os) const = 0;
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) = 0;
    unsigned seed() const { return seed_; }
    unsigned decisions() const { return decisions_; }
    const Problem::problem_t<T>& problem() const { return problem_; }
};

// Abstract class for improvement of a base policy
template<typename T> class improvement_t : public policy_t<T> {
  protected:
    const policy_t<T> *base_policy_;

    improvement_t(const Problem::problem_t<T> &problem, const policy_t<T> *base_policy)
      : policy_t<T>(problem), base_policy_(base_policy) {
    }

  public:
    improvement_t(const Problem::problem_t<T> &problem)
      : policy_t<T>(problem), base_policy_(0) {
    }
    virtual ~improvement_t() { }
};

}; // namespace Policy


// Online evaluation
namespace Evaluation {

template<typename T>
inline float evaluation_trial(const Policy::policy_t<T> &policy, const T &s, unsigned max_depth) {
    T state = s;
    size_t steps = 0;
    float cost = 0;
    float discount = 1;
    if( policy.problem().dead_end(state) ) return policy.problem().dead_end_value();
    while( (steps < max_depth) && !policy.problem().terminal(state) ) {
        //std::cout << "evaluation_trial: " << state << std::flush;
        Problem::action_t action = policy(state);
        //std::cout << ", a=" << action << std::endl;
        if( action == Problem::noop ) {
            //std::cout << "no applicable action" << std::endl;
            return cost + policy.problem().dead_end_value();
        }
        assert(policy.problem().applicable(state, action));
        std::pair<T, bool> p = policy.problem().sample(state, action);
        cost += discount * policy.problem().cost(state, action);
        discount *= policy.problem().discount();
        state = p.first;
        ++steps;
        if( policy.problem().dead_end(state) ) {
            return cost + policy.problem().dead_end_value();
        }
    }
    return cost;
}

template<typename T>
inline float evaluation(const Policy::policy_t<T> &policy, const T &s, unsigned number_trials, unsigned max_depth, bool verbose = false) {
    float value = 0;
    if( verbose ) std::cout << "#trials=" << number_trials << ":";
    for( unsigned i = 0; i < number_trials; ++i ) {
        if( verbose ) std::cout << " " << i << std::flush;
        value += evaluation_trial(policy, s, max_depth);
    }
    if( verbose ) std::cout << std::endl;
    return value / number_trials;
}

template<typename T>
inline std::pair<float, float>
  evaluation_with_stdev(const Policy::policy_t<T> &policy,
                        const T &s,
                        unsigned number_trials,
                        unsigned max_depth,
                        bool verbose = false) {
    float sum = 0;
    std::vector<float> values;
    values.reserve(number_trials);
    if( verbose ) std::cout << "#trials=" << number_trials << ":";
    for( unsigned trial = 0; trial < number_trials; ++trial ) {
        if( verbose ) std::cout << " " << trial << std::flush;
        values.push_back(evaluation_trial(policy, s, max_depth));
        sum += values.back();
        if( verbose ) {
            std::cout << "(" << std::setprecision(1) << sum/(1+trial) << ")"
                      << std::flush;
        }
    }
    if( verbose ) std::cout << std::endl;

    // compute average
    float avg = 0;
    for( unsigned i = 0; i < number_trials; ++i ) {
        avg += values[i];
    }
    avg /= number_trials;

    // compute stdev
    float stdev = 0;
    for( unsigned i = 0; i < number_trials; ++i ) {
        stdev += (avg - values[i]) * (avg - values[i]);
    }
    stdev = sqrt(stdev) / (number_trials - 1);
    return std::make_pair(avg, stdev);
}

}; // namespace Evaluation

}; // namespace Online

#undef DEBUG

#endif

