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

    mutable float setup_time_;
    mutable float base_policy_time_;
    mutable float heuristic_time_;
    mutable unsigned decisions_;

  public:
    policy_t(const Problem::problem_t<T> &problem)
      : problem_(problem),
        seed_(g_seed),
        setup_time_(0),
        base_policy_time_(0),
        heuristic_time_(0),
        decisions_(0) {
    }
    virtual ~policy_t() { }
    virtual policy_t<T>* clone() const = 0;
    virtual std::string name() const = 0;
    virtual Problem::action_t operator()(const T &s) const = 0;
    virtual void reset_stats() const = 0;
    virtual void print_other_stats(std::ostream &os, int indent) const = 0;
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) = 0;

    typedef enum { No, Yes, Optional } usage_t;
    virtual usage_t uses_base_policy() const = 0;
    virtual usage_t uses_heuristic() const = 0;
    virtual usage_t uses_algorithm() const = 0;

    unsigned seed() const { return seed_; }
    float setup_time() const { return setup_time_; }
    float base_policy_time() const { return base_policy_time_; }
    float heuristic_time() const { return heuristic_time_; }
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
inline std::pair<size_t, float> evaluation_trial(const Policy::policy_t<T> &policy, const T &s, unsigned max_steps) {
    T state = s;
    size_t steps = 0;
    float cost = 0;
    float discount = 1;
    if( policy.problem().dead_end(state) ) {
        return std::make_pair(0, policy.problem().dead_end_value());
    } else {
        while( (steps < max_steps) && !policy.problem().terminal(state) ) {
            //std::cout << "evaluation_trial: " << state << std::flush;
            Problem::action_t action = policy(state);
            //std::cout << ", a=" << action << std::endl;
            if( action == Problem::noop ) {
                std::cout << "error: policy returned non-applicable action '" << policy.problem().action_name(action) << "'" << std::endl;
                return std::make_pair(steps, cost + policy.problem().dead_end_value());
            }
            //std::cout << " BEFORE APPLICATION=" << state << std::endl;
            assert(policy.problem().applicable(state, action));
            std::pair<T, bool> p = policy.problem().sample(state, action);
            cost += discount * policy.problem().cost(state, action);
            discount *= policy.problem().discount();
            state = p.first;
            //std::cout << "SAMPLED APPLICATION=" << state << std::endl;
            ++steps;
            if( policy.problem().dead_end(state) ) {
                std::cout << "warning: dead-end state reached" << std::endl;
                return std::make_pair(steps, cost + policy.problem().dead_end_value());
            }
        }
        return std::make_pair(steps, cost);
    }
}

template<typename T>
inline float evaluation(const Policy::policy_t<T> &policy, const T &s, unsigned number_trials, unsigned max_steps, bool verbose = false) {
    size_t steps = 0;
    float value = 0;
    if( verbose ) std::cout << "#trials=" << number_trials << ":";
    for( unsigned i = 0; i < number_trials; ++i ) {
        if( verbose ) std::cout << " " << i << std::flush;
        std::pair<size_t, float> p = evaluation_trial(policy, s, max_steps);
        steps += p.first;
        value += p.second;
        if( verbose ) std::cout << " (" << p.first << " " << std::setprecision(1) << p.second << ")" << std::flush;
    }
    if( verbose ) std::cout << std::endl;
    return value / number_trials;
}

template<typename T>
inline std::pair<std::pair<float, float>, std::pair<float, float> >
  evaluation_with_stdev(const Policy::policy_t<T> &policy,
                        unsigned number_trials,
                        unsigned max_steps,
                        bool verbose = false) {
    size_t steps_sum = 0;
    float cost_sum = 0;
    std::vector<size_t> steps;
    std::vector<float> values;
    values.reserve(number_trials);
    if( verbose ) std::cout << "#trials=" << number_trials << ":";
    for( unsigned trial = 0; trial < number_trials; ++trial ) {
        if( verbose ) std::cout << " " << trial << std::flush;
        std::pair<size_t, float> p = evaluation_trial(policy, policy.problem().init(), max_steps);
        steps.push_back(p.first);
        steps_sum += p.first;
        values.push_back(p.second);
        cost_sum += p.second;
        if( verbose ) {
            std::cout << " (" << p.first << " " << std::setprecision(1) << p.second << " " << std::setprecision(1) << cost_sum / (1 + trial) << ")" << std::flush;
        }
    }
    if( verbose ) std::cout << std::endl;

    // compute average per trial
    float steps_avg = 0;
    float values_avg = 0;
    for( unsigned i = 0; i < number_trials; ++i ) {
        steps_avg += steps[i];
        values_avg += values[i];
    }
    steps_avg /= number_trials;
    values_avg /= number_trials;

    // compute stdev per trial
    float steps_stdev = 0;
    float values_stdev = 0;
    for( unsigned i = 0; i < number_trials; ++i ) {
        steps_stdev += (steps_avg - steps[i]) * (steps_avg - steps[i]);
        values_stdev += (values_avg - values[i]) * (values_avg - values[i]);
    }
    steps_stdev = sqrt(steps_stdev) / (number_trials - 1);
    values_stdev = sqrt(values_stdev) / (number_trials - 1);
    return std::make_pair(std::make_pair(steps_avg, steps_stdev), std::make_pair(values_avg, values_stdev));
}

}; // namespace Evaluation

}; // namespace Online

#undef DEBUG

#endif

