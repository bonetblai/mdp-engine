/*
 *  Copyright (C) 2006 Universidad Simon Bolivar
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

template<typename T>
float evaluation_trial(const Problem::problem_t<T> &problem, const Policy::policy_t<T> &policy, const T &s, unsigned max_depth) {
    T state = s;
    size_t steps = 1;
    float cost = 0;
    while( !problem.terminal(state) && (steps <= max_depth) ) {
        Problem::action_t action = policy(state);
        std::pair<T, bool> p = problem.sample(state, action);
        cost += problem.cost(state, action);
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
        return 0;
    }
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

template<typename T> class rollout_t : public improvement_t<T> {
  protected:
    unsigned width_;
    unsigned depth_;

  public:
    rollout_t(const Problem::problem_t<T> &problem, const policy_t<T> &base_policy, unsigned width, unsigned depth)
      : improvement_t<T>(problem, base_policy),
        width_(width), depth_(depth) {
    }
    virtual ~rollout_t() { }

    virtual Problem::action_t operator()(const T &s) const {
        std::vector<Problem::action_t> best_actions;
        best_actions.reserve(policy_t<T>::problem_.number_actions());
        float best_value = std::numeric_limits<float>::max();
        for( Problem::action_t a = 0; a < policy_t<T>::problem_.number_actions(); ++a ) {
            std::pair<T, bool> p = policy_t<T>::problem_.sample(s, a);
            float value = evaluate(p.first);
            if( value <= best_value ) {
                if( value < best_value ) {
                    best_value = value;
                    best_actions.clear();
                }
                best_actions.push_back(a);
            }
        }
        assert(!best_actions.empty());
        return best_actions[Random::uniform(best_actions.size())];
    }

    float evaluate(const T &s) const {
        return evaluation(policy_t<T>::problem_, improvement_t<T>::base_policy_, s, width_, depth_);
    }
};

template<typename T> class nested_rollout_t : public policy_t<T> {
  public:
    const policy_t<T> *base_policy_;
    const policy_t<T> *nested_policy_;
    int nesting_level_;

    nested_rollout_t(const Problem::problem_t<T> &problem, const policy_t<T> &base_policy, unsigned width, unsigned depth, int nesting_level)
      : policy_t<T>(problem),
        base_policy_(0), nested_policy_(0), nesting_level_(nesting_level) {
        assert(nesting_level_ >= 0);
        if( nesting_level_ == 0 ) {
            nested_policy_ = &base_policy;
        } else {
            base_policy_ = new nested_rollout_t<T>(problem, base_policy, width, depth, --nesting_level);
            nested_policy_ = new rollout_t<T>(problem, *base_policy_, width, depth);
        }
    }
    virtual ~nested_rollout_t() {
        delete base_policy_;
        if( nesting_level_ > 0 ) delete nested_policy_;
    }
    virtual Problem::action_t operator()(const T &s) const {
        return (*nested_policy_)(s);
    }
};

template<typename T> class uct_t : public improvement_t<T> {
  protected:

  public:
    uct_t(const Problem::problem_t<T> &problem, const policy_t<T> &base_policy)
      : improvement_t<T>(problem, base_policy) {
    }
    virtual ~uct_t() { }
    virtual Problem::action_t operator()(const T &s) const {
        return 0;
    }
};

}; // namespace Policy

#undef DEBUG

#endif

