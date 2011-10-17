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

#ifndef UCT_H
#define UCT_H

#include "policy.h"

#include <iostream>
#include <cassert>
#include <limits>
#include <vector>

//#define DEBUG

namespace Policy {

#if 0

template<typename T> struct node_t {
    const T &state_;
    float value_;
    const node_t *parent_;
    int visits_;
    std::vector<node_t*> children_;
    std::vector<int> counts_;
    node_t(const T &s, int num_actions)
      : state_(s), value_(0), parent_(0), visits_(0),
        children_(num_actions, 0),
        counts_(num_actions, 0) {
    } 
    ~node_t() { }

    float uct_trial(const Problem::problem_t<T> &problem, float C) {
        float best_qvalue = std::numeric_limits<float>::max();
        Problem::action_t best_action = Problem::noop;

        // calculate best child
        float log_n_s = logf(visits_);
        for( int a = 0; a < problem.number_actions(); ++a ) {
            float qv = 0;
            if( children_[a] != 0 ) {
                float bonus = C * sqrtf(log_ns / counts_[a]);
                qv = children_[a]->value_ / visits_ + bonus;
            } else {
                qv = problem.cost(state_, a);
            }

            // select best child
            if( qv < qbest_value ) {
                best_action = a;
                best_qvalue = value;
            }
        }

        // recurse on best child
        if( children_[best_action] != 0 ) {
            float leaf_value = children_[best_action]->uct_trial(problem, C);
            leaf_value += problem.cost(state_, best_action);
            value_ += leaf_value; // update stored value
            ++counts_[best_action]; // increase number of times action is pulled
        } else {
            // create a new node initilizing its value to a single sample
            node_t<T> *node = new node_t<T>(problem.num_actions());
            
        }

        // increase number of visits
        ++visits_;
    }
};
#endif

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

