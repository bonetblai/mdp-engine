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

#ifndef ROLLOUT_H
#define ROLLOUT_H

#include "policy.h"

#include <iostream>
#include <cassert>
#include <limits>
#include <vector>

//#define DEBUG

namespace Policy {

template<typename T> class rollout_t : public improvement_t<T> {
  protected:
    unsigned width_;
    unsigned depth_;

  public:
    rollout_t(const policy_t<T> &base_policy, unsigned width, unsigned depth)
      : improvement_t<T>(base_policy),
        width_(width), depth_(depth) {
    }
    virtual ~rollout_t() { }

    virtual Problem::action_t operator()(const T &s) const {
        Problem::action_t best_action = Problem::noop;
        float best_value = std::numeric_limits<float>::max();
        for( Problem::action_t a = 0; a < policy_t<T>::problem_.number_actions(s); ++a ) {
            if( policy_t<T>::problem().applicable(s, a) ) {
                float value = 0;
                for( unsigned trial = 0; trial < width_; ++trial ) {
                    std::pair<T, bool> p = policy_t<T>::problem_.sample(s, a);
                    value += policy_t<T>::problem_.cost(s, a) + DISCOUNT * evaluate(p.first);
                }
                value /= width_;
                if( value < best_value ) {
                    best_value = value;
                    best_action = a;
                }
            }
        }
        assert(best_action != Problem::noop);
        return best_action;
    }

    float evaluate(const T &s) const {
        return evaluation(improvement_t<T>::base_policy_, s, 1, depth_);
    }
};

template<typename T> class nested_rollout_t : public policy_t<T> {
  public:
    const policy_t<T> *base_policy_;
    const policy_t<T> *nested_policy_;
    int nesting_level_;

    nested_rollout_t(const policy_t<T> &base_policy, unsigned width, unsigned depth, int nesting_level)
      : policy_t<T>(base_policy.problem()),
        base_policy_(0), nested_policy_(0), nesting_level_(nesting_level) {
        assert(nesting_level_ >= 0);
        if( nesting_level_ == 0 ) {
            nested_policy_ = &base_policy;
        } else {
            base_policy_ = new nested_rollout_t<T>(base_policy, width, depth, --nesting_level);
            nested_policy_ = new rollout_t<T>(*base_policy_, width, depth);
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

}; // namespace Policy

#undef DEBUG

#endif

