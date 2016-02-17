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

#ifndef ROLLOUT_H
#define ROLLOUT_H

#include "policy.h"

#include <iostream>
#include <sstream>
#include <cassert>
#include <limits>
#include <vector>

//#define DEBUG

namespace Online {

namespace Policy {

namespace Rollout {

template<typename T> class nested_rollout_t;

// nested rollout policy
template<typename T> class rollout_t : public improvement_t<T> {
  using policy_t<T>::problem_;
  using improvement_t<T>::base_policy_;
  protected:
    unsigned width_;
    unsigned depth_;
    unsigned nesting_;

    rollout_t(const Problem::problem_t<T> &problem,
              const policy_t<T> *base_policy,
              unsigned width,
              unsigned depth,
              unsigned nesting)
      : improvement_t<T>(problem, base_policy),
        width_(width), depth_(depth), nesting_(nesting) {
    }

  public:
    rollout_t(const Problem::problem_t<T> &problem)
      : improvement_t<T>(problem), width_(0), depth_(0), nesting_(0) {
    }
    virtual ~rollout_t() { }
    virtual policy_t<T>* clone() const {
        return new rollout_t(problem_, base_policy_, width_, depth_, nesting_);
    }
    virtual std::string name() const {
        return std::string("rollout(policy=") + (base_policy_ == 0 ? std::string("null") : base_policy_->name()) +
          std::string(",width=") + std::to_string(width_) +
          std::string(",depth=") + std::to_string(depth_) +
          std::string(",nesting=") + std::to_string(nesting_) + ")";
    }

    virtual Problem::action_t operator()(const T &s) const {
        ++policy_t<T>::decisions_;
        Problem::action_t best_action = Problem::noop;
        float best_value = std::numeric_limits<float>::max();
        for( Problem::action_t a = 0; a < problem_.number_actions(s); ++a ) {
            if( problem_.applicable(s, a) ) {
                float value = 0;
                for( unsigned trial = 0; trial < width_; ++trial ) {
                    std::pair<T, bool> p = problem_.sample(s, a);
                    value += problem_.cost(s, a) + problem_.discount() * evaluate(p.first);
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
    virtual void reset_stats() const {
        problem_.clear_expansions();
        if( base_policy_ != 0 ) base_policy_->reset_stats();
    }
    virtual void print_stats(std::ostream &os) const {
        os << "stats: policy=" << name() << std::endl;
        os << "stats: decisions=" << policy_t<T>::decisions_ << std::endl;
        base_policy_->print_stats(os);
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("width");
        if( it != parameters.end() ) width_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("depth");
        if( it != parameters.end() ) depth_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("nesting");
        if( it != parameters.end() ) depth_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("policy");
        if( it != parameters.end() ) {
            delete base_policy_;
            dispatcher.create_request(problem_, it->first, it->second);
            base_policy_ = dispatcher.fetch_policy(it->second);
        }
#ifdef DEBUG
        std::cout << "debug: rollout(): params:"
                  << " width=" << width_
                  << " depth=" << depth_
                  << " nesting=" << nesting_
                  << " policy=" << (base_policy_ == 0 ? std::string("null") : base_policy_->name())
                  << std::endl;
#endif
    }

    float evaluate(const T &s) const {
        if( base_policy_ == 0 ) {
            std::cout << "error: (base) policy must be specified for rollout() policy!" << std::endl;
            exit(1);
        }
        return Evaluation::evaluation(*base_policy_, s, 1, depth_);
    }

    friend nested_rollout_t<T>;
};

template<typename T> class nested_rollout_t : public improvement_t<T> {
  using policy_t<T>::problem_;
  using improvement_t<T>::base_policy_;
  protected:
    std::vector<const policy_t<T>*> nested_policies_;
    unsigned width_;
    unsigned depth_;
    unsigned nesting_;

    nested_rollout_t(const Problem::problem_t<T> &problem,
                     const policy_t<T> *base_policy,
                     unsigned width,
                     unsigned depth,
                     unsigned nesting)
      : improvement_t<T>(problem, base_policy),
        width_(width), depth_(depth), nesting_(nesting) {
        make_nested_policies();
    }

  public:
    nested_rollout_t(const Problem::problem_t<T> &problem)
      : improvement_t<T>(problem), width_(0), depth_(0), nesting_(0) { }
    virtual ~nested_rollout_t() {
        for( int i = 0; i < int(nested_policies_.size()); ++i )
            delete nested_policies_[i];
    }
    virtual policy_t<T>* clone() const {
        return new nested_rollout_t(problem_, base_policy_, width_, depth_, nesting_);
    }
    virtual std::string name() const {
        return std::string("nested_rollout(policy=") + (base_policy_ == 0 ? std::string("null") : base_policy_->name()) +
          std::string(",width=") + std::to_string(width_) +
          std::string(",depth=") + std::to_string(depth_) +
          std::string(",nesting=") + std::to_string(nesting_) + ")";
    }

    virtual Problem::action_t operator()(const T &s) const {
        assert(!nested_policies_.empty());
        return (*nested_policies_.back())(s);
    }
    virtual void reset_stats() const {
        problem_.clear_expansions();
        nested_policies_.back()->reset_stats();
    }
    virtual void print_stats(std::ostream &os) const {
        os << "stats: policy=" << name() << std::endl;
        for( int i = 0; i < int(nested_policies_.size()); ++i )
            nested_policies_[i]->print_stats(os);
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("width");
        if( it != parameters.end() ) width_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("depth");
        if( it != parameters.end() ) depth_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("nesting");
        if( it != parameters.end() ) nesting_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("policy");
        if( it != parameters.end() ) {
            delete base_policy_;
            dispatcher.create_request(problem_, it->first, it->second);
            base_policy_ = dispatcher.fetch_policy(it->second);
        }
#ifdef DEBUG
        std::cout << "debug: nested-rollout(): params:"
                  << " width=" << width_
                  << " depth=" << depth_
                  << " nesting=" << nesting_
                  << " policy=" << (base_policy_ == 0 ? std::string("null") : base_policy_->name())
                  << std::endl;
#endif
        make_nested_policies();
    }

    float evaluate(const T &s) const {
        assert(!nested_policies_.empty());
        return Evaluation::evaluation(*nested_policies_.bac(), s, 1, depth_);
    }

    void make_nested_policies() {
        if( base_policy_ == 0 ) {
            std::cout << "error: (base) policy must be specified for nested-rollout() policy!" << std::endl;
            exit(1);
        }
        nested_policies_.reserve(1 + nesting_);
        nested_policies_.push_back(base_policy_->clone());
        for( unsigned level = 0; level < nesting_; ++level ) {
            const policy_t<T> *policy = nested_policies_.back();
            policy_t<T> *rollout = new Rollout::rollout_t<T>(problem_, policy, width_, depth_, 1 + level);
            nested_policies_.push_back(rollout);
        }
    }
};

}; // namespace Rollout

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

