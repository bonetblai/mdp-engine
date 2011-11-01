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

#ifndef AO_H
#define AO_H

#include "policy.h"

#include <iostream>
#include <cassert>
#include <limits>
#include <vector>
#include <queue>

//#define DEBUG

namespace Policy {


////////////////////////////////////////////////


template<typename T> struct ao_node_t {
    mutable float value_;
    unsigned depth_;
    unsigned priority_;
    const ao_node_t *parent_;
    std::vector<const ao_node_t*> children_;

    ao_node_t(float value = 0, unsigned depth = 0, unsigned priority = 0, const ao_node_t<T> *parent = 0)
      : value_(value), depth_(depth), priority_(priority), parent_(parent) { }

    virtual ~ao_node_t() { }
    virtual void print(std::ostream &os) const = 0;

    void print_tree(std::ostream &os) const {
        print(os);
        os << std::endl;
        for( size_t i = 0, isz = ao_node_t<T>::children_.size(); i < isz; ++i )
            ao_node_t<T>::children_[i]->print_tree(os);
    }
};

template<typename T> struct state_node_t;
template<typename T> struct action_node_t : public ao_node_t<T> {
    Problem::action_t action_;
    std::vector<float> probability_;

    action_node_t(Problem::action_t action, unsigned depth, unsigned priority, const ao_node_t<T> *parent = 0)
      : ao_node_t<T>(0, depth, priority, parent), action_(action) { }

    virtual ~action_node_t() {
        for( size_t i = 0, isz = ao_node_t<T>::children_.size(); i < isz; ++i )
            delete ao_node_t<T>::children_[i];
    }

    void propagate(const Problem::problem_t<T> &problem) const {
        float value = 0;
        for( size_t i = 0, isz = ao_node_t<T>::children_.size(); i < isz; ++i ) {
            value += probability_[i] * ao_node_t<T>::children_[i]->value_;
        }
        assert(ao_node_t<T>::parent_ != 0);
        const T &s = static_cast<const state_node_t<T>*>(ao_node_t<T>::parent_)->state_;
        ao_node_t<T>::value_ = problem.cost(s, action_) + DISCOUNT * value;
        static_cast<const state_node_t<T>*>(ao_node_t<T>::parent_)->propagate(problem);
    }

    virtual void print(std::ostream &os) const {
        os << std::setw(2 * ao_node_t<T>::depth_) << ""
           << "[action=" << action_
           << ",value=" << ao_node_t<T>::value_
           << ",depth=" << ao_node_t<T>::depth_
           << ",priority=" << ao_node_t<T>::priority_
           << "]";
    }
};

template<typename T> struct state_node_t : public ao_node_t<T> {
    const T state_;
    mutable Problem::action_t best_action_;

    state_node_t(const T &state, unsigned depth, unsigned priority, const ao_node_t<T> *parent = 0)
      : ao_node_t<T>(0, depth, priority, parent),
        state_(state), best_action_(Problem::noop) { }

    virtual ~state_node_t() {
        for( size_t i = 0, isz = ao_node_t<T>::children_.size(); i < isz; ++i )
            delete ao_node_t<T>::children_[i];
    }

    void propagate(const Problem::problem_t<T> &problem) const {
        float value = std::numeric_limits<float>::max();
        for( size_t i = 0, isz = ao_node_t<T>::children_.size(); i < isz; ++i ) {
            float child_value = ao_node_t<T>::children_[i]->value_;
            if( child_value < value ) {
                value = child_value;
                best_action_ = static_cast<const action_node_t<T>*>(ao_node_t<T>::children_[i])->action_;
            }
        }
        ao_node_t<T>::value_ = value;
        if( ao_node_t<T>::parent_ != 0 )
            static_cast<const action_node_t<T>*>(ao_node_t<T>::parent_)->propagate(problem);
    }

    virtual void print(std::ostream &os) const {
        os << std::setw(2 * ao_node_t<T>::depth_) << ""
           << "[state=" << state_
           << ",best_action=" << best_action_
           << ",value=" << ao_node_t<T>::value_
           << ",depth=" << ao_node_t<T>::depth_
           << ",priority=" << ao_node_t<T>::priority_
           << "]";
    }
};

template<typename T> struct min_priority_t {
    bool operator()(const state_node_t<T> *n1, const state_node_t<T> *n2) {
        return n1->priority_ > n2->priority_;
    }
};


////////////////////////////////////////////////


template<typename T> class ao_t : public improvement_t<T> {
  protected:
    unsigned width_;
    unsigned depth_bound_;
    unsigned discrepancy_bound_;
    mutable unsigned num_nodes_;
    mutable state_node_t<T> *root_;
    mutable std::priority_queue<state_node_t<T>*, std::vector<state_node_t<T>*>, min_priority_t<T> > priority_queue_;

  public:
    ao_t(const Problem::problem_t<T> &problem, const policy_t<T> &base_policy, unsigned width, unsigned depth_bound, unsigned discrepancy_bound = std::numeric_limits<unsigned>::max())
      : improvement_t<T>(problem, base_policy),
        width_(width),
        depth_bound_(depth_bound),
        discrepancy_bound_(discrepancy_bound),
        num_nodes_(0),
        root_(0) {
    }
    virtual ~ao_t() { }

    virtual Problem::action_t operator()(const T &s) const {
        // initialize tree and priority queue
        clear();
        root_ = new state_node_t<T>(s, 0, 0);
        priority_queue_.push(root_);

        // expand leaves and propagate values
        for( unsigned i = 0; (i < width_) && !priority_queue_.empty(); ++i ) {
            const state_node_t<T> *node = expand();
            if( node == 0 ) break;
            node->propagate(policy_t<T>::problem_);
        }
        assert((width_ == 0) || (root_ != 0));

        // select best action
        return width_ == 0 ? improvement_t<T>::base_policy_(s) : root_->best_action_;
    }

    void clear() const {
        num_nodes_ = 0;
        if( root_ != 0 ) {
            delete root_;
            root_ = 0;
        }
        while( !priority_queue_.empty() )
            priority_queue_.pop();
    }
    size_t size() const { return num_nodes_; }
    void print_tree(std::ostream &os) const {
        if( root_ == 0 ) {
            os << "(empty)" << std::endl;
        } else {
            //root_->print_tree(os);
            root_->print(os); os << std::endl;
        }
    }

    const state_node_t<T>* expand() const {
        // get best open node
        state_node_t<T> *node = priority_queue_.top();
        priority_queue_.pop();
        //std::cout << "pop " << node << " "; node->print(std::cout); std::cout << std::endl;
        //std::cout << "pri=" << node->priority_ << std::endl;
        assert(node->children_.empty());
        if( node->priority_ > discrepancy_bound_ ) return 0;
        node->children_.reserve(policy_t<T>::problem_.number_actions());

        // compute best action for this node according to base policy
        Problem::action_t best_action = improvement_t<T>::base_policy_(node->state_);

        // expand node
        std::vector<std::pair<T, float> > outcomes;
        for( Problem::action_t a = 0; a < policy_t<T>::problem_.number_actions(); ++a ) {
            if( policy_t<T>::problem_.applicable(node->state_, a) ) {
                //std::cout << "action=" << a << std::endl;
                // create action node for this action
                unsigned priority = node->priority_ + (a == best_action ? 0 : 1);
                action_node_t<T> *a_node = new action_node_t<T>(a, 1 + node->depth_, priority, node);
                node->children_.push_back(a_node);
                ++num_nodes_;

                // generate successor states
                policy_t<T>::problem_.next(node->state_, a, outcomes);
                a_node->children_.reserve(outcomes.size());
                a_node->probability_.reserve(outcomes.size());
                for( size_t i = 0, isz = outcomes.size(); i < isz; ++i ) {
                    const T &state = outcomes[i].first;
                    float prob = outcomes[i].second;
                    state_node_t<T> *s_node = new state_node_t<T>(state, 2 + node->depth_, priority, a_node);
                    a_node->children_.push_back(s_node);
                    a_node->probability_.push_back(prob);
                    ++num_nodes_;

                    // set default value of child using base policy
                    s_node->value_ = evaluate(state);
                    a_node->value_ += prob * s_node->value_;

                    // insert child into priority queue
                    if( s_node->depth_ <= 2*depth_bound_ ) {
                        priority_queue_.push(s_node);
                        //std::cout << "push " << s_node << " "; s_node->print(std::cout); std::cout << std::endl;
                    }
                }

                // set value for new action node
                a_node->value_ = policy_t<T>::problem_.cost(node->state_, a) + DISCOUNT * a_node->value_;
            }
        }
        return node;
    }

    float evaluate(const T &s) const {
        return evaluation(improvement_t<T>::base_policy_, s, 1, depth_bound_);
    }
};

}; // namespace Policy

#undef DEBUG

#endif

