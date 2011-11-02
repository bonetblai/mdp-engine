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

#ifndef AO2_H
#define AO2_H

#include "policy.h"

#include <iostream>
#include <cassert>
#include <limits>
#include <vector>
#include <queue>

//#define DEBUG

namespace Policy {


////////////////////////////////////////////////


template<typename T> struct ao2_node_t {
    mutable float value_;
    unsigned depth_;
    unsigned priority_;
    mutable float delta_;
    const ao2_node_t *parent_;
    std::vector<const ao2_node_t*> children_;

    ao2_node_t(float value = 0, unsigned depth = 0, unsigned priority = 0, const ao2_node_t<T> *parent = 0)
      : value_(value), depth_(depth), priority_(priority), delta_(0), parent_(parent) { }

    virtual ~ao2_node_t() { }
    virtual void print(std::ostream &os) const = 0;

    void print_tree(std::ostream &os) const {
        print(os);
        os << std::endl;
        for( unsigned i = 0, isz = ao2_node_t<T>::children_.size(); i < isz; ++i )
            ao2_node_t<T>::children_[i]->print_tree(os);
    }
};

template<typename T> struct state_node2_t;
template<typename T> struct action_node2_t : public ao2_node_t<T> {
    Problem::action_t action_;
    std::vector<float> probability_;

    action_node2_t(Problem::action_t action, unsigned depth, unsigned priority, const ao2_node_t<T> *parent = 0)
      : ao2_node_t<T>(0, depth, priority, parent), action_(action) { }

    virtual ~action_node2_t() {
        for( unsigned i = 0, isz = ao2_node_t<T>::children_.size(); i < isz; ++i )
            delete ao2_node_t<T>::children_[i];
    }

    void propagate(const Problem::problem_t<T> &problem) const {
        float value = 0;
        for( unsigned i = 0, isz = ao2_node_t<T>::children_.size(); i < isz; ++i ) {
            value += probability_[i] * ao2_node_t<T>::children_[i]->value_;
        }
        assert(ao2_node_t<T>::parent_ != 0);
        const state_node2_t<T> *s_node = static_cast<const state_node2_t<T>*>(ao2_node_t<T>::parent_);
        ao2_node_t<T>::value_ = problem.cost(s_node->state_, action_) + DISCOUNT * value;

        // backprop
        s_node->propagate(problem);
    }

    virtual void print(std::ostream &os) const {
        os << std::setw(2 * ao2_node_t<T>::depth_) << ""
           << "[action=" << action_
           << ",value=" << ao2_node_t<T>::value_
           << ",depth=" << ao2_node_t<T>::depth_
           << ",priority=" << ao2_node_t<T>::priority_
           << ",delta=" << ao2_node_t<T>::delta_
           << "]";
    }
};

template<typename T> struct state_node2_t : public ao2_node_t<T> {
    const T state_;
    mutable unsigned best_action_;

    state_node2_t(const T &state, unsigned depth, unsigned priority, const ao2_node_t<T> *parent = 0)
      : ao2_node_t<T>(0, depth, priority, parent),
        state_(state), best_action_(0) { }

    virtual ~state_node2_t() {
        for( unsigned i = 0, isz = ao2_node_t<T>::children_.size(); i < isz; ++i )
            delete ao2_node_t<T>::children_[i];
    }

    Problem::action_t best_action() const {
        return static_cast<const action_node2_t<T>*>(ao2_node_t<T>::children_[best_action_])->action_;
    }

    void propagate(const Problem::problem_t<T> &problem) const {
        float value = std::numeric_limits<float>::max();
        for( unsigned i = 0, isz = ao2_node_t<T>::children_.size(); i < isz; ++i ) {
            float child_value = ao2_node_t<T>::children_[i]->value_;
            if( child_value < value ) {
                value = child_value;
                best_action_ = i;
            }
        }
        ao2_node_t<T>::value_ = value;
        if( ao2_node_t<T>::parent_ != 0 )
            static_cast<const action_node2_t<T>*>(ao2_node_t<T>::parent_)->propagate(problem);
    }

    virtual void print(std::ostream &os) const {
        os << std::setw(2 * ao2_node_t<T>::depth_) << ""
           << "[state=" << state_
           << ",best_action=" << best_action()
           << ",value=" << ao2_node_t<T>::value_
           << ",depth=" << ao2_node_t<T>::depth_
           << ",priority=" << ao2_node_t<T>::priority_
           << ",delta=" << ao2_node_t<T>::delta_
           << "]";
    }
};

template<typename T> struct min_priority2_t {
    bool operator()(const state_node2_t<T> *n1, const state_node2_t<T> *n2) {
        return n1->priority_ > n2->priority_;
    }
};


////////////////////////////////////////////////


template<typename T> class ao2_t : public improvement_t<T> {
  protected:
    unsigned width_;
    unsigned depth_bound_;
    unsigned discrepancy_bound_;
    mutable unsigned num_nodes_;
    mutable state_node2_t<T> *root_;
    mutable std::priority_queue<state_node2_t<T>*, std::vector<state_node2_t<T>*>, min_priority2_t<T> > priority_queue_;

  public:
    ao2_t(const Problem::problem_t<T> &problem, const policy_t<T> &base_policy, unsigned width, unsigned depth_bound, unsigned discrepancy_bound = std::numeric_limits<unsigned>::max())
      : improvement_t<T>(problem, base_policy),
        width_(width),
        depth_bound_(depth_bound),
        discrepancy_bound_(discrepancy_bound),
        num_nodes_(0),
        root_(0) {
    }
    virtual ~ao2_t() { }

    virtual Problem::action_t operator()(const T &s) const {
        // initialize tree and priority queue
        clear();
        root_ = new state_node2_t<T>(s, 0, 0);
        priority_queue_.push(root_);

        // expand leaves and propagate values
        for( unsigned i = 0; (i < width_) && !priority_queue_.empty(); ++i ) {
            const state_node2_t<T> *node = expand();
            if( node == 0 ) break;
            propagate(node);
            //recompute_delta(root_, std::numeric_limits<float>::max(), true);
        }
        assert((width_ == 0) || (root_ != 0));

        // select best action
        return width_ == 0 ? improvement_t<T>::base_policy_(s) : root_->best_action();
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
    unsigned size() const { return num_nodes_; }
    void print_tree(std::ostream &os) const {
        if( root_ == 0 ) {
            os << "(empty)" << std::endl;
        } else {
            //root_->print_tree(os);
            root_->print(os); os << std::endl;
        }
    }

    const state_node2_t<T>* expand() const {
        // get best open node
        state_node2_t<T> *node = priority_queue_.top();
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
                action_node2_t<T> *a_node = new action_node2_t<T>(a, node->depth_, priority, node);
                node->children_.push_back(a_node);
                ++num_nodes_;

                // generate successor states
                policy_t<T>::problem_.next(node->state_, a, outcomes);
                a_node->children_.reserve(outcomes.size());
                a_node->probability_.reserve(outcomes.size());
                for( unsigned i = 0, isz = outcomes.size(); i < isz; ++i ) {
                    const T &state = outcomes[i].first;
                    float prob = outcomes[i].second;
                    state_node2_t<T> *s_node = new state_node2_t<T>(state, 1 + node->depth_, priority, a_node);
                    a_node->children_.push_back(s_node);
                    a_node->probability_.push_back(prob);
                    ++num_nodes_;

                    // set default value of child using base policy
                    s_node->value_ = evaluate(state, 1 + node->depth_);
                    a_node->value_ += prob * s_node->value_;

                    // insert child into priority queue
                    if( s_node->depth_ <= depth_bound_ ) {
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

    void propagate(const state_node2_t<T> *s_node) const {
        while( s_node != 0 ) {
            float value = std::numeric_limits<float>::max();
            for( unsigned i = 0, isz = s_node->children_.size(); i < isz; ++i ) {
                float child_value = s_node->children_[i]->value_;
                if( child_value < value ) {
                    value = child_value;
                    s_node->best_action_ = static_cast<const action_node2_t<T>*>(s_node->children_[i])->action_;
                }
            }
            s_node->value_ = value;
            if( s_node->parent_ != 0 ) {
                const action_node2_t<T> *a_node = static_cast<const action_node2_t<T>*>(s_node->parent_);
                float value = 0;
                for( unsigned i = 0, isz = a_node->children_.size(); i < isz; ++i ) {
                    value += a_node->probability_[i] * a_node->children_[i]->value_;
                }
                assert(a_node->parent_ != 0);
                s_node = static_cast<const state_node2_t<T>*>(a_node->parent_);
                a_node->value_ = policy_t<T>::problem_.cost(s_node->state_, a_node->action_) + DISCOUNT * value;
            } else {
                s_node = 0;
            }
        }
    }

    void recompute_delta(const state_node2_t<T> *s_node, float delta, bool in_best_policy) const {
        float Delta = std::numeric_limits<float>::max();
        for( unsigned i = 0, isz = s_node->children_.size(); i < isz; ++i ) {
            action_node2_t<T> *a_node = static_cast<const action_node2_t<T>*>(s_node->children_[i]);
            float dif = a_node->value_ - s_node->value_;
            if( dif > 0 ) Delta = Utils::min(Delta, dif);
        }

        for( unsigned i = 0, isz = s_node->children_.size(); i < isz; ++i ) {
            action_node2_t<T> *a_node = static_cast<const action_node2_t<T>*>(s_node->children_[i]);
            float dif = a_node->value_ - s_node->value_;
            if( dif == 0 ) {
                recompute_delta(a_node, delta, Delta, true, in_best_policy);
            } else {
                recompute_delta(a_node, delta, dif, false, in_best_policy);
            }
        }
    }
    void recompute_delta(const action_node2_t<T> *a_node, float delta, float Delta,
                         bool best_action, bool in_best_policy) const {
        for( unsigned i = 0, isz = a_node->children_.size(); i < isz; ++i ) {
            float prob = a_node->probability_[i];
            state_node2_t<T> *s_node = static_cast<const state_node2_t<T>*>(a_node->children_[i]);
            float ndelta = 0;
            if( in_best_policy && best_action ) {
                // delta_{s´} = min { delta_s, Delta } / gamma * P(s'|sa)
                // Delta = min_{ a != a* } T(sa) - T(sa*)
                ndelta = Utils::min(delta, Delta);
            } else if( in_best_policy ) {
                // delta_{s´} = Delta / gamma * P(s'|sa) for a != a*
                // Delta = T(sa) - T(sa*)
                ndelta = Delta;
            } else {
                // delta_{s´} = (delta_s + Delta) / gamma * P(s'|sa) for all a
                // Delta = T(sa) - T(sa*)
                ndelta = Delta;
            }
            ndelta = ndelta / (DISCOUNT * prob);
            s_node->delta_ = ndelta;
            recompute_delta(s_node, ndelta, in_best_policy);
        }
    }

    float evaluate(const T &s, unsigned depth) const {
        return evaluation(improvement_t<T>::base_policy_, s, 1, depth_bound_ - depth);
    }
};

}; // namespace Policy

#undef DEBUG

#endif

