/*
 *  Copyright (C) 2015 Universidad Simon Bolivar
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

#ifndef PAC_H
#define PAC_H

#include "policy.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <queue>
#include <vector>
#include <math.h>

//#define DEBUG

namespace Online {

namespace Policy {

namespace PAC {

////////////////////////////////////////////////
//
// Nodes
//

template<typename T> struct node_t {
    const T *state_;
    unsigned char depth_;

    float lower_bound_;
    float upper_bound_;
    bool pruned_;

    node_t *parent_;
    std::vector<node_t*> children_;

    node_t(const T &state, unsigned char depth)
      : state_(new T(state)), depth_(depth),
        lower_bound_(0), upper_bound_(0), pruned_(false),
        parent_(0) {
    }
    virtual ~node_t() { delete state_; }

    float gap() const { return upper_bound_ - lower_bound_; }
    virtual void print(std::ostream &os) const = 0;
};

template<typename T> struct action_node_t : public node_t<T> {
    using node_t<T>::state_;
    using node_t<T>::depth_;
    using node_t<T>::children_;

    Problem::action_t action_;
    std::vector<float> prob_;

    action_node_t(const T &state, unsigned char depth, Problem::action_t action)
      : node_t<T>(state, depth), action_(action) {
    }
    virtual ~action_node_t() { }

    virtual void print(std::ostream &os) const {
        os << "[s=" << *state_
           << ",d=" << int(depth_)
           << ",a=" << action_
           << "]";
    }
};

template<typename T> struct state_node_t : public node_t<T> {
    using node_t<T>::state_;
    using node_t<T>::depth_;
    using node_t<T>::children_;

    float merit_;

    state_node_t(const T &state, unsigned char depth) : node_t<T>(state, depth), merit_(0) { }
    virtual ~state_node_t() { }

    Problem::action_t select_action(bool random_ties, bool use_upper_bound = true) const {
        assert(!children_.empty());
        float best_value = use_upper_bound ? children_[0]->upper_bound_ : children_[0]->lower_bound_;
        std::vector<Problem::action_t> best_actions;
        best_actions.reserve(random_ties ? children_.size() : 1);
        best_actions.push_back(static_cast<const action_node_t<T>*>(children_[0])->action_);

        for( int i = 1; i < int(children_.size()); ++i ) {
            const action_node_t<T> *a_node = static_cast<const action_node_t<T>*>(children_[i]);
            if( (use_upper_bound && (a_node->upper_bound_ <= best_value)) || (!use_upper_bound && (a_node->lower_bound_ <= best_value)) ) {
                if( (use_upper_bound && (a_node->upper_bound_ < best_value)) || (!use_upper_bound && (a_node->lower_bound_ < best_value)) )
                    best_actions.clear();
                best_value = use_upper_bound ? a_node->upper_bound_ : a_node->lower_bound_;
                best_actions.push_back(a_node->action_);
            }
        }

        assert(!best_actions.empty());
        return best_actions[Random::uniform(best_actions.size())];
    }

    virtual void print(std::ostream &os) const {
        os << "[s=" << *state_
           << ",d=" << int(depth_)
           << "]";
    }
};

template<typename T> struct state_node_comparator_t {
    state_node_comparator_t() { }
    bool operator()(const state_node_t<T> *n1, const state_node_t<T> *n2) const {
        return n1->merit_ > n2->merit_;
    }
};

template<typename T> inline std::ostream& operator<<(std::ostream &os, const node_t<T> &node) {
    node.print(os);
    return os;
}

////////////////////////////////////////////////
//
// Policy
//

template<typename T> class pac_tree_t : public improvement_t<T> {
  using policy_t<T>::problem;

  protected:
    unsigned width_;
    unsigned horizon_;
    float parameter_;
    bool random_ties_;

    // parameters
    mutable float delta_;
    mutable float epsilon_;
    mutable unsigned max_num_samples_;
    mutable const Heuristic::heuristic_t<T> *heuristic_;

    // computed parameters
    mutable float gamma_;
    mutable std::vector<float> solved_threshold_;
    mutable std::vector<float> num_samples_;

  public:
    pac_tree_t(const policy_t<T> &base_policy,
               unsigned width,
               unsigned horizon,
               float parameter,
               bool random_ties)
      : improvement_t<T>(base_policy),
        width_(width),
        horizon_(horizon),
        parameter_(parameter),
        random_ties_(random_ties) {
        std::stringstream name_stream;
        name_stream << "pac-tree("
                    << "width=" << width_
                    << ",horizon=" << horizon_
                    << ",par=" << parameter_
                    << ",random-ties=" << (random_ties_ ? "true" : "false")
                    << ")";
        policy_t<T>::set_name(name_stream.str());
std::cout << "HOLA: " << name_stream.str() << std::endl;
    }
    virtual ~pac_tree_t() { }

    void set_parameters(float epsilon,
                        float delta,
                        unsigned max_num_samples,
                        const Heuristic::heuristic_t<T> *heuristic) const {
        epsilon_ = epsilon;
        delta_ = delta;
        max_num_samples_ = max_num_samples;
        heuristic_ = heuristic;

        gamma_ = problem().discount();

        // threshold = epsilon / 2 * gamma^d
//std::cout << "HOLA: thresholds:";
        solved_threshold_ = std::vector<float>(1 + horizon_, 0);
        for( int d = 0; d <= horizon_; ++d ) {
            float threshold = epsilon_ / (2 * powf(gamma_, d));
            solved_threshold_[d] = threshold;
//std::cout << " " << threshold;
        }
//std::cout << std::endl;

        // l(s,d) = 4 gamma^(2d) C_max (d ln b + d ln 2 - ln delta) / (1 - gamma) epsilon^2
//std::cout << "HOLA: num-samples:";
        num_samples_ = std::vector<float>(1 + horizon_, 0);
        for( int d = 0; d <= horizon_; ++d ) {
            float lsd = 4 * powf(gamma_ * gamma_, d) * problem().max_absolute_cost();
//std::cout << " [" << lsd;
            lsd *= d * log(2 * problem().max_combined_branching()) - log(delta_);
//std::cout << " " << lsd;
            lsd /= (1 - gamma_) * epsilon_ * epsilon_;
//std::cout << " " << lsd << "]";
            num_samples_[d] = std::min<float>(lsd, max_num_samples_);
        }
//std::cout << std::endl;
    }

    virtual Problem::action_t operator()(const T &s) const {
//std::cout << "HOLA: X0" << std::endl;
        std::priority_queue<state_node_t<T>*, std::vector<state_node_t<T>*>, state_node_comparator_t<T> > heap;
        std::list<state_node_t<T>*> leaf_nodes;

        ++policy_t<T>::decisions_;

//std::cout << "HOLA: X1" << std::endl;
        state_node_t<T> *root = new state_node_t<T>(s, 0);
//std::cout << "HOLA: X2" << std::endl;
        compute_bounds(*root);
//std::cout << "HOLA: X3" << std::endl;

        Problem::action_t action = 0;
        if( solved(*root) ) {
//std::cout << "HOLA: X4" << std::endl;
            delete root;
            action = improvement_t<T>::base_policy_(s);
//std::cout << "HOLA: X5" << std::endl;
        } else {
//std::cout << "HOLA: X6" << std::endl;
            compute_merit(*root);
//std::cout << "HOLA: X7" << std::endl;
            heap.push(root);
//std::cout << "HOLA: X8: width=" << width_ << std::endl;

            for( int i = 0; !heap.empty() && (i < width_); ++i ) {
//std::cout << "HOLA: heapsz=" << heap.size() << std::endl;
                state_node_t<T> *node = heap.top();
                heap.pop();
//std::cout << "HOLA: X9: ptr=" << node << ", node=" << *node << ", lower=" << node->lower_bound_ << ", upper=" << node->upper_bound_ << std::endl;

                // node must be active leaf
                assert(!solved(*node));
                assert(!node->pruned_);
                assert(node->children_.empty());

                // expand leaf node
//std::cout << "HOLA: X10" << std::endl;
                expand(*node, leaf_nodes);
//std::cout << "HOLA: X11" << std::endl;
                propagate_values(*node);
//std::cout << "HOLA: X12" << std::endl;

                if( heap.empty() ) {
                    std::cout << "size of list of leaf nodes = " << leaf_nodes.size() << std::endl;
                    typename std::list<state_node_t<T>*>::iterator it = leaf_nodes.begin();
                    while( it != leaf_nodes.end() ) {
                        if( !solved(**it) && !(*it)->pruned_ ) {
                            compute_merit(**it);
//std::cout << "HOLA: X13: ptr=" << *it << ", node=" << **it << ", lower=" << (*it)->lower_bound_ << ", upper=" << (*it)->upper_bound_ << ", merit=" << (*it)->merit_ << std::endl;
                            ++it;
                        } else {
                            leaf_nodes.erase(it);
                        }
                    }
          
                    std::cout << "allocating heap of size = " << leaf_nodes.size() << std::endl;
                    heap = std::priority_queue<state_node_t<T>*, std::vector<state_node_t<T>*>, state_node_comparator_t<T> >(leaf_nodes.begin(), leaf_nodes.end());
                    leaf_nodes.clear();
                }
            }
std::cout << "HOLA: X100" << std::endl;

            action = root->select_action(random_ties_);
            delete_tree(root);
//std::cout << "HOLA: X7" << std::endl;
        }
//std::cout << "HOLA: X8" << std::endl;
        assert(problem().applicable(s, action));
//std::cout << "HOLA: X9" << std::endl;
        return action;
    }
    virtual const policy_t<T>* clone() const {
        return new pac_tree_t(improvement_t<T>::base_policy_, width_, horizon_, parameter_, random_ties_);
    }
    virtual void print_stats(std::ostream &os) const {
        os << "stats: policy=" << policy_t<T>::name() << std::endl;
        os << "stats: decisions=" << policy_t<T>::decisions_ << std::endl;
        improvement_t<T>::base_policy_.print_stats(os);
    }

    void compute_bounds(state_node_t<T> &node) const {
        node.lower_bound_ = heuristic_->value(*node.state_); // XXXX: should use (s, depth)
        node.upper_bound_ = evaluate(node);
//std::cout << "HOLA: ptr=" << &node << ", node=" << node << ", lower=" << node.lower_bound_ << ", upper=" << node.upper_bound_ << std::endl;
    }

    bool solved(state_node_t<T> &node) const {
        return node.gap() < solved_threshold_[node.depth_]; 
    }

    // assuming the upper bound decreases by gap / 4, compute sum of gap reductions on ancestor nodes
    void compute_merit(state_node_t<T> &node) const { }

    void expand(state_node_t<T> &node, std::list<state_node_t<T>*> &leaf_nodes) const {
        assert(node.children_.empty());
        int nactions = problem().number_actions(*node.state_);
        std::vector<node_t<T>*> children;
        children.reserve(nactions);
        for( Problem::action_t a = 0; a < nactions; ++a ) {
            if( problem().applicable(*node.state_, a) ) {
                action_node_t<T> *a_node = new action_node_t<T>(*node.state_, node.depth_, a);
                std::vector<std::pair<T, float> > outcomes;
                problem().next(*node.state_, a, outcomes);

                a_node->prob_ = std::vector<float>(outcomes.size(), 0);
                a_node->children_ = std::vector<node_t<T>*>(outcomes.size(), 0);

                for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
                    const T &state = outcomes[i].first;
                    float prob = outcomes[i].second;

                    state_node_t<T> *s_node = new state_node_t<T>(state, 1 + node.depth_);
                    compute_bounds(*s_node);
                    leaf_nodes.push_back(s_node);

                    a_node->children_[i] = s_node;
                    a_node->prob_[i] = prob;
                }

                update_bounds(*a_node);
                children.push_back(a_node);
            }
        }
        node.children_ = std::move(children);
        // we don't update bounds here. It is done in propagate_values()
    }

    void update_bounds(state_node_t<T> &node) const {
        assert(!node.children_.empty());
        node.lower_bound_ = node.children_[0]->lower_bound_;
        node.upper_bound_ = node.children_[0]->upper_bound_;
        for( int i = 1; i < int(node.children_.size()); ++i ) {
            const action_node_t<T> &a_node = *static_cast<const action_node_t<T>*>(node.children_[i]);
            node.lower_bound_ = std::min(node.lower_bound_, a_node.lower_bound_);
            node.upper_bound_ = std::min(node.upper_bound_, a_node.upper_bound_);
        }

        // mark nodes (subtrees) as pruned
        for( int i = 0; i < int(node.children_.size()); ++i ) {
            action_node_t<T> &a_node = *static_cast<action_node_t<T>*>(node.children_[i]);
            if( (node.upper_bound_ < a_node.lower_bound_) && !a_node.pruned_ )
                mark_node_as_pruned(a_node);
        }
    }
    void update_bounds(action_node_t<T> &a_node) const {
        assert(!a_node.children_.empty());
        assert(a_node.children_.size() == a_node.prob_.size());
        a_node.lower_bound_ = 0;
        a_node.upper_bound_ = 0;
        for( int i = 0; i < int(a_node.children_.size()); ++i ) {
            const node_t<T> &node = *a_node.children_[i];
            float prob = a_node.prob_[i];
            a_node.lower_bound_ += prob * node.lower_bound_;
            a_node.upper_bound_ += prob * node.upper_bound_;
        }
        a_node.lower_bound_ = problem().cost(*a_node.state_, a_node.action_) + gamma_ * a_node.lower_bound_;
        a_node.upper_bound_ = problem().cost(*a_node.state_, a_node.action_) + gamma_ * a_node.upper_bound_;
    }

    void mark_node_as_pruned(state_node_t<T> &node) const {
        std::cout << "pruning node"; node.print(std::cout); std::cout << std::endl;
        for( int i = 0; i < int(node.children_.size()); ++i ) {
            action_node_t<T> &a_node = *static_cast<action_node_t<T>*>(node.children_[i]);
            if( !a_node.pruned_ ) mark_node_as_pruned(a_node);
        }
        node.pruned_ = true;
    }
    void mark_node_as_pruned(action_node_t<T> &a_node) const {
        std::cout << "pruning anode"; a_node.print(std::cout); std::cout << std::endl;
        for( int i = 0; i < int(a_node.children_.size()); ++i ) {
            state_node_t<T> &node = *static_cast<state_node_t<T>*>(a_node.children_[i]);
            if( !node.pruned_ ) mark_node_as_pruned(node);
        }
        a_node.pruned_ = true;
    }

    void propagate_values(state_node_t<T> &node) const {
        update_bounds(node);
        if( node.parent_ != 0 ) {
            assert(dynamic_cast<action_node_t<T>*>(node.parent_) != 0);
            propagate_values(*static_cast<action_node_t<T>*>(node.parent_));
        }
    }
    void propagate_values(action_node_t<T> &a_node) const {
        update_bounds(a_node);
        if( a_node.parent_ != 0 ) {
            assert(dynamic_cast<state_node_t<T>*>(a_node.parent_) != 0);
            propagate_values(*static_cast<state_node_t<T>*>(a_node.parent_));
        }
    }

    float evaluate(const T &s, unsigned num_samples, unsigned depth) const {
        return Evaluation::evaluation(improvement_t<T>::base_policy_, s, num_samples, horizon_ - depth);
    }
    float evaluate(const state_node_t<T> &node) const {
        return evaluate(*node.state_, num_samples_[node.depth_], node.depth_);
    }

    void delete_tree(state_node_t<T> *node) const {
        for( int i = 0; i < int(node->children_.size()); ++i ) {
            action_node_t<T> *a_node = static_cast<action_node_t<T>*>(node->children_[i]);
            delete_tree(a_node);
        }
        delete node;
    }
    void delete_tree(action_node_t<T> *a_node) const {
        for( int i = 0; i < int(a_node->children_.size()); ++i ) {
            state_node_t<T> *node = static_cast<state_node_t<T>*>(a_node->children_[i]);
            delete_tree(node);
        }
        delete a_node;
    }
};

}; // namespace PAC

template<typename T>
inline const policy_t<T>* make_pac_tree(const policy_t<T> &base_policy,
                                        unsigned width,
                                        unsigned horizon,
                                        float parameter,
                                        bool random_ties) {
    return new PAC::pac_tree_t<T>(base_policy, width, horizon, parameter, random_ties);
}

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

