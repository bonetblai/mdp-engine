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

#ifndef IW_BEL_H
#define IW_BEL_H

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <vector>
#include <math.h>

#include "pomdp.h"

//#define DEBUG

namespace Online {

namespace Policy {

namespace IWBel {

////////////////////////////////////////////////
//
// Nodes
//

template<typename T> struct node_t {
    const T belief_;
    const POMDP::feature_t<T> *feature_;
    const node_t<T> *parent_;
    Problem::action_t a_;
    float g_;

    node_t(const T &belief, const POMDP::feature_t<T> *feature, const node_t<T> *parent = 0, Problem::action_t a = 0, float cost = 0)
      : belief_(belief), feature_(feature), parent_(parent), a_(a), g_(0) {
        if( parent_ != 0 ) g_ = parent_->g_ + cost;
    }
    ~node_t() { }
};

template<typename T> inline node_t<T>* get_root_node(const T &belief, const POMDP::feature_t<T> *feature) {
    return new node_t<T>(belief, feature);
}

template<typename T> inline node_t<T>* get_node(const T &belief, const POMDP::feature_t<T> *feature, const node_t<T> *parent, Problem::action_t a, float cost) {
    return new node_t<T>(belief, feature, parent, a, cost);
}

////////////////////////////////////////////////
//
// Hash Table
//

template<typename T> struct node_map_function_t {
    size_t operator()(const T *bel) const {
        return bel->hash();
    }
};

#if 0
struct data_t {
    std::vector<float> values_;
    std::vector<int> counts_;
    data_t(const std::vector<float> &values, const std::vector<int> &counts)
      : values_(values), counts_(counts) { }
    data_t(const data_t &data)
      : values_(data.values_), counts_(data.counts_) { }
    data_t(data_t &&data)
      : values_(std::move(data.values_)), counts_(std::move(data.counts_)) { }
};
#endif

template<typename T> class node_hash_t : public Hash::generic_hash_map_t<const T*, const node_t<T>*, node_map_function_t<T> > {
  public:
    typedef typename Hash::generic_hash_map_t<const T*, const node_t<T>*, node_map_function_t<T> > base_type;
    typedef typename base_type::const_iterator const_iterator;
    //typedef typename base_type::iterator iterator; //CHECK
    //const_iterator begin() const { return base_type::begin(); } //CHECK
    //const_iterator end() const { return base_type::end(); } //CHECK

  public:
    node_hash_t() { }
    virtual ~node_hash_t() { }
    void print(std::ostream &os) const {
        assert(0); // CHECK
#if 0
        for( const_iterator it = begin(); it != end(); ++it ) {
            os << "(" << it->first.first << "," << it->first.second << ")" << std::endl;
        }
#endif
    }
};

////////////////////////////////////////////////
//
// Policy
//

template<typename T> class iw_bel_t : public policy_t<T> {
  protected:
    const POMDP::pomdp_t<T> &pomdp_;

    mutable node_hash_t<T> node_table_;

    struct min_priority_t {
        bool operator()(const node_t<T> *n1, const node_t<T> *n2) const {
            return n1->g_ > n2->g_;
        }
    };
    typedef typename std::priority_queue<const node_t<T>*, std::vector<const node_t<T>*>, min_priority_t> priority_queue_t;

    iw_bel_t(const POMDP::pomdp_t<T> &pomdp,
             unsigned width,
             unsigned horizon,
             float parameter,
             bool random_ties)
      : policy_t<T>(pomdp),
        pomdp_(pomdp) {
    }

  protected:


  public:
    iw_bel_t(const POMDP::pomdp_t<T> &pomdp)
      : policy_t<T>(pomdp),
        pomdp_(pomdp) {
    }
    virtual ~iw_bel_t() { }
    virtual policy_t<T>* clone() const {
        return new iw_bel_t(pomdp_);
    }
    virtual std::string name() const {
        return std::string("iw-bel()");
    }

    Problem::action_t operator()(const T &bel) const {
        std::cout << "bel=" << bel << std::endl;
        std::cout << "cardinality=" << bel.cardinality() << std::endl;
        std::cout << "throwing pruned BFS for goal" << std::endl;

        if( pomdp_.dead_end(bel) ) return Problem::noop;

        const POMDP::feature_t<T> *feature = pomdp_.get_feature(bel);
        node_t<T> *root = get_root_node(bel, feature);
        const node_t<T> *goal = pruned_bfs(root);

        if( goal == 0 ) {
            // goal node wasn't found, return random action
            std::vector<Problem::action_t> applicable_actions;
            applicable_actions.reserve(pomdp_.number_actions(bel));
            for( Problem::action_t a = 0; a < pomdp_.number_actions(bel); ++a ) {
                if( pomdp_.applicable(bel, a) ) {
                    applicable_actions.push_back(a);
                }
            }
            return applicable_actions.empty() ? Problem::noop : applicable_actions[Random::random(applicable_actions.size())];
        } else {
            // return first action in path
            const node_t<T> *node = goal;
            const node_t<T> *parent = node->parent_;
            while( parent != 0 ) {
                node = parent;
                parent = node->parent_;
            }
            assert(node->belief_ == bel);
            return node->a_;
        }
    }
    virtual void reset_stats() const {
#if 0
        policy_t<T>::setup_time_ = 0;
        policy_t<T>::base_policy_time_ = 0;
        policy_t<T>::heuristic_time_ = 0;
        problem_.clear_expansions();
        if( base_policy_ != 0 ) base_policy_->reset_stats();
#endif
    }
    virtual void print_other_stats(std::ostream &os, int indent) const {
#if 0
        os << std::setw(indent) << ""
           << "other-stats: name=" << name()
           << " decisions=" << policy_t<T>::decisions_
           << std::endl;
        if( base_policy_ != 0 ) base_policy_->print_other_stats(os, 2 + indent);
#endif
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
#if 0
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("width");
        if( it != parameters.end() ) width_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("horizon");
        if( it != parameters.end() ) horizon_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("parameter");
        if( it != parameters.end() ) parameter_ = strtod(it->second.c_str(), 0);
        it = parameters.find("random-ties");
        if( it != parameters.end() ) random_ties_ = it->second == "true";
        it = parameters.find("policy");
        if( it != parameters.end() ) {
            delete base_policy_;
            dispatcher.create_request(problem_, it->first, it->second);
            base_policy_ = dispatcher.fetch_policy(it->second);
        }
        policy_t<T>::setup_time_ = base_policy_ == 0 ? 0 : base_policy_->setup_time();
#ifdef DEBUG
        std::cout << "debug: iw-bel(): params:"
                  << " width=" << width_
                  << " horizon=" << horizon_
                  << " parameter=" << parameter_
                  << " random-ties=" << (random_ties_ ? "true" : "false")
                  << " policy=" << (base_policy_ == 0 ? std::string("null") : base_policy_->name())
                  << std::endl;
#endif
#endif
    }
    virtual typename policy_t<T>::usage_t uses_base_policy() const { return policy_t<T>::usage_t::No; }
    virtual typename policy_t<T>::usage_t uses_heuristic() const { return policy_t<T>::usage_t::No; }
    virtual typename policy_t<T>::usage_t uses_algorithm() const { return policy_t<T>::usage_t::No; }

    void clear_feature_table() const {
        assert(0);
        // CHECK
    }
    bool lookup_feature(const POMDP::feature_t<T> &feature) const {
        assert(0);
        return false; // CHECK
    }
    bool insert_feature(const POMDP::feature_t<T> &feature) const {
        assert(0);
        return false; // CHECK
    }

    void clear_node_table() const {
        for( typename node_hash_t<T>::const_iterator it = node_table_.begin(); it != node_table_.end(); ++it )
            delete it->second;
        node_table_.clear();
    }
    const node_t<T>* lookup_node(const T &belief) const {
        typename node_hash_t<T>::const_iterator it = node_table_.find(&belief);
        return it == node_table_.end() ? 0 : it->second;
    }
    bool insert_node(const node_t<T> *node) const {
        std::pair<typename node_hash_t<T>::iterator, bool> p = node_table_.insert(std::make_pair(&node->belief_, node));
        return p.second;
    }

    // pruned breadth-first search
    const node_t<T>* pruned_bfs(const node_t<T> *root) const {
        priority_queue_t q;
        std::vector<std::pair<T, float> > outcomes;

        clear_feature_table();
        clear_node_table();

        insert_feature(*root->feature_);
        insert_node(root);
        q.push(root);
        while( !q.empty() ) {
            const node_t<T> *node = q.top();
            q.pop();

            if( pomdp_.terminal(node->belief_) ) {
                return node;
            }

            for( Problem::action_t a = 0; a < pomdp_.number_actions(node->belief_); ++a ) {
                if( pomdp_.applicable(node->belief_, a) ) {
                    pomdp_.next(node->belief_, a, outcomes);

                    // select best (unexpanded) successors to continue search
                    int best = -1;
                    for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
                        if( pomdp_.dead_end(outcomes[i].first) ) continue;
                        if( lookup_node(outcomes[i].first) != 0 ) continue;
                        if( (best == -1) && (outcomes[i].second > outcomes[best].second) ) {
                            best = i;
                        }
                    }

                    // insert in queue if it is novel (CHECK: what happens if it is not novel but there is another successor that is novel?)
                    const POMDP::feature_t<T> *feature = pomdp_.get_feature(outcomes[best].first);
                    if( insert_feature(*feature) ) {
                        float cost = pomdp_.cost(node->belief_, a);
                        node_t<T> *new_node = get_node(outcomes[best].first, feature, node, a, cost);
                        q.push(new_node);
                    }
                }
            }
        }
        return 0;
    }
};

}; // namespace IWBel

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

