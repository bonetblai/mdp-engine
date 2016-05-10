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

#ifndef IW_BASE_H
#define IW_BASE_H

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <queue>
#include <vector>
#include <math.h>

#include "pomdp.h"

//#define EASY
//#define DEBUG

#define MAGIC_NUMBER    117

namespace Online {

namespace Policy {

namespace IWBase {

////////////////////////////////////////////////
//
// Tuple Factory
//

class tuple_factory_t {
  protected:
    std::vector<int> pool_pointers_;           // contains offsets into pools pointing to next available space in pool
    std::vector<std::pair<int, int*> > pool_;  // pools: each pool is stored as (pool-sz, pointer)
    std::vector<std::list<int*> > free_list_;  // free lists: allocation is always tried first on free list. Indexed by size.

    int space_in_use_;
    int tuples_in_use_;
    int allocated_space_;
    int space_in_free_list_;
    int tuples_in_free_list_;

    int* fetch_tuple_from_pool(int size) {
        for( int i = int(pool_pointers_.size()) - 1; i >= 0; --i ) {
            if( pool_pointers_[i] + 1 + size <= pool_[i].first ) {
                int *tuple = &pool_[i].second[pool_pointers_[i]];
                pool_pointers_[i] += 1 + size;
                tuple[0] = size;
                return tuple;
            }
        }

        // need to allocate new pool
        int n = pool_.empty() ? 1 + size : (size > 2 * pool_.back().first ? 1 + size : 2 * pool_.back().first);
#ifdef DEBUG
        std::cout << "tuple_factory_t: new pool: index=" << pool_.size() << ", size=" << n << std::endl;
#endif
        pool_.push_back(std::make_pair(n, new int[n]));
        pool_pointers_.push_back(1 + size);
        pool_.back().second[0] = size;
        allocated_space_ += n;
        return pool_.back().second;
    }

  public:
    tuple_factory_t()
      : space_in_use_(0),
        tuples_in_use_(0),
        allocated_space_(0),
        space_in_free_list_(0),
        tuples_in_free_list_(0) {
    }
    virtual ~tuple_factory_t() {
        for( int i = 0; i < int(pool_.size()); ++i )
            delete[] pool_[i].second;
    }

    int* get_tuple(int size) {
        if( size >= free_list_.size() )
            free_list_.resize(1 + size);

        int *tuple = 0;
        if( free_list_[size].empty() ) {
            tuple = fetch_tuple_from_pool(size);
        } else {
            tuple = free_list_[size].front();
            free_list_[size].pop_front();
            space_in_free_list_ -= size;
            --tuples_in_free_list_;
        }
        assert(tuple != 0);
        assert(tuple[0] == size);
        space_in_use_ += size;
        ++tuples_in_use_;
        return tuple;
    }

    void free_tuple(const int *tuple) {
        if( tuple[0] >= free_list_.size() )
            free_list_.resize(1 + tuple[0]);
        free_list_[tuple[0]].push_front(const_cast<int *>(tuple));
        space_in_use_ -= tuple[0];
        --tuples_in_use_;
        space_in_free_list_ += tuple[0];
        ++tuples_in_free_list_;
        assert(space_in_use_ >= 0);
        assert(tuples_in_use_ >= 0);
    }
    void free_tuples(const std::vector<const int *> &tuples) {
        for( int i = 0; i < int(tuples.size()); ++i )
            free_tuple(tuples[i]);
    }

    void print_stats(std::ostream &os) const {
        os << "tuple_factory_t: stats:"
           << " space-in-use=" << space_in_use_
           << ", tuples-in-use=" << tuples_in_use_
           << ", allocated-space=" << allocated_space_
           << ", space-in-free-list=" << space_in_free_list_
           << ", tuples-in-free-list=" << tuples_in_free_list_
           << std::endl;
    }
    void print_tuple(std::ostream &os, const int *tuple) const {
        os << "<";
        for( int i = 1; i <= tuple[0]; ++i ) {
            os << tuple[i];
            if( 1 + i <= tuple[0] ) os << ",";
        }
        os << ">" << std::flush;
    }
    void print_tuples(std::ostream &os, const std::vector<const int*> &tuples) const {
        os << "[";
        for( int i = 0; i < int(tuples.size()); ++i ) {
            print_tuple(os, tuples[i]);
            if( 1 + i < int(tuples.size()) ) os << ",";
        }
        os << "]" << std::flush;
    }
};


////////////////////////////////////////////////
//
// Nodes
//

template<typename T> struct node_t {
    const T belief_;
    const node_t<T> *parent_;
    Problem::action_t a_;
    float g_;
    const std::vector<const int*> *tuples_;
    int novelty_;
    const float *tie_breaker_;

    node_t(const node_t<T> &node) = delete;
    node_t(node_t<T> &&node) = delete;

    node_t(const T &belief, const node_t<T> *parent = 0, Problem::action_t a = Problem::noop, float cost = 0)
      : belief_(belief), parent_(parent), a_(a), g_(0), tuples_(0), novelty_(0), tie_breaker_(0) {
        if( parent_ != 0 ) g_ = parent_->g_ + cost;
    }
    node_t(T &&belief, const node_t<T> *parent = 0, Problem::action_t a = Problem::noop, float cost = 0)
      : belief_(std::move(belief)), parent_(parent), a_(a), g_(0), tuples_(0), novelty_(0), tie_breaker_(0) {
        if( parent_ != 0 ) g_ = parent_->g_ + cost;
    }
    ~node_t() { }

    void print(std::ostream &os) const {
        if( tie_breaker_ == 0 )
            os << "[bel=" << belief_ << ",a=" << a_ << ",g=" << g_ << ",novelty=" << novelty_ << ",pa=" << parent_ << "]";
    }
};

template<typename T> inline std::ostream& operator<<(std::ostream &os, const node_t<T> &node) {
    node.print(os);
    return os;
}

template<typename T> inline node_t<T>* get_root_node(const T &belief) {
    return new node_t<T>(belief);
}

template<typename T> inline node_t<T>* get_node(const T &belief, const node_t<T> *parent, Problem::action_t a, float cost) {
    return new node_t<T>(belief, parent, a, cost);
}

template<typename T> inline node_t<T>* get_node(T &&belief, const node_t<T> *parent, Problem::action_t a, float cost) {
    return new node_t<T>(std::move(belief), parent, a, cost);
}


////////////////////////////////////////////////
//
// Hash Table
//

struct tuple_hash_function_t {
    size_t operator()(const int *tuple) const {
        return Utils::jenkins_one_at_a_time_hash(&tuple[1], tuple[0]);
    }
    bool operator()(const int *t1, const int *t2) const {
        return (t1[0] == t2[0]) && (memcmp(&t1[1], &t2[1], sizeof(int) * t1[0]) == 0);
    }
};

class tuple_hash_t : public Hash::generic_hash_set_t<const int*, tuple_hash_function_t, tuple_hash_function_t> {
  public:
    typedef typename Hash::generic_hash_set_t<const int*, tuple_hash_function_t, tuple_hash_function_t> base_type;
    typedef typename base_type::const_iterator const_iterator;
    typedef typename base_type::iterator iterator;

  public:
    tuple_hash_t() { }
    virtual ~tuple_hash_t() { }
    void print(std::ostream &os) const {
        assert(0); // CHECK
    }
};

////////////////////////////////////////////////
//
// Policy
//

template<typename T> class iw_base_t : public policy_t<T> {
  public:
    typedef enum { MOST_LIKELY, SAMPLE } determinization_t;
    typedef enum { REWARD, TARGET } stop_criterion_t;

  protected:
    struct node_priority_t {
        const std::map<int, int> &non_determined_variables_map_;
        node_priority_t(const std::map<int, int> &non_determined_variables_map)
          : non_determined_variables_map_(non_determined_variables_map) {
        }

        bool tie_break(const node_t<T> *n1, const node_t<T> *n2) const {
            bool n1_dominates_n2 = true;
#if 0 // CHECK: not using tie-breaker in this version
            for( int i = 0; i < int(non_determined_variables_map_.size()); ++i ) {
                if( n1->tie_breaker_[i] > n2->tie_breaker_[i] ) {
                    n1_dominates_n2 = false;
                    break;
                }
            }
#endif
            return n1_dominates_n2;
        }

        bool operator()(const node_t<T> *n1, const node_t<T> *n2) const {
            return (n1->g_ > n2->g_) || ((n1->g_ == n2->g_) && tie_break(n1, n2));
        }
    };
    typedef typename std::priority_queue<const node_t<T>*, std::vector<const node_t<T>*>, node_priority_t> priority_queue_t;
    typedef typename std::queue<const node_t<T>*> fifo_queue_t;

  protected:
    const POMDP::pomdp_t<T> &pomdp_;
    std::map<int, int> non_determined_variables_map_;

    // parameters
    unsigned width_;
    int prune_threshold_;
    int discretization_parameter_;
    determinization_t determinization_;
    stop_criterion_t stop_criterion_;
    unsigned max_expansions_;
    bool random_ties_;

    // tuples
    mutable std::vector<tuple_hash_t> tuple_hash_;
    mutable tuple_factory_t tuple_factory_;

    // open and closed list
    mutable std::vector<priority_queue_t> open_lists_;
    mutable std::list<const node_t<T>*> closed_list_;

    struct candidate_t {
        int index_;
        int novelty_;
        const std::vector<const int*> *tuples_;
        const float *tie_breaker_;
        candidate_t(int index, int novelty, const std::vector<const int*> *tuples, const float *tie_breaker)
          : index_(index), novelty_(novelty), tuples_(tuples), tie_breaker_(tie_breaker) {
        }
    };

    iw_base_t(const POMDP::pomdp_t<T> &pomdp,
              unsigned width,
              int prune_threshold,
              int discretization_parameter,
              determinization_t determinization,
              stop_criterion_t stop_criterion,
              unsigned max_expansions,
              bool random_ties)
      : policy_t<T>(pomdp),
        pomdp_(pomdp),
        width_(width),
        prune_threshold_(prune_threshold),
        discretization_parameter_(discretization_parameter),
        determinization_(determinization),
        stop_criterion_(stop_criterion),
        max_expansions_(max_expansions),
        random_ties_(random_ties) {
        set_map_for_non_determined_variables();
        allocate_open_lists_and_hashes();
    }

  public:
    iw_base_t(const POMDP::pomdp_t<T> &pomdp)
      : policy_t<T>(pomdp),
        pomdp_(pomdp),
        width_(0),
        prune_threshold_(1),
        discretization_parameter_(1),
        determinization_(MOST_LIKELY),
        stop_criterion_(TARGET),
        max_expansions_(std::numeric_limits<unsigned>::max()),
        random_ties_(true) {
        set_map_for_non_determined_variables();
        allocate_open_lists_and_hashes();
    }
    virtual ~iw_base_t() { }
    virtual policy_t<T>* clone() const {
        return new iw_base_t(pomdp_);
    }
    virtual std::string name() const {
        return std::string("iw-base(") +
          std::string("width=") + std::to_string(width_) +
          std::string(",prune-threshold=") + std::to_string(prune_threshold_) +
          std::string(",discretization-parameter=") + std::to_string(discretization_parameter_) +
          std::string(",determinization=") + (determinization_ == MOST_LIKELY ? "most-likely" : "sample") +
          std::string(",stop-criterion=") + (stop_criterion_ == TARGET ? "target" : "reward") +
          std::string(",max-expansions=") + std::to_string(max_expansions_) +
          std::string(",random_ties=") + (random_ties_ ? "true" : "false") +
          std::string(")");
    }

    Problem::action_t operator()(const T &bel) const {
        assert(open_lists_are_empty());
        assert(tuple_hash_is_empty());
        assert(closed_list_.empty());
#if 0//def EASY//def DEBUG
        std::cout << std::endl
                  << "**** REQUEST FOR ACTION ****" << std::endl
                  << "root=" << bel << std::endl
                  << "available actions:";
        for( Problem::action_t a = 0; a < pomdp_.number_actions(bel); ++a ) {
            if( pomdp_.applicable(bel, a) )
                std::cout << " " << pomdp_.action_name(a);
        }
        std::cout << std::endl
                  << "throwing BFS for goal" << std::endl;
#endif

        ++policy_t<T>::decisions_;
        if( pomdp_.dead_end(bel) ) return Problem::noop;

        node_t<T> *root = get_root_node(bel);
        fill_tuples(*root);
        root->novelty_ = compute_novelty(*root->tuples_);
        if( root->novelty_ >= prune_threshold_ ) {
#ifdef EASY
            std::cout << "ROOT PRUNED: NOVELTY > threshold (" << root->novelty_ << " > " << prune_threshold_ << ")" << std::endl;
#endif
            tuple_factory_.free_tuples(*root->tuples_);
            delete[] root->tie_breaker_;
            delete root->tuples_;
            delete root;
            return Online::Policy::random_t<T>(policy_t<T>::problem_)(bel);
        }

        // perform search
        const node_t<T> *node = uniform_cost_search(root);

#ifdef EASY//def DEBUG
        print_stats(std::cout);
        std::cout << "node=";
        if( node == 0 )
            std::cout << "null";
        else
            std::cout << *node;
        std::cout << std::endl;
#endif

        Problem::action_t action = Problem::noop;
        if( node == 0 ) {
            // goal node wasn't found, return random action
            action = Online::Policy::random_t<T>(policy_t<T>::problem_)(bel);
        } else {
            // return first action in path
            const node_t<T> *n = node;
            const node_t<T> *parent = node->parent_;
            assert(parent != 0);
            while( parent->parent_ != 0 ) {
#ifdef EASY//def DEBUG
                std::cout << "action=" << pomdp_.action_name(n->a_) << std::endl;
#endif
                n = parent;
                parent = n->parent_;
            }
            assert(n->parent_->belief_ == bel);
#ifdef EASY//def DEBUG
            std::cout << "action=" << pomdp_.action_name(n->a_) << std::endl;
#endif
            action = n->a_;
        }
        free_resources();
#if 0//def EASY//def DEBUG
        print_stats(std::cout);
        std::cout << "BEST ACTION=" << pomdp_.action_name(action) << std::endl;
#endif
        return action;
    }

    virtual void reset_stats() const {
        pomdp_.clear_expansions();
    }
    virtual void print_other_stats(std::ostream &os, int indent) const {
        os << std::setw(indent) << ""
           << "other-stats: name=" << name()
           << " decisions=" << policy_t<T>::decisions_
           << std::endl;
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("width");
        if( it != parameters.end() ) width_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("prune-threshold");
        if( it != parameters.end() ) {
            prune_threshold_ = strtol(it->second.c_str(), 0, 0);
            allocate_open_lists_and_hashes();
        }
        it = parameters.find("discretization-parameter");
        if( it != parameters.end() ) discretization_parameter_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("dp");
        if( it != parameters.end() ) discretization_parameter_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("determinization");
        if( it != parameters.end() ) determinization_ = it->second == "sample" ? SAMPLE : MOST_LIKELY;
        it = parameters.find("stop-criterion");
        if( it != parameters.end() ) stop_criterion_ = it->second == "reward" ? REWARD : TARGET;
        it = parameters.find("max-expansions");
        if( it != parameters.end() ) max_expansions_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("random-ties");
        if( it != parameters.end() ) random_ties_ = it->second == "true";
        policy_t<T>::setup_time_ = 0;
#ifdef DEBUG
        std::cout << "debug: iw-base(): params:"
                  << " width=" << width_
                  << " prune-threshold=" << prune_threshold_
                  << " discretization-parameter=" << discretization_parameter_
                  << " determinization=" << (determinization_ == MOST_LIKELY ? "most-likely" : "sample")
                  << " stop-criterion=" << (stop_criterion_ == TARGET ? "target" : "reward")
                  << " max-expansions=" << max_expansions_
                  << " random-ties=" << (random_ties_ ? "true" : "false")
                  << std::endl;
#endif
    }
    virtual typename policy_t<T>::usage_t uses_base_policy() const { return policy_t<T>::usage_t::No; }
    virtual typename policy_t<T>::usage_t uses_heuristic() const { return policy_t<T>::usage_t::No; }
    virtual typename policy_t<T>::usage_t uses_algorithm() const { return policy_t<T>::usage_t::No; }

    void fill_tuples(node_t<T> &node) const {
        if( node.tuples_ != 0 ) {
            tuple_factory_.free_tuples(*node.tuples_);
            const_cast<std::vector<const int*>*>(node.tuples_)->clear();
        } else {
            node.tuples_ = new std::vector<const int*>();
        }
        assert(node.tie_breaker_ == 0); // CHECK: not using tie-breaker in this version
#if 0 // CHECK: not using tie-breaker in this version
        if( node.tie_breaker_ == 0 ) {
            node.tie_breaker_ = new float[non_determined_variables_map_.size()];
        }
#endif
        std::vector<const int *> tuples;
        fill_tuples(node.belief_, tuples, const_cast<float*>(node.tie_breaker_));
        *const_cast<std::vector<const int*>*>(node.tuples_) = std::move(tuples);
    }
    void fill_tuples(const T &belief, std::vector<const int*> &tuples, float *tie_breaker) const {
        // tuples of size 1: (a) tuples (X,x,dp) that means belief satisfies
        // p = P(X=x) and discret(p) = dp for all *non-determined* variables X
        // and values x of X, and (b) tuples (X,x) that means belief satisfies
        // X=x for all *determined* variables X with value x
        if( prune_threshold_ > 0 ) {
            for( int vid = 0; vid < pomdp_.number_variables(); ++vid ) {
                if( pomdp_.determined(vid) ) {
                    // calculate code for atom 1(b) (X,x)
                    int value = belief.value(vid);
                    assert(value != -1);
                    int code = value * pomdp_.number_variables() + vid;

                    // fill tuple and insert
                    int *tuple = tuple_factory_.get_tuple(1);
                    assert(tuple[0] == 1);
                    tuple[1] = code;
                    tuples.push_back(tuple);
                } else {
                    std::vector<float> probabilities(pomdp_.domain_size(vid), 0);
                    pomdp_.fill_values_for_variable(belief, vid, probabilities);
                    for( int value = 0; value < int(probabilities.size()); ++value ) {
                        // calculate code for atom 1(b) (X,x,dp)
                        float p = probabilities[value];
                        int dp = ceilf(p * discretization_parameter_);
                        assert(dp <= discretization_parameter_);
                        int code = (value * (1 + discretization_parameter_) + dp) * pomdp_.number_variables() + vid;

                        // fill tuple and insert
                        int *tuple = tuple_factory_.get_tuple(1);
                        assert(tuple[0] == 1);
                        tuple[1] = code;
                        tuples.push_back(tuple);
                    }
                }
            }
        }

        // tuples of novelty 2: pairs <(X,x),(Y,y,dp)> that means belief satisfies
        // X=x, p = P(Y=y), p > 0, and discret(p) = dp for *determined* variables X
        // and *non-determined* variables Y
        if( prune_threshold_ > 1 ) {
            for( int xvid = 0; xvid < pomdp_.number_variables(); ++xvid ) {
                if( pomdp_.determined(xvid) ) {
                    // calculate code for atom (X,x)
                    int xvalue = belief.value(xvid);
                    assert(xvalue != -1);
                    int xcode = xvalue * pomdp_.number_variables() + xvid;
                    for( int yvid = 0; yvid < pomdp_.number_variables(); ++yvid ) {
                        if( !pomdp_.determined(yvid) ) {
                            std::vector<float> probabilities(pomdp_.domain_size(yvid), 0);
                            pomdp_.fill_values_for_variable(belief, yvid, probabilities);
                            for( int yvalue = 0; yvalue < int(probabilities.size()); ++yvalue ) {
                                // calculate code for atom (Y,y,dp)
                                float p = probabilities[yvalue];
                                int dp = ceilf(p * discretization_parameter_);
                                assert(dp <= discretization_parameter_);
                                int ycode = (yvalue * (1 + discretization_parameter_) + dp) * pomdp_.number_variables() + yvid;

                                // fill tuple and insert
                                int *tuple = tuple_factory_.get_tuple(2);
                                assert(tuple[0] == 2);
                                tuple[1] = xcode;
                                tuple[2] = ycode;
                                tuples.push_back(tuple);
                            }
                        }
                    }
                }
            }
        }

#ifdef DEBUG
        std::cout << "[tuples=";
        print_tuples(std::cout, tuples);
        std::cout << "]" << std::endl;
#endif
    }

    void print_tuples(std::ostream &os, const std::vector<const int*> &tuples) const {
        os << "{";
        for( int i = 0; i < int(tuples.size()); ++i ) {
            const int *tuple = tuples[i];
            print_tuple(os, tuple);
            if( i + 1 < int(tuples.size()) ) os << ",";
        }
        os << "}";
    }
    void print_tuple(std::ostream &os, const int *tuple) const {
        os << tuple[0] << "=<";
        for( int i = 1; i <= tuple[0]; ++i ) {
            int code = tuple[i];
            int vid = code % pomdp_.number_variables();
            if( pomdp_.determined(vid) ) {
                int value = code / pomdp_.number_variables();
                os << "(X" << vid << "," << value << ")";
            } else {
                code = code / pomdp_.number_variables();
                int dp = code % (1 + discretization_parameter_);
                int value = code / (1 + discretization_parameter_);
                os << "(X" << vid << "," << value << ",dp=" << dp << ")";
            }
            if( 1 + i <= tuple[0] ) os << ",";
        }
        os << ">";
    }

    void print_stats(std::ostream &os) const {
        os << "stats: open.sz[" << open_lists_.size() << "]=[";
        for( int i = 0; i < int(open_lists_.size()); ++i ) {
            os << open_lists_[i].size();
            if( 1 + i < int(open_lists_.size()) ) os << ",";
        }
        os << "], closed.sz=" << closed_list_.size() << ", tuples.sz=[";
        for( int i = 0; i < int(tuple_hash_.size()); ++i ) {
            os << tuple_hash_[i].size();
            if( 1 + i < int(tuple_hash_.size()) ) os << ",";
        }
        os << "]" << std::endl;
        tuple_factory_.print_stats(os);
    }
    void free_resources() const {
        for( int i = 0; i < int(open_lists_.size()); ++i ) {
            priority_queue_t &open_list = open_lists_[i];
            while( !open_list.empty() ) {
                const node_t<T> *node = open_list.top();
                open_list.pop();
                assert(node->tuples_ != 0);
                tuple_factory_.free_tuples(*node->tuples_);
                delete[] node->tie_breaker_;
                delete node->tuples_;
                delete node;
            }
        }

        for( typename std::list<const node_t<T>*>::const_iterator it = closed_list_.begin(); it != closed_list_.end(); ++it ) {
            const node_t<T> *node = *it;
            assert(node->tuples_ != 0);
            tuple_factory_.free_tuples(*node->tuples_);
            delete[] node->tie_breaker_;
            delete node->tuples_;
            delete node;
        }
        closed_list_.clear();

        for( int i = 0; i < int(tuple_hash_.size()); ++i )
            tuple_hash_[i].clear();
    }
 
    // breadth-first search
    const node_t<T>* uniform_cost_search(const node_t<T> *root) const {
        std::vector<std::pair<T, float> > outcomes;

        push_node(root);
        for( unsigned iter = 0; !open_lists_are_empty() && (iter < max_expansions_); ++iter ) {
            const node_t<T> *node = select_node_for_expansion();
            closed_list_.push_front(node);

#ifdef EASY
            std::cout << "SELECT: node=" << *node << std::endl;
#endif

            // check whether we need to stop
            if( pomdp_.terminal(node->belief_) ) {
#ifdef DEBUG
                std::cout << "TERMINAL FOUND: bel=" << node->belief_ << std::endl;
#endif
                return node;
            }

            // insert node's tuples into hash of seen tuples
            assert(node->tuples_ != 0);
            register_tuples(*node->tuples_);
            
            // expand node
            std::vector<candidate_t> candidates;
            for( Problem::action_t a = 0; a < pomdp_.number_actions(node->belief_); ++a ) {
                if( pomdp_.applicable(node->belief_, a) ) {
                    // expand node
                    pomdp_.next(node->belief_, a, outcomes);

#ifdef EASY
                    std::cout << "action=" << pomdp_.action_name(a) << ", outcomes.sz=" << outcomes.size() << std::endl;
#endif

                    assert(candidates.empty());
                    for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
#ifdef EASY
                        std::cout << "    " << outcomes[i].first << std::endl;
                        std::cout << "    considering outcome " << i << std::flush;
#endif
                        if( pomdp_.dead_end(outcomes[i].first) ) {
#ifdef EASY
                            std::cout << " --> DEAD END" << std::endl;
#endif
                            continue;
                        }

                        // compute tuples and novelty for this outcome
                        std::vector<const int*> *tuples = new std::vector<const int*>();
                        float *tie_breaker = 0; //new float[non_determined_variables_map_.size()]; // CHECK: not using tie-breaker in this version
                        fill_tuples(outcomes[i].first, *tuples, tie_breaker);
                        int novelty = compute_novelty(*tuples);
#ifdef EASY
                        std::cout << " (novelty=" << novelty << ")" << std::flush;
#endif

                        // prune node if all its tuples already seen
                        if( novelty >= prune_threshold_ ) {
                            tuple_factory_.free_tuples(*tuples);
                            delete[] tie_breaker;
                            delete tuples;
#ifdef EASY
                            std::cout << " --> PRUNED: NOVELTY > threshold (" << prune_threshold_ << ")" << std::endl;
#endif
                            continue;
                        }

                        // save candidate (if applicable)
                        if( determinization_ == SAMPLE ) {
                            std::cout << " --> SAVED" << std::endl;
                            candidates.push_back(candidate_t(i, novelty, tuples, tie_breaker));
                        } else {
                            // check if this outcome is most probable (best candidate) // CHECK: IDEA: keep most probable modulo discretization and sample below
                            if( candidates.empty() || (outcomes[i].second >= outcomes[candidates.back().index_].second) ) {
                                if( !candidates.empty() && (outcomes[i].second > outcomes[candidates.back().index_].second) )
                                    clear_candidate_list(candidates);
                                candidates.push_back(candidate_t(i, novelty, tuples, tie_breaker));
#ifdef EASY
                                std::cout << " --> CURRENT BEST (p=" << outcomes[i].second << ")" << std::endl;
#endif
                            } else {
                                tuple_factory_.free_tuples(*tuples);
                                delete[] tie_breaker;
                                delete tuples;
#ifdef EASY
                                std::cout << " --> PRUNED: NOT BEST (p=" << outcomes[i].second << ")" << std::endl;
#endif
                            }
                        }
                    }

                    // select best candidate
                    if( !candidates.empty() ) {
                        // compute cdf of candidates for performing sampling
                        std::vector<float> cdf(candidates.size(), 0);
                        for( int i = 0; i < int(candidates.size()); ++i ) {
                            cdf[i] = i == 0 ? 0 : cdf[i - 1];
                            if( determinization_ == SAMPLE )
                                cdf[i] += outcomes[candidates[i].index_].second;
                            else
                                cdf[i] += 1.0 / float(candidates.size());
                        }
                        for( int i = 0; i < int(cdf.size()); ++i )
                            cdf[i] /= cdf.back();

                        // sample from cdf and create new node
                        float cost = pomdp_.cost(node->belief_, a);
                        int sampled = Random::sample_from_distribution(cdf.size(), &cdf[0]);
                        node_t<T> *new_node = get_node(std::move(outcomes[candidates[sampled].index_].first), node, a, cost);
                        new_node->novelty_ = candidates[sampled].novelty_;
                        new_node->tuples_ = candidates[sampled].tuples_;
                        new_node->tie_breaker_ = candidates[sampled].tie_breaker_;
                        push_node(new_node);
#ifdef EASY
                        std::cout << "    candidates: #=" << candidates.size() << " selected: node=" << *new_node << std::endl;
#endif

                        // clear non selected tuples 
                        candidates[sampled] = candidates.back();
                        candidates.pop_back();
                        clear_candidate_list(candidates);
                    }
                }
            }
        }

        // no terminal node was found during search and the either the queue is empty
        // or we ran out of expansions.
        //
        // Return a best node in open/closed if stop-criterion is REWARD
        if( stop_criterion_ == REWARD ) {
#if 0 // CHECK
            std::vector<const node_t<T>*> best;
            for( int i = 0; i < int(open_lists_.size()); ++i ) {
                const priority_queue_t &open_list = open_lists_[i];
                const std::vector<const node_t<T>*> &container = Container(open_list);
                for( int j = 0; j < int(container.size()); ++j ) {
                    const node_t<T> *node = container[j];
                    if( best.empty() || (node->g_ <= best.back()->g_) ) {
                        if( !best.empty() && (node->g_ < best.back()->g_) )
                            best.clear();
                        best.push_back(node);
                    }
                }
            }
#endif

#if 0 // CHECK
            // search closed list when all open-lists are empty
            if( false && best.empty() ) { // CHECK
                assert(open_lists_are_empty());
                for( typename std::list<const node_t<T>*>::const_iterator it = closed_list_.begin(); it != closed_list_.end(); ++it ) {
                    const node_t<T> *node = *it;
                    if( best.empty() || (node->g_ <= best.back()->g_) ) {
                        if( !best.empty() && (node->g_ < best.back()->g_) )
                            best.clear();
                        best.push_back(*it);
                    }
                }
            }
#endif
            //if( open_lists_[0].empty() ) std::cout << "**** BEST IS EMPTY" << std::endl;
            return open_lists_[0].empty() ? 0 : open_lists_[0].top(); // CHECK
        }
        return 0;
    }

    void clear_candidate_list(std::vector<candidate_t> &candidates) const {
        for( int i = 0; i < int(candidates.size()); ++i ) {
            tuple_factory_.free_tuples(*candidates[i].tuples_);
            delete[] candidates[i].tie_breaker_;
            delete candidates[i].tuples_;
        }
        candidates.clear();
    }

    bool tuple_hash_is_empty() const {
        for( int i = 0; i < int(tuple_hash_.size()); ++i ) {
            if( !tuple_hash_[i].empty() )
                return false;
        }
        return true;
    }
    void register_tuples(const std::vector<const int*> &tuples) const {
        std::vector<int> delta_sizes(tuple_hash_.size(), 0);
        for( int i = 0; i < int(tuples.size()); ++i ) {
            const int *tuple = tuples[i];
            int size = tuple[0];
            assert((size > 0) && (size <= int(tuple_hash_.size())));
            if( tuple_hash_[size - 1].find(tuple) == tuple_hash_[size - 1].end() ) {
                tuple_hash_[size - 1].insert(tuple);
                ++delta_sizes[size - 1];
            }
        }
#ifdef DEBUG
        std::cout << "register_tuples: #tuples=" << tuples.size() << ", hash-size variations:";
        for( int i = 0; i < int(tuple_hash_.size()); ++i ) {
            if( delta_sizes[i] > 0 )
                std::cout << " (" << 1 + i << "," << delta_sizes[i] << ")";
        }
        std::cout << std::endl;
#endif
    }

    int compute_novelty(const std::vector<const int*> &tuples) const {
        int novelty = std::numeric_limits<int>::max();
        for( int i = 0; i < int(tuples.size()); ++i ) {
            const int *tuple = tuples[i];
            assert((tuple[0] > 0) && (tuple[0] <= int(tuple_hash_.size())));
            tuple_hash_t::const_iterator it = tuple_hash_[tuple[0] - 1].find(tuple);
            if( it == tuple_hash_[tuple[0] - 1].end() )
                novelty = tuple[0] - 1 < novelty ? tuple[0] - 1 : novelty;
        }
        assert(novelty >= 0);
        return novelty;
    }

    void set_map_for_non_determined_variables() {
        for( int vid = 0; vid < pomdp_.number_variables(); ++vid ) {
            if( !pomdp_.determined(vid) )
                non_determined_variables_map_.insert(std::make_pair(vid, non_determined_variables_map_.size()));
        }
    }

    void allocate_open_lists_and_hashes() {
        open_lists_.clear();
        open_lists_.reserve(1 + prune_threshold_);
        for( int i = 0; i <= prune_threshold_; ++i )
            open_lists_.push_back(priority_queue_t(node_priority_t(non_determined_variables_map_)));

        tuple_hash_.reserve(1 + prune_threshold_);
        for( int i = 0; i <= prune_threshold_; ++i )
            tuple_hash_.push_back(tuple_hash_t());
    }
    bool open_lists_are_empty() const {
        for( int i = 0; i < int(open_lists_.size()); ++i ) {
            if( !open_lists_[i].empty() )
                return false;
        }
        return true;
    }
    void push_node(const node_t<T> *node) const {
        assert(node->novelty_ >= 0);
        assert(node->novelty_ < int(open_lists_.size()));
        //open_lists_[node->novelty_].push(node); // CHECK: using only one open list
        open_lists_[0].push(node); // CHECK: using only one open list
    }
    const node_t<T>* select_node_for_expansion() const {
#ifdef EASY
        print_stats(std::cout);
#endif
        for( int i = 0; i < int(open_lists_.size()); ++i ) {
            if( !open_lists_[i].empty() ) {
                const node_t<T> *node = open_lists_[i].top();
                open_lists_[i].pop();
                return node;
            }
        }
        return 0;
    }
};

}; // namespace IWBase

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

