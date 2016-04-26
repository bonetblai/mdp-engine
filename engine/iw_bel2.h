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

#ifndef IW_BEL2_H
#define IW_BEL2_H

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

namespace Online {

namespace Policy {

namespace IWBel2 {

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
    std::vector<const int*> tuples_;
    int novelty_;

    node_t(const T &belief, const node_t<T> *parent = 0, Problem::action_t a = Problem::noop, float cost = 0)
      : belief_(belief), parent_(parent), a_(a), g_(0), novelty_(0) {
        if( parent_ != 0 ) g_ = parent_->g_ + cost;
    }
    ~node_t() { }

    void set_tuples(const std::vector<const int*> &tuples) {
        tuples_ = tuples;
    }
    void push_tuple(const int *tuple) {
        tuples_.push_back(tuple);
    }

    void print(std::ostream &os) const {
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


template <typename T, typename S, typename C>
inline const S& Container(const std::priority_queue<T, S, C> &q) {
    struct HackedQueue : private std::priority_queue<T, S, C> {
        static const S& Container(const std::priority_queue<T, S, C> &q) {
            return q.*&HackedQueue::c;
        }
    };
    return HackedQueue::Container(q);
}

////////////////////////////////////////////////
//
// Hash Table
//

template<typename T> struct node_map_function_t {
    size_t operator()(const T *bel) const {
        return bel->hash();
    }
    bool operator()(const T *b1, const T *b2) const {
        return *b1 == *b2;
    }
};

#if 0 // CHECK
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

template<typename T> class node_hash_t : public Hash::generic_hash_map_t<const T*, const node_t<T>*, node_map_function_t<T>, node_map_function_t<T> > {
  public:
    typedef typename Hash::generic_hash_map_t<const T*, const node_t<T>*, node_map_function_t<T>, node_map_function_t<T> > base_type;
    typedef typename base_type::const_iterator const_iterator;
    typedef typename base_type::iterator iterator;

  public:
    node_hash_t() { }
    virtual ~node_hash_t() { }
    void print(std::ostream &os) const {
        assert(0); // CHECK
#if 0 // CHECK
        for( const_iterator it = begin(); it != end(); ++it ) {
            os << "(" << it->first.first << "," << it->first.second << ")" << std::endl;
        }
#endif
    }
};

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

template<typename T> class iw_bel2_t : public policy_t<T> {
  public:
    typedef enum { MOST_LIKELY, SAMPLE } determinization_t;
    typedef enum { REWARD, TARGET } stop_criterion_t;
    typedef enum { TOTAL, KL, KL_SYM, JS, UNKNOWN } divergence_t;
    typedef enum { MAX, ADD } score_aggregation_t;

    static divergence_t divergence_from_name(const std::string &ds) {
        for( int d = TOTAL; d < UNKNOWN; ++d ) {
            if( ds == divergence_name(static_cast<divergence_t>(d)) )
                return static_cast<divergence_t>(d);
        }
        return UNKNOWN;
    }
    static std::string divergence_name(divergence_t d) {
        if( d == TOTAL )
            return "total";
        else if( d == KL )
            return "kl";
        else if( d == KL_SYM )
            return "kl-sym";
        else if( d == JS )
            return "js";
        else
            return "unknown";
    }
  protected:
    struct min_priority_t {
        bool operator()(const node_t<T> *n1, const node_t<T> *n2) const {
            return n1->g_ > n2->g_;
        }
    };
    typedef typename std::priority_queue<const node_t<T>*, std::vector<const node_t<T>*>, min_priority_t> priority_queue_t;

  protected:
    const POMDP::pomdp_t<T> &pomdp_;

    // parameters
    unsigned width_;
    int prune_threshold_;
    int discretization_parameter_;
    determinization_t determinization_;
    stop_criterion_t stop_criterion_;
    divergence_t divergence_;
    unsigned max_expansions_;
    score_aggregation_t score_aggregation_;
    bool random_ties_;

    // tuples
    mutable std::vector<tuple_hash_t> tuple_hash_;
    mutable tuple_factory_t tuple_factory_;

    // open and closed list
    mutable std::vector<priority_queue_t> open_lists_;
    mutable std::list<const node_t<T>*> closed_list_;
    //mutable std::list<const node_t<T>*> open_list_; // CHECK: REMOVE

#if 0
    // hash table (dynamic memory that must be cleared after action selection)
    mutable node_hash_t<T> node_table_;
#endif

#if 0
    struct min_priority_t {
        bool operator()(const node_t<T> *n1, const node_t<T> *n2) const {
            return n1->g_ > n2->g_;
        }
    };
    typedef typename std::priority_queue<const node_t<T>*, std::vector<const node_t<T>*>, min_priority_t> priority_queue_t;
#endif

    iw_bel2_t(const POMDP::pomdp_t<T> &pomdp,
             unsigned width,
             int prune_threshold,
             int discretization_parameter,
             determinization_t determinization,
             stop_criterion_t stop_criterion,
             divergence_t divergence,
             unsigned max_expansions,
             score_aggregation_t score_aggregation,
             bool random_ties)
      : policy_t<T>(pomdp),
        pomdp_(pomdp),
        width_(width),
        prune_threshold_(prune_threshold),
        discretization_parameter_(discretization_parameter),
        determinization_(determinization),
        stop_criterion_(stop_criterion),
        divergence_(divergence),
        max_expansions_(max_expansions),
        score_aggregation_(score_aggregation),
        random_ties_(random_ties) {
        allocate_open_lists_and_tuple_hash();
    }

  protected:


  public:
    iw_bel2_t(const POMDP::pomdp_t<T> &pomdp)
      : policy_t<T>(pomdp),
        pomdp_(pomdp),
        width_(0),
        prune_threshold_(1),
        discretization_parameter_(1),
        determinization_(MOST_LIKELY),
        stop_criterion_(TARGET),
        divergence_(TOTAL),
        max_expansions_(std::numeric_limits<unsigned>::max()),
        score_aggregation_(MAX),
        random_ties_(true) {
        allocate_open_lists_and_tuple_hash();
    }
    virtual ~iw_bel2_t() { }
    virtual policy_t<T>* clone() const {
        return new iw_bel2_t(pomdp_);
    }
    virtual std::string name() const {
        return std::string("iw-bel(") +
          std::string("width=") + std::to_string(width_) +
          std::string("prune-threshold=") + std::to_string(prune_threshold_) +
          std::string("discretization-parameter=") + std::to_string(discretization_parameter_) +
          std::string(",determinization=") + (determinization_ == MOST_LIKELY ? "most-likely" : "sample") +
          std::string(",stop-criterion=") + (stop_criterion_ == TARGET ? "target" : "reward") +
          std::string(",divergence=") + divergence_name(divergence_) +
          std::string(",max-expansions=") + std::to_string(max_expansions_) +
          std::string(",score-aggregation=") + (score_aggregation_ == MAX ? "max" : "add") +
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
        root->novelty_ = 1;
        const node_t<T> *node = breadth_first_search(root);

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
            std::vector<Problem::action_t> applicable_actions;
            applicable_actions.reserve(pomdp_.number_actions(bel));
            for( Problem::action_t a = 0; a < pomdp_.number_actions(bel); ++a ) {
                if( pomdp_.applicable(bel, a) )
                    applicable_actions.push_back(a);
            }
            if( !applicable_actions.empty() )
                action = applicable_actions[!random_ties_ ? 0 : Random::random(applicable_actions.size())];
        } else {
            // return first action in path
            const node_t<T> *n = node;
            const node_t<T> *parent = node->parent_;
            assert(parent != 0);
            while( parent->parent_ != 0 ) {
#if 0//def EASY//def DEBUG
                std::cout << "action=" << pomdp_.action_name(n->a_) << std::endl;
#endif
                n = parent;
                parent = n->parent_;
            }
            assert(n->parent_->belief_ == bel);
#if 0//def EASY//def DEBUG
            std::cout << "action=" << pomdp_.action_name(n->a_) << std::endl;
#endif
            action = n->a_;
        }
        free_resources();
#if 1//def EASY//def DEBUG
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
            allocate_open_lists_and_tuple_hash();
        }
        it = parameters.find("discretization-parameter");
        if( it != parameters.end() ) discretization_parameter_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("dp");
        if( it != parameters.end() ) discretization_parameter_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("determinization");
        if( it != parameters.end() ) determinization_ = it->second == "sample" ? SAMPLE : MOST_LIKELY;
        it = parameters.find("stop-criterion");
        if( it != parameters.end() ) stop_criterion_ = it->second == "reward" ? REWARD : TARGET;
        it = parameters.find("divergence");
        if( it != parameters.end() ) divergence_ = divergence_from_name(it->second);
        it = parameters.find("max-expansions");
        if( it != parameters.end() ) max_expansions_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("score-aggregation");
        if( it != parameters.end() ) score_aggregation_ = it->second == "max" ? MAX : ADD;
        it = parameters.find("random-ties");
        if( it != parameters.end() ) random_ties_ = it->second == "true";
        policy_t<T>::setup_time_ = 0;
#ifdef DEBUG
        std::cout << "debug: iw-bel(): params:"
                  << " width=" << width_
                  << " prune-threshold=" << prune-threshold_
                  << " discretization-parameter=" << discretization_parameter_
                  << " determinization=" << (determinization_ == MOST_LIKELY ? "most-likely" : "sample")
                  << " stop-criterion=" << (stop_criterion_ == TARGET ? "target" : "reward")
                  << " divergence=" << divergence_name(divergence_)
                  << " max-expansions=" << max_expansions_
                  << " score-aggregation=" << (score_aggregation_ == MAX ? "max" : "add")
                  << " random-ties=" << (random_ties_ ? "true" : "false")
                  << std::endl;
#endif
    }
    virtual typename policy_t<T>::usage_t uses_base_policy() const { return policy_t<T>::usage_t::No; }
    virtual typename policy_t<T>::usage_t uses_heuristic() const { return policy_t<T>::usage_t::No; }
    virtual typename policy_t<T>::usage_t uses_algorithm() const { return policy_t<T>::usage_t::No; }

    void fill_tuples(node_t<T> &node) const {
        fill_tuples(node.belief_, node.tuples_);
    }
    void fill_tuples(const T &belief, std::vector<const int*> &tuples) const {
        // fill tuples of size 1
        if( prune_threshold_ < 1 ) return;
        for( int vid = 0; vid < belief.number_variables(); ++vid ) {
            std::vector<std::pair<int, float> > values_for_vid;
            belief.fill_values_for_variable(vid, values_for_vid);
            for( int i = 0; i < int(values_for_vid.size()); ++i ) {
                int value = values_for_vid[i].first;
                float p = values_for_vid[i].second;
                int dp = ceilf(p * discretization_parameter_);
                assert(dp <= discretization_parameter_);
                int code = (value * (1 + discretization_parameter_) + dp) * belief.number_variables() + vid;
                int *tuple = tuple_factory_.get_tuple(1);
                assert(tuple[0] == 1);
                tuple[1] = code;
                tuples.push_back(tuple);
            }
        }

        // fill tuples of size 2
        if( prune_threshold_ < 2 ) return;
        int value_v0 = belief.value(0);
        assert(value_v0 != -1); // variable 0 is determined // CHECK
        int dp_v0 = discretization_parameter_;
        int code_v0 = (value_v0 * (1 + discretization_parameter_) + dp_v0) * belief.number_variables() + 0;
        for( int vid = 1; vid < belief.number_variables(); ++vid ) {
            //int dsz = belief.domain_size(vid);
            std::vector<std::pair<int, float> > values_for_vid;
            belief.fill_values_for_variable(vid, values_for_vid);
            for( int i = 0; i < int(values_for_vid.size()); ++i ) {
                int value = values_for_vid[i].first;
                float p = values_for_vid[i].second;
                int dp = ceilf(p * discretization_parameter_);
                assert(dp <= discretization_parameter_);
                int code = (value * (1 + discretization_parameter_) + dp) * belief.number_variables() + vid;
                int *tuple = tuple_factory_.get_tuple(2);
                assert(tuple[0] == 2);
                tuple[1] = code_v0;
                tuple[2] = code;
                tuples.push_back(tuple);
            }
        }

#ifdef DEBUG
        std::cout << "[tuples={";
        for( int i = 0; i < int(tuples.size()); ++i ) {
            const int *tuple = tuples[i];
            assert(tuple[0] == 2); // this tuple is a pair

            int code_v0 = tuple[1];
            int vid_v0 = code_v0 % belief.number_variables();
            int value_v0 = (code_v0 / belief.number_variables()) / (1 + discretization_parameter_);
            int dp_v0 = (code_v0 / belief.number_variables()) % (1 + discretization_parameter_);
            assert(vid_v0 == 0);
            assert(dp_v0 == discretization_parameter_); // prob. of (v0=value_v0) = 1

            int code_v1 = tuple[2];
            int vid_v1 = code_v1 % belief.number_variables();
            int value_v1 = (code_v1 / belief.number_variables()) / (1 + discretization_parameter_);
            int dp_v1 = (code_v1 / belief.number_variables()) % (1 + discretization_parameter_);

            std::cout << "<(v" << vid_v0 << "," << value_v0 << "," << dp_v0 << "),(v" << vid_v1 << "," << value_v1 << "," << dp_v1 << ")>";
            if( i + 1 < int(tuples.size()) ) std::cout << ",";
        }
        std::cout << "}]" << std::endl;
#endif
    }

#if 0
    void clear_node_table() const {
        for( typename node_hash_t<T>::const_iterator it = node_table_.begin(); it != node_table_.end(); ++it ) {
            delete it->second;
        }
        node_table_.clear();
    }
    const node_t<T>* lookup_node_in_table(const T &belief) const {
        typename node_hash_t<T>::const_iterator it = node_table_.find(&belief);
        //std::cout << "LOOKUP: bel=" << belief << ", found=" << (it == node_table_.end() ? "NO" : "YES") << std::endl;
        return it == node_table_.end() ? 0 : it->second;
    }
    bool insert_node_in_table(const node_t<T> *node) const {
        //std::cout << "INSERT: bel=" << node->belief_ << std::endl;
        std::pair<typename node_hash_t<T>::iterator, bool> p = node_table_.insert(std::make_pair(&node->belief_, node));
        return p.second;
    }
#endif

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
                tuple_factory_.free_tuples(node->tuples_);
                delete node;
            }
        }

        for( typename std::list<const node_t<T>*>::const_iterator it = closed_list_.begin(); it != closed_list_.end(); ++it ) {
            const node_t<T> *node = *it;
            tuple_factory_.free_tuples(node->tuples_);
            delete node;
        }
        closed_list_.clear();

        for( int i = 0; i < int(tuple_hash_.size()); ++i )
            tuple_hash_[i].clear();
    }
 
    // breadth-first search
    const node_t<T>* breadth_first_search(const node_t<T> *root) const {
        std::vector<std::pair<T, float> > outcomes;

        //insert_node_in_table(root);
        push_node(root);
        for( unsigned iter = 0; !open_lists_are_empty() && (iter < max_expansions_); ++iter ) {
            const node_t<T> *node = select_node_for_expansion();
            closed_list_.push_front(node);
            register_tuples(node->tuples_);
            
#ifdef EASY
            std::cout << "SELECT: node=" << *node << std::endl;
#endif

            // check whether we need to stop
            if( pomdp_.terminal(node->belief_) ) {
#if 0//def DEBUG
                std::cout << "TERMINAL FOUND: bel=" << node->belief_ << std::endl;
#endif
                return node;
            }

            // expand node
            std::vector<std::pair<int, std::pair<int, std::vector<const int*> > > > candidates;
            for( Problem::action_t a = 0; a < pomdp_.number_actions(node->belief_); ++a ) {
                if( pomdp_.applicable(node->belief_, a) ) {
                    // expand node
                    pomdp_.next(node->belief_, a, outcomes);
#ifdef EASY
                    std::cout << "action=" << pomdp_.action_name(a) << ", outcomes.sz=" << outcomes.size() << std::endl;
#endif
                    if( determinization_ == SAMPLE ) {
                        for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
#ifdef EASY
                            std::cout << "    considering outcome " << i << std::flush;
#endif
                            if( pomdp_.dead_end(outcomes[i].first) ) {
#ifdef EASY
                                std::cout << " --> DEAD" << std::endl;
#endif
                                continue;
                            }

                            // prune node if all its tuples already seen
                            std::vector<const int*> tuples;
                            fill_tuples(outcomes[i].first, tuples);
                            int novelty = compute_novelty(tuples);
#ifdef EASY
                            std::cout << " (novelty=" << novelty << ")" << std::flush;
#endif
                            if( novelty > prune_threshold_ ) {
                                tuple_factory_.free_tuples(tuples);
#ifdef EASY
                                std::cout << " --> PRUNED: NOVELTY > threshold (" << prune_threshold_ << ")" << std::endl;
#endif
                                continue;
                            }

                            // save candidate
                            candidates.push_back(std::make_pair(i, std::make_pair(novelty, std::move(tuples))));
                        }
                    } else {
                        for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
#ifdef EASY
                            std::cout << "    considering outcome " << i << std::flush;
#endif
                            if( pomdp_.dead_end(outcomes[i].first) ) {
#ifdef EASY
                                std::cout << " --> DEAD" << std::endl;
#endif
                                continue;
                            }

                            // prune node if all its tuples already seen
                            std::vector<const int*> tuples;
                            fill_tuples(outcomes[i].first, tuples);
                            int novelty = compute_novelty(tuples);
#ifdef EASY
                            std::cout << " (novelty=" << novelty << ")" << std::flush;
#endif
                            if( novelty > prune_threshold_ ) {
                                tuple_factory_.free_tuples(tuples);
#ifdef EASY
                                std::cout << " --> NOVELTY > threshold (" << prune_threshold_ << ")" << std::endl;
#endif
                                continue;
                            }

                            // check if this outcome is most probable (best candidate)
                            if( candidates.empty() || (outcomes[i].second >= outcomes[candidates.back().first].second) ) {
                                if( !candidates.empty() && (outcomes[i].second > outcomes[candidates.back().first].second) )
                                    clear_candidate_list(candidates);
                                candidates.push_back(std::make_pair(i, std::make_pair(novelty, std::move(tuples))));
#ifdef EASY
                                std::cout << " --> CURRENT BEST (p=" << outcomes[i].second << ")" << std::endl;
#endif
                            } else {
                                tuple_factory_.free_tuples(tuples);
#ifdef EASY
                                std::cout << " --> NOT BEST (p=" << outcomes[i].second << ")" << std::endl;
#endif
                            }
                        }
                    }

                    if( !candidates.empty() ) {
                        // compute cdf of candidates for sampling
                        std::vector<float> cdf(candidates.size(), 0);
                        for( int i = 0; i < int(candidates.size()); ++i ) {
                            cdf[i] = i == 0 ? 0 : cdf[i - 1];
                            if( determinization_ == SAMPLE )
                                cdf[i] += outcomes[candidates[i].first].second;
                            else
                                cdf[i] += 1.0 / float(candidates.size());
                        }
                        for( int i = 0; i < int(cdf.size()); ++i )
                            cdf[i] /= cdf.back();

                        // sample from cdf and create new node
                        float cost = pomdp_.cost(node->belief_, a);
                        int sampled = Random::sample_from_distribution(cdf.size(), &cdf[0]);
                        node_t<T> *new_node = get_node(outcomes[candidates[sampled].first].first, node, a, cost);
                        new_node->novelty_ = candidates[sampled].second.first;
                        new_node->tuples_ = candidates[sampled].second.second;
                        push_node(new_node);

                        // clear non selected tuples 
                        candidates[sampled] = candidates.back();
                        candidates.pop_back();
                        clear_candidate_list(candidates);
                    }
                }
            }
        }

        // return best node in open/closed if stop-criterion is REWARD
        if( stop_criterion_ == REWARD ) {
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

            // search closed list when all open-lists are empty
            if( best.empty() ) {
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
#if 0
            const std::list<const node_t<T>*> *list_to_search = open_list_.empty() ? &closed_list_ : &open_list_;
            for( typename std::list<const node_t<T>*>::const_iterator it = list_to_search->begin(); it != list_to_search->end(); ++it ) {
                const node_t<T> *node = *it;
                if( best.empty() || (node->g_ <= best.back()->g_) ) {
                    if( !best.empty() && (node->g_ < best.back()->g_) )
                        best.clear();
                    best.push_back(*it);
                }
            }
#endif
            if( best.empty() ) std::cout << "**** BEST IS EMPTY" << std::endl;
            return best.empty() ? 0 : best[Random::random(best.size())];
        } else {
            return 0;
        }
    }

    void clear_candidate_list(std::vector<std::pair<int, std::pair<int, std::vector<const int*> > > > &candidates) const {
        for( int i = 0; i < int(candidates.size()); ++i )
            tuple_factory_.free_tuples(candidates[i].second.second);
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
        for( int i = 0; i < int(tuples.size()); ++i ) {
            const int *tuple = tuples[i];
            assert((tuple[0] > 0) && (tuple[0] < int(tuple_hash_.size())));
            tuple_hash_[tuple[0]].insert(tuple);
        }
    }

    int compute_novelty(const std::vector<const int*> &tuples) const {
        int novelty = std::numeric_limits<int>::max();
        for( int i = 0; i < int(tuples.size()); ++i ) {
            const int *tuple = tuples[i];
            assert((tuple[0] > 0) && (tuple[0] < int(tuple_hash_.size())));
            tuple_hash_t::const_iterator it = tuple_hash_[tuple[0]].find(tuple);
            if( it == tuple_hash_[tuple[0]].end() )
                novelty = tuple[0] < novelty ? tuple[0] : novelty;
        }
        return novelty;
    }

    void allocate_open_lists_and_tuple_hash() {
        open_lists_.clear();
        open_lists_.reserve(1 + prune_threshold_);
        for( int i = 0; i <= prune_threshold_; ++i )
            open_lists_.push_back(priority_queue_t(min_priority_t()));

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
    bool push_node(const node_t<T> *node) const {
        assert(node->novelty_ >= 1);
        if( node->novelty_ < int(open_lists_.size()) ) {
            open_lists_[node->novelty_].push(node);
            return true;
        } else {
            delete node;
            return false;
        }
    }
    const node_t<T>* select_node_for_expansion() const {
#if 0
#if 0//def DEBUG
        std::cout << "select: ENTRY: open.sz=" << open_list_.size() << ", closed.sz=" << closed_list_.size() << std::endl;
#endif
        std::vector<typename std::list<const node_t<T>*>::iterator> best_nodes;
        float best_score = std::numeric_limits<float>::min();
        for( typename std::list<const node_t<T>*>::iterator it = open_list_.begin(); it != open_list_.end(); ++it ) {
            const node_t<T> &node = **it;
            assert(!pomdp_.dead_end(node.belief_));
            int score = compute_novelty(node.tuples_);
            if( best_nodes.empty() || (score >= best_score) ) {
                if( score > best_score ) best_nodes.clear();
                best_score = score;
                best_nodes.push_back(it);
            }
        }
#if 0//def DEBUG
        std::cout << "    best: #=" << best_nodes.size() << ", score=" << best_score << std::endl,
#endif
        assert(!best_nodes.empty());
        typename std::list<const node_t<T>*>::iterator best = best_nodes[!random_ties_ ? 0 : Random::random(best_nodes.size())];
        const node_t<T> *node = *best;
        open_list_.erase(best);
#if 0//def DEBUG
        std::cout << "select: EXIT: open.sz=" << open_list_.size() << ", closed.sz=" << closed_list_.size() << ", node-ptr=" << node << ", score=" << best_score << ", node=" << *node << std::endl;
#endif
        return node;
#endif

#if 0
        const node_t<T> *node = open_list_.front();
        open_list_.pop_front();
        return node;
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

    float score(const node_t<T> &n1, const node_t<T> &n2) const {
#if 0
        assert(n1.feature_ != 0);
        assert(n2.feature_ != 0);

        float score_on_marginals = 0;
        assert(n1.feature_->number_marginals_ == n2.feature_->number_marginals_);
        for( int i = 0; i < n1.feature_->number_marginals_; ++i ) {
            float s = score(n1.feature_->marginals_[i], n2.feature_->marginals_[i]);
            score_on_marginals = s > score_on_marginals ? s : score_on_marginals;
        }

        float score_on_fixed_tuples = 0;
        assert(n1.feature_->number_fixed_tuples_ == n2.feature_->number_fixed_tuples_);
        for( int i = 0; i < n1.feature_->number_fixed_tuples_; ++i ) {
            float s = score(n1.feature_->fixed_tuples_[i], n2.feature_->fixed_tuples_[i]);
            score_on_fixed_tuples = s > score_on_fixed_tuples ? s : score_on_fixed_tuples;
        }

        //std::cout << "score-ft=" << score_on_fixed_tuples << std::endl;
        return score_on_marginals + score_on_fixed_tuples;
#endif
    }
    float score(const std::vector<float> &d1, const std::vector<float> &d2) const {
        if( divergence_ == TOTAL )
            return score_total(d1, d2);
        else if( divergence_ == KL )
            return score_kl(d1, d2, false);
        else if( divergence_ == KL_SYM )
            return score_kl(d1, d2, true);
        else if( divergence_ == JS )
            return score_js(d1, d2);
        else
            return 0;
    }
    float score_total(const std::vector<float> &p, const std::vector<float> &q) const {
        return Random::total_divergence(p, q);
    }
    float score_kl(const std::vector<float> &p, const std::vector<float> &q, bool symmetric) const {
        return Random::kl_divergence(p, q, symmetric);
    }
    float score_js(const std::vector<float> &p, const std::vector<float> &q) const {
        return Random::js_divergence(p, q);
    }

    float score(const std::vector<int> &t1, const std::vector<int> &t2) const {
        assert(t1.size() == t2.size());
        int s = 0;
        for( int i = 0, isz = int(t1.size()); i < isz; ++i )
            s += t1[i] != t2[i] ? 1 : 0;
        return float(s) / float(t1.size());
    }
};

}; // namespace IWBel2

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

