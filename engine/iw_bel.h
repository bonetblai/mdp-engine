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

#define DEBUG

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

    void print(std::ostream &os) const {
        os << "[bel=" << belief_ << ",a=" << a_ << ",g=" << g_ << ",p=" << parent_ << "]";
    }
};

template<typename T> inline std::ostream& operator<<(std::ostream &os, const node_t<T> &node) {
    node.print(os);
    return os;
}

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
    typedef typename base_type::iterator iterator;

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
  public:
    typedef enum { MOST_LIKELY, SAMPLE } determinization_t;
    typedef enum { REWARD, TARGET } stop_criterion_t;
    typedef enum { TOTAL, KL, KL_SYMMETRIC } divergence_t;

  protected:
    const POMDP::pomdp_t<T> &pomdp_;

    unsigned width_;
    determinization_t determinization_;
    stop_criterion_t stop_criterion_;
    divergence_t divergence_;
    unsigned max_expansions_;

    mutable node_hash_t<T> node_table_;

    struct min_priority_t {
        bool operator()(const node_t<T> *n1, const node_t<T> *n2) const {
            return n1->g_ > n2->g_;
        }
    };
    typedef typename std::priority_queue<const node_t<T>*, std::vector<const node_t<T>*>, min_priority_t> priority_queue_t;

    iw_bel_t(const POMDP::pomdp_t<T> &pomdp,
             unsigned width,
             determinization_t determinization,
             stop_criterion_t stop_criterion,
             divergence_t divergence,
             unsigned max_expansions,
             float parameter,
             bool random_ties)
      : policy_t<T>(pomdp),
        pomdp_(pomdp),
        width_(width),
        determinization_(determinization),
        stop_criterion_(stop_criterion),
        divergence_(divergence),
        max_expansions_(max_expansions) {
    }

  protected:


  public:
    iw_bel_t(const POMDP::pomdp_t<T> &pomdp)
      : policy_t<T>(pomdp),
        pomdp_(pomdp),
        width_(0),
        determinization_(MOST_LIKELY),
        stop_criterion_(TARGET),
        divergence_(TOTAL),
        max_expansions_(std::numeric_limits<unsigned>::max()) {
    }
    virtual ~iw_bel_t() { }
    virtual policy_t<T>* clone() const {
        return new iw_bel_t(pomdp_);
    }
    virtual std::string name() const {
        return std::string("iw-bel(") +
          std::string("width=") + std::to_string(width_) +
          std::string(",determinization=") + std::to_string(determinization_) +
          std::string(",stop-criterion=") + std::to_string(stop_criterion_) +
          std::string(",divergence=") + std::to_string(divergence_) +
          std::string(",max-expansions=") + std::to_string(max_expansions_) +
          std::string(")");
    }

    Problem::action_t operator()(const T &bel) const {
        std::cout << "bel=" << bel << std::endl;
        std::cout << "cardinality=" << bel.cardinality() << std::endl;
        std::cout << "throwing BFS for goal" << std::endl;

        if( pomdp_.dead_end(bel) ) return Problem::noop;

        const POMDP::feature_t<T> *feature = pomdp_.get_feature(bel);
        node_t<T> *root = get_root_node(bel, feature);
        //const node_t<T> *goal = pruned_bfs(root);
        const node_t<T> *goal = breadth_first_search(root);

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
            std::cout << "BEST=" << node->a_ << std::endl;
            return node->a_;
        }
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
        it = parameters.find("determinization");
        if( it != parameters.end() ) determinization_ = it->second == "sampling" ? SAMPLE : MOST_LIKELY;
        it = parameters.find("stop-criterion");
        if( it != parameters.end() ) stop_criterion_ = it->second == "reward" ? REWARD : TARGET;
        it = parameters.find("divergence");
        if( it != parameters.end() ) divergence_ = it->second == "total" ? TOTAL : (it->second == "kl" ? KL : KL_SYMMETRIC);
        it = parameters.find("max-expansions");
        if( it != parameters.end() ) max_expansions_ = strtol(it->second.c_str(), 0, 0);
#if 0
        it = parameters.find("random-ties");
        if( it != parameters.end() ) random_ties_ = it->second == "true";
#endif
        policy_t<T>::setup_time_ = 0;
#ifdef DEBUG
        std::cout << "debug: iw-bel(): params:"
                  << " width=" << width_
                  << " determinization=" << determinization_
                  << " stop-criterion=" << stop_criterion_
                  << " divergence=" << divergence_
                  << " max-expansions=" << max_expansions_
                  << std::endl;
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

            // check whether we need to stop
            if( pomdp_.terminal(node->belief_) ) {
                return node;
            }

            // expand node
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

    // breadth-first search
    const node_t<T>* breadth_first_search(const node_t<T> *root) const {
        std::list<const node_t<T>*> open_list, closed_list;
        std::vector<std::pair<T, float> > outcomes;

        //clear_feature_table();
        clear_node_table();

        //insert_feature(*root->feature_);
        insert_node(root);
        open_list.push_back(root);
        for( unsigned iter = 0; !open_list.empty() && ((stop_criterion_ != TARGET) || (iter < max_expansions_)); ++iter ) {
            const node_t<T> *node = select_node_for_expansion(open_list, closed_list);
            closed_list.push_front(node);

            // check whether we need to stop
            if( (stop_criterion_ == TARGET) && pomdp_.terminal(node->belief_) ) {
                return node;
            }

            // expand node
            for( Problem::action_t a = 0; a < pomdp_.number_actions(node->belief_); ++a ) {
                if( pomdp_.applicable(node->belief_, a) ) {
                    node_t<T> *new_node = 0;
                    const POMDP::feature_t<T> *feature = 0;
                    float cost = pomdp_.cost(node->belief_, a);

                    // determinize the next belief
                    if( determinization_ == SAMPLE ) {
                        std::pair<const T, bool> p = pomdp_.sample(node->belief_, a);
                        feature = pomdp_.get_feature(p.first);
                        new_node = get_node(p.first, feature, node, a, cost);
                    } else {
                        pomdp_.next(node->belief_, a, outcomes);

                        // select best (unexpanded) successors to continue search
                        int best = -1;
                        for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
                            if( pomdp_.dead_end(outcomes[i].first) ) continue;
                            if( lookup_node(outcomes[i].first) != 0 ) continue;
                            if( (best == -1) || (outcomes[i].second > outcomes[best].second) ) {
                                best = i;
                            }
                        }
                        assert((best >= 0) && (best < outcomes.size()));
                        feature = pomdp_.get_feature(outcomes[best].first);
                        new_node = get_node(outcomes[best].first, feature, node, a, cost);
                    }

                    // insert new node into open list
                    open_list.push_back(new_node);
                }
            }
        }
        return 0;
    }

    const node_t<T>* select_node_for_expansion(std::list<const node_t<T>*> &open_list, const std::list<const node_t<T>*> &closed_list) const {
        std::cout << "hola: open.sz=" << open_list.size() << ", closed.sz=" << closed_list.size() << std::endl;
        typename std::list<const node_t<T>*>::iterator best;
        float best_score = std::numeric_limits<float>::min();
        for( typename std::list<const node_t<T>*>::iterator it = open_list.begin(); it != open_list.end(); ++it ) {
            float max_score = std::numeric_limits<float>::min();
            for( typename std::list<const node_t<T>*>::const_iterator jt = closed_list.begin(); jt != closed_list.end(); ++jt ) {
                float s = score(**it, **jt);
                if( s > max_score ) max_score = s;
            }
            if( (best_score == std::numeric_limits<float>::min()) || (max_score > best_score) ) {
                best_score = max_score;
                best = it;
            }
        }
        const node_t<T> *node = *best;
        open_list.erase(best);
        std::cout << "select: open.sz=" << open_list.size() << ", closed.sz=" << closed_list.size() << ", node-ptr=" << node << ", score=" << best_score << ", node=" << *node << std::endl;
        return node;
    }

    float score(const node_t<T> &n1, const node_t<T> &n2) const {
        assert(n1.feature_ != 0);
        assert(n2.feature_ != 0);
        const std::vector<std::vector<float> > &m1 = n1.feature_->marginals_;
        const std::vector<std::vector<float> > &m2 = n2.feature_->marginals_;
        assert(m1.size() == m2.size());

        float max_score = 0;
        for( int i = 0, isz = int(m1.size()); i < isz; ++i ) {
            float s = score(m1[i], m2[i]);
            max_score = s > max_score ? s : max_score;
        }
        return max_score;
    }
    float score(const std::vector<float> &d1, const std::vector<float> &d2) const {
        return divergence_ == TOTAL ? score_total(d1, d2) : score_kl(d1, d2, divergence_ == KL_SYMMETRIC);
    }
    float score_total(const std::vector<float> &p, const std::vector<float> &q) const {
        assert(p.size() == q.size());
        float score = 0;
        for( int i = 0, isz = int(p.size()); i < isz; ++i )
            score += fabs(p[i] - q[i]);
        return score / 2;
    }
    float score_kl(const std::vector<float> &p, const std::vector<float> &q, bool symmetric) const {
        assert(p.size() == q.size());
        float h_p = 0, h_pq = 0, h_q = 0, h_qp = 0;
        for( int i = 0, isz = int(p.size()); i < isz; ++i ) {
            h_p += p[i] * log2f(p[i]);
            h_pq += p[i] * log2f(q[i]);
            if( symmetric ) {
                h_q += q[i] * log2f(q[i]);
                h_qp += q[i] * log2f(p[i]);
            }
        }
        return (h_p - h_pq) + (h_q - h_qp);
    }
};

}; // namespace IWBel

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

