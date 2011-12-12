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

#ifndef MCTS_H
#define MCTS_H

#include "policy.h"

#include <iostream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <vector>
#include <math.h>

//#define DEBUG

namespace Policy {

template<typename T> class hash_function_t {
  public:
    size_t operator()(const std::pair<Problem::action_t, T> &p) const { return p.second.hash(); }
    size_t operator()(const std::pair<unsigned, T> &p) const { return p.second.hash(); }
};


////////////////////////////////////////////////


template<typename T> struct node_t;
template<typename T> class mcts_hash_t : public Hash::generic_hash_map_t<std::pair<Problem::action_t, T>, const node_t<T>*, hash_function_t<T> > {
  public:
    typedef typename Hash::generic_hash_map_t<std::pair<Problem::action_t, T>, const node_t<T>*, hash_function_t<T> > base_type;
    typedef typename base_type::const_iterator const_iterator;
    const_iterator begin() const { return base_type::begin(); }
    const_iterator end() const { return base_type::end(); }

  public:
    mcts_hash_t() { }
    virtual ~mcts_hash_t() { }
    void print(std::ostream &os) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            os << "(" << it->first.first << "," << it->first.second << ")" << std::endl;
        }
    }
};

template<typename T> struct node_t {
    mutable std::vector<unsigned> counts_;
    mutable std::vector<float> values_;
    mutable mcts_hash_t<T> children_;
    node_t(int num_actions)
      : counts_(1+num_actions, 0),
        values_(1+num_actions, 0) {
    } 
    ~node_t() { }

    Problem::action_t select_action2(const Problem::problem_t<T> &problem, float uct_parameter) const {
        float log_ns = logf(counts_[0]);
        Problem::action_t best_action = Problem::noop;
        float best_value = std::numeric_limits<float>::max();
        for( Problem::action_t a = 0; a < problem.number_actions(); ++a ) {
            if( counts_[1+a] == 0 ) return a;
            assert(counts_[0] > 0);
            float bonus = uct_parameter * sqrtf(log_ns / counts_[1+a]);
            float value = values_[1+a] + bonus;
            if( value < best_value ) {
                best_value = value;
                best_action = a;
            }
        }
        assert(best_action != Problem::noop);
        return best_action;
    }
};


////////////////////////////////////////////////


struct data_t {
    std::vector<float> values_;
    std::vector<int> counts_;
    data_t(const std::vector<float> &values, const std::vector<int> &counts)
      : values_(values), counts_(counts) { }
};

template<typename T> class mcts_table_t : public Hash::generic_hash_map_t<std::pair<unsigned, T>, data_t, hash_function_t<T> > {
  public:
    typedef typename Hash::generic_hash_map_t<std::pair<unsigned, T>, data_t, hash_function_t<T> > base_type;
    typedef typename base_type::const_iterator const_iterator;
    const_iterator begin() const { return base_type::begin(); }
    const_iterator end() const { return base_type::end(); }

  public:
    mcts_table_t() { }
    virtual ~mcts_table_t() { }
    void print(std::ostream &os) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            os << "(" << it->first.first << "," << it->first.second << ")" << std::endl;
        }
    }
};


////////////////////////////////////////////////


template<typename T> class mcts_t : public improvement_t<T> {
  protected:
    unsigned width_;
    unsigned depth_bound_;
    float uct_parameter_;
    mutable mcts_table_t<T> table_;

  public:
    mcts_t(const policy_t<T> &base_policy, unsigned width, unsigned depth_bound, float uct_parameter)
      : improvement_t<T>(base_policy),
        width_(width),
        depth_bound_(depth_bound),
        uct_parameter_(uct_parameter) {
        number_nodes_ = 0;
    }
    virtual ~mcts_t() { }
    virtual const policy_t<T>* clone() const {
        return new mcts_t(improvement_t<T>::base_policy_, width_, depth_bound_, uct_parameter_);
    }

    virtual Problem::action_t operator()(const T &s) const {
        table_.clear();
        for( unsigned i = 0; i < width_; ++i ) {
            search_tree(s, 0);
        }
        typename mcts_table_t<T>::iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        Problem::action_t action = select_action(s, it->second, 0, false);
        assert(policy_t<T>::problem().applicable(s, action));
        return action;
    }

    float value(const T &s, Problem::action_t a) const {
        typename mcts_table_t<T>::const_iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        return it->second.values_[1+a];
    }
    unsigned count(const T &s, Problem::action_t a) const {
        typename mcts_table_t<T>::const_iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        return it->second.counts_[1+a];
    }
    size_t size() const { return table_.size(); }
    void print_table(std::ostream &os) const {
        table_.print(os);
    }

    float search_tree(const T &s, unsigned depth) const {
#ifdef DEBUG
        std::cout << std::setw(2*depth) << "" << "search_tree(" << s << "):";
#endif
        if( (depth == depth_bound_) || policy_t<T>::problem().terminal(s) ) {
#ifdef DEBUG
            std::cout << " end" << std::endl;
#endif
            return 0;
        }

        if( policy_t<T>::problem().dead_end(s) ) {
            return policy_t<T>::problem().dead_end_value();
        }

        typename mcts_table_t<T>::iterator it = table_.find(std::make_pair(depth, s));

        if( it == table_.end() ) {
            std::vector<float> values(1 + policy_t<T>::problem().number_actions(s), 0);
            std::vector<int> counts(1 + policy_t<T>::problem().number_actions(s), 0);
            table_.insert(std::make_pair(std::make_pair(depth, s), data_t(values, counts)));
#ifdef DEBUG
            std::cout << " insert 0" << std::endl;
#endif
            float value = evaluate(s, depth);
            return value;
        } else {
            // select action for this node and increase counts
            Problem::action_t a = select_action(s, it->second, depth, true);
            ++it->second.counts_[0];
            ++it->second.counts_[1+a];

            // sample next state
            std::pair<const T, bool> p = policy_t<T>::problem().sample(s, a);
            float cost = policy_t<T>::problem().cost(s, a);

#ifdef DEBUG
            std::cout << " count=" << it->second.counts_[0]-1
                      << " fetch " << std::setprecision(5) << it->second.values_[1+a]
                      << " act=" << a
                      << " next=" << p.first
                      << std::endl;
#endif

            // do recursion and update value
            float &old_value = it->second.values_[1+a];
            float n = it->second.counts_[1+a];
            float new_value = cost + policy_t<T>::problem().discount() * search_tree(p.first, 1 + depth);
            old_value += (new_value - old_value) / n;
            return old_value;
        }
    }

    Problem::action_t select_action(const T &state, const data_t &data, int depth, bool add_bonus) const {
        float log_ns = logf(data.counts_[0]);
        Problem::action_t best_action = Problem::noop;
        float best_value = std::numeric_limits<float>::max();

        for( Problem::action_t a = 0; a < policy_t<T>::problem().number_actions(state); ++a ) {
            if( policy_t<T>::problem().applicable(state, a) ) {
                // if this action has never been taken in this node, select it
                if( data.counts_[1+a] == 0 ) {
#ifdef DEBUG
                    std::cout << " (empty count)";
#endif
                    return a;
                }

                // compute score of action adding bonus (if applicable)
                assert(data.counts_[0] > 0);
                float bonus = add_bonus ? uct_parameter_ * sqrtf(log_ns / data.counts_[1+a]) : 0;
                float value = data.values_[1+a] + bonus;

                // update best action so far
                if( value < best_value ) {
                    best_value = value;
                    best_action = a;
                }
            }
        }
        assert(best_action != Problem::noop);
        return best_action;
    }

    unsigned number_nodes_;
    unsigned SIZE() const { return number_nodes_; }

    // different states at same depth are treated as different nodes in the tree
    float search(const T &s, const node_t<T> *node, unsigned depth) {
#ifdef DEBUG
        std::cout << std::setw(2*depth) << "" << "search(" << s << "):";
#endif
        if( (depth == depth_bound_) || policy_t<T>::problem().terminal(s) ) {
#ifdef DEBUG
            std::cout << " end" << std::endl;
#endif
            return 0;
        } else if( policy_t<T>::problem().dead_end(s) ) {
            assert(0);
        } else {
            // select action for this node and increase counts
            Problem::action_t a = node->select_action2(policy_t<T>::problem(), uct_parameter_);
            ++node->counts_[0];
            ++node->counts_[1+a];

            // sample next state
            std::pair<const T, bool> p = policy_t<T>::problem().sample(s, a);
            typename mcts_hash_t<T>::const_iterator it = node->children_.find(std::make_pair(a, p.first));
            if( it == node->children_.end() ) {
                ++number_nodes_;
                node_t<T> *new_child = new node_t<T>(policy_t<T>::problem().number_actions(p.first));
                node->children_.insert(std::make_pair(std::make_pair(a, p.first), new_child));
#ifdef DEBUG
                std::cout << " insert " << p.first << " w/ value=" << 0 << std::endl;
#endif
                return evaluate(p.first);
            } else {
                const node_t<T> *child = it->second;
                float cost = policy_t<T>::problem().cost(s, a);

#ifdef DEBUG
                std::cout << " counts=(" << node->counts_[0] << "," << node->counts_[1+a] << ")"
                          << " value=" << node->values_[1+a] << std::endl;
#endif

                // do recursion and update value
                float new_value = cost + policy_t<T>::problem().discount() * search(p.first, child, 1+depth);
                float &old_value = node->values_[1+a];
                old_value += (new_value - old_value) / (float)node->counts_[1+a];
                return old_value;
            }
        }
    }

    mcts_hash_t<T> root_hash_;
    float SEARCH(const T &s) {
        typename mcts_hash_t<T>::const_iterator it = root_hash_.find(std::make_pair(0, s));
        if( it == root_hash_.end() ) {
            node_t<T> *node = new node_t<T>(policy_t<T>::problem().number_actions(s));
            it = root_hash_.insert(std::make_pair(std::make_pair(0, s), node)).first;
        }
        return search(s, it->second, 0);
    }
    float VALUE(const T &s, Problem::action_t a) const {
        typename mcts_hash_t<T>::const_iterator it = root_hash_.find(std::make_pair(0, s));
        assert(it != root_hash_.end());
        return it->second->values_[1+a];
    }
    unsigned COUNT(const T &s, Problem::action_t a) const {
        typename mcts_hash_t<T>::const_iterator it = root_hash_.find(std::make_pair(0, s));
        assert(it != root_hash_.end());
        return it->second->counts_[1+a];
    }
        

    float evaluate(const T &s, unsigned depth) const {
        return Evaluation::evaluation(improvement_t<T>::base_policy_, s, 1, depth_bound_ - depth);
    }
};

}; // namespace Policy

#undef DEBUG

#endif

