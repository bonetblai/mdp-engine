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
#include <cassert>
#include <limits>
#include <vector>
#include <math.h>

//#define DEBUG

namespace Policy {

#if 0

template<typename T> struct node_t {
    const T &state_;
    float value_;
    const node_t *parent_;
    int visits_;
    std::vector<node_t*> children_;
    std::vector<int> counts_;
    node_t(const T &s, int num_actions)
      : state_(s), value_(0), parent_(0), visits_(0),
        children_(num_actions, 0),
        counts_(num_actions, 0) {
    } 
    ~node_t() { }

    float uct_trial(const Problem::problem_t<T> &problem, float C) {
        float best_qvalue = std::numeric_limits<float>::max();
        Problem::action_t best_action = Problem::noop;

        // calculate best child
        float log_n_s = logf(visits_);
        for( int a = 0; a < problem.number_actions(); ++a ) {
            float qv = 0;
            if( children_[a] != 0 ) {
                float bonus = C * sqrtf(log_ns / counts_[a]);
                qv = children_[a]->value_ / visits_ + bonus;
            } else {
                qv = problem.cost(state_, a);
            }

            // select best child
            if( qv < qbest_value ) {
                best_action = a;
                best_qvalue = value;
            }
        }

        // recurse on best child
        if( children_[best_action] != 0 ) {
            float leaf_value = children_[best_action]->uct_trial(problem, C);
            leaf_value += problem.cost(state_, best_action);
            value_ += leaf_value; // update stored value
            ++counts_[best_action]; // increase number of times action is pulled
        } else {
            // create a new node initilizing its value to a single sample
            node_t<T> *node = new node_t<T>(problem.num_actions());
            
        }

        // increase number of visits
        ++visits_;
    }
};
#endif

struct data_t {
    std::vector<float> values_;
    std::vector<int> counts_;
    data_t(const std::vector<float> &values, const std::vector<int> &counts)
      : values_(values), counts_(counts) { }
};


template<typename T> class hash_function_t {
  public:
    size_t operator()(const std::pair<unsigned, T> &p) const { return p.second.hash(); }
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

template<typename T> class mcts_t : public improvement_t<T> {
  protected:
    unsigned width_;
    unsigned depth_;
    float uct_parameter_;
    mutable mcts_table_t<T> table_;

  public:
    mcts_t(const Problem::problem_t<T> &problem, const policy_t<T> &base_policy, unsigned width, unsigned depth)
      : improvement_t<T>(problem, base_policy), width_(width), depth_(depth) {
      uct_parameter_ = -3;
    }
    virtual ~mcts_t() { }

    virtual Problem::action_t operator()(const T &s) const {
        table_.clear();
        for( unsigned i = 0; i < width_; ++i ) {
            search_tree(s, 0);
        }
        typename mcts_table_t<T>::iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        return select_action(it->second, false);
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
        std::cout << std::setw(2*depth) << "" << "search(" << s << "):";
#endif
        if( policy_t<T>::problem().terminal(s) ) {
#ifdef DEBUG
            std::cout << " end" << std::endl;
#endif
            return 0;
        }

        typename mcts_table_t<T>::iterator it = table_.find(std::make_pair(depth, s));
        if( it == table_.end() ) {
            std::vector<float> values(1 + policy_t<T>::problem().number_actions(), 0);
            std::vector<int> counts(1 + policy_t<T>::problem().number_actions(), 0);
            float value = evaluate(s);
            //values[0] = value;
            //counts[0] = 1;
            table_.insert(std::make_pair(std::make_pair(depth, s), data_t(values, counts)));
#ifdef DEBUG
            std::cout << " insert " << value << std::endl;
#endif
            return value;
        } else if( depth == depth_ ) {
#ifdef DEBUG
            std::cout << " count=" << it->second.counts_[0]
                      << " fetch " << it->second.values_[0]
                      << " return" << std::endl;
#endif
            //++it->second.counts_[0];
            return it->second.values_[0];
        } else {
            Problem::action_t a = select_action(it->second, depth);
            ++it->second.counts_[0];
            ++it->second.counts_[1+a];
            std::pair<const T&, bool> p = policy_t<T>::problem().sample(s, a);
            float cost = policy_t<T>::problem().cost(s, a);
#ifdef DEBUG
            std::cout << " count=" << it->second.counts_[0]-1
                      << " fetch " << it->second.values_[0]
                      << " act=" << a
                      << " next=" << p.first
                      << std::endl;
#endif
            float new_value = cost + .95 * search_tree(p.first, 1 + depth);
            float &old_value = it->second.values_[1+a];
            old_value += (new_value - old_value) / (float)it->second.counts_[1+a];
            // CHECK: learning rate is 0.5, it could be 1/n(s,a)
            return old_value;
        }
    }

#if 0
def search(self, state, depth = 0):
    if self.is_terminal(state): return 0
    action = self.select_action(state)
    new_state, cost = self.sample_next_state(state, action)

    if random() < 1.0/(self.state_visit_counts[(state)] + 1):
        try: q = cost + self.gamma*self.Q[(new_state, action)]
        except KeyError: q = cost + self.gamma*self.V_approx[new_state]
    else:
        q = cost + self.gamma*self.search(new_state, depth + 1)
    assert q != 0

    self.state_visit_counts[(state)] += 1
    self.nr_samples += 1
    self.state_action_counts[(state, action)] += 1

    try:
        old_average = self.Q[(state, action)]
        n = self.state_action_counts[(state, action)]
        #new_average = old_average + (1.0/n)*(q - old_average)
        new_average = old_average + (0.5)*(q - old_average)
        except KeyError:
            new_average = q

    self.Q[(state, action)] = new_average
    return q
#endif

    Problem::action_t select_action(const data_t &data, int depth, bool add_bonus = true) const {
        //if( Random::real() < .1 ) return Random::uniform(policy_t<T>::problem().number_actions());
        float log_ns = logf(data.counts_[0]);
        Problem::action_t best_action = Problem::noop;
        float best_value = std::numeric_limits<float>::max();
        for( Problem::action_t a = 0; a < policy_t<T>::problem().number_actions(); ++a ) {
            if( data.counts_[1+a] == 0 ) {
#ifdef DEBUG
                std::cout << " (empty count)";
#endif
                return a;
            }
            assert(data.counts_[0] > 0);
            float bonus = add_bonus ? uct_parameter_ * sqrtf(log_ns / data.counts_[1+a]) : 0;
            float value = data.values_[1+a] + bonus;
            if( value < best_value ) {
                best_value = value;
                best_action = a;
            }
        }
        assert(best_action != Problem::noop);
        return best_action;
    }

    float evaluate(const T &s) const {
        return 0;//evaluation(improvement_t<T>::base_policy_, s, width_, depth_);
    }
};

}; // namespace Policy

#undef DEBUG

#endif

