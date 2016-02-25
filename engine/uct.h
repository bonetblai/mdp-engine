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

#ifndef UCT_H
#define UCT_H

#include "policy.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <vector>
#include <math.h>

//#define DEBUG

namespace Online {

namespace Policy {

namespace UCT {

////////////////////////////////////////////////
//
// Tree
//

template<typename T> struct node_t {
    mutable std::vector<unsigned> counts_;
    mutable std::vector<float> values_;
    node_t(int num_actions)
      : counts_(1+num_actions, 0),
        values_(1+num_actions, 0) {
    } 
    ~node_t() { }
};

////////////////////////////////////////////////
//
// Hash Table
//

template<typename T> struct map_functions_t {
    size_t operator()(const std::pair<unsigned, T> &p) const {
        return p.second.hash();
    }
};

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

template<typename T> class hash_t :
  public Hash::generic_hash_map_t<std::pair<unsigned, T>,
                                  data_t,
                                  map_functions_t<T> > {
  public:
    typedef typename Hash::generic_hash_map_t<std::pair<unsigned, T>,
                                              data_t,
                                              map_functions_t<T> >
            base_type;
    typedef typename base_type::const_iterator const_iterator;
    const_iterator begin() const { return base_type::begin(); }
    const_iterator end() const { return base_type::end(); }

  public:
    hash_t() { }
    virtual ~hash_t() { }
    void print(std::ostream &os) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            os << "(" << it->first.first << "," << it->first.second << ")" << std::endl;
        }
    }
};

////////////////////////////////////////////////
//
// Policy
//

template<typename T> class uct_t : public improvement_t<T> {
  using policy_t<T>::problem_;
  using improvement_t<T>::base_policy_;
  protected:
    unsigned width_;
    unsigned horizon_;
    float parameter_;
    bool random_ties_;
    mutable hash_t<T> table_;

    uct_t(const Problem::problem_t<T> &problem,
          const policy_t<T> *base_policy,
          unsigned width,
          unsigned horizon,
          float parameter,
          bool random_ties)
      : improvement_t<T>(problem, base_policy),
        width_(width),
        horizon_(horizon),
        parameter_(parameter),
        random_ties_(random_ties) {
    }

  public:
    uct_t(const Problem::problem_t<T> &problem)
      : improvement_t<T>(problem, 0),
        width_(0), horizon_(0), parameter_(0), random_ties_(false) {
    }
    virtual ~uct_t() { }
    virtual policy_t<T>* clone() const {
        return new uct_t(problem_, base_policy_, width_, horizon_, parameter_, random_ties_);
    }
    virtual std::string name() const {
        return std::string("uct(policy=") + (base_policy_ == 0 ? std::string("null") : base_policy_->name()) +
          std::string(",width=") + std::to_string(width_) +
          std::string(",horizon=") + std::to_string(horizon_) +
          std::string(",parameter=") + std::to_string(parameter_) +
          std::string(",random-ties=") + (random_ties_ ? "true" : "false") + ")";
    }

    virtual Problem::action_t operator()(const T &s) const {
        if( base_policy_ == 0 ) {
            std::cout << Utils::error() << "(base) policy must be specified for uct() policy!" << std::endl;
            exit(1);
        }

        ++policy_t<T>::decisions_;
        if( horizon_ == 0 ) {
            return (*base_policy_)(s);
        } else {
            table_.clear();
            for( unsigned i = 0; i < width_; ++i )
                search_tree(s, 0);
            typename hash_t<T>::iterator it = table_.find(std::make_pair(0, s));
            assert(it != table_.end());
            Problem::action_t action = select_action(s, it->second, 0, false, random_ties_);
            assert(problem_.applicable(s, action));
            return action;
        }
    }
    virtual void reset_stats() const {
        policy_t<T>::setup_time_ = 0;
        policy_t<T>::base_policy_time_ = 0;
        policy_t<T>::heuristic_time_ = 0;
        problem_.clear_expansions();
        if( base_policy_ != 0 ) base_policy_->reset_stats();
    }
    virtual void print_other_stats(std::ostream &os, int indent) const {
        os << std::setw(indent) << ""
           << "other-stats: name=" << name()
           << " decisions=" << policy_t<T>::decisions_
           << std::endl;
        if( base_policy_ != 0 ) base_policy_->print_other_stats(os, 2 + indent);
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
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
        std::cout << "debug: uct(): params:"
                  << " width=" << width_
                  << " horizon=" << horizon_
                  << " parameter=" << parameter_
                  << " random-ties=" << (random_ties_ ? "true" : "false")
                  << " policy=" << (base_policy_ == 0 ? std::string("null") : base_policy_->name())
                  << std::endl;
#endif
    }
    virtual typename policy_t<T>::usage_t uses_base_policy() const { return policy_t<T>::usage_t::Yes; }
    virtual typename policy_t<T>::usage_t uses_heuristic() const { return policy_t<T>::usage_t::No; }
    virtual typename policy_t<T>::usage_t uses_algorithm() const { return policy_t<T>::usage_t::No; }

    float value(const T &s, Problem::action_t a) const {
        typename hash_t<T>::const_iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        return it->second.values_[1+a];
    }
    unsigned count(const T &s, Problem::action_t a) const {
        typename hash_t<T>::const_iterator it = table_.find(std::make_pair(0, s));
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

        if( (depth == horizon_) || problem_.terminal(s) ) {
#ifdef DEBUG
            std::cout << " end" << std::endl;
#endif
            return 0;
        }

        if( problem_.dead_end(s) ) {
#ifdef DEBUG
            std::cout << " dead-end" << std::endl;
#endif
            return problem_.dead_end_value();
        }

        typename hash_t<T>::iterator it = table_.find(std::make_pair(depth, s));

        if( it == table_.end() ) {
            std::vector<float> values(1 + problem_.number_actions(s), 0);
            std::vector<int> counts(1 + problem_.number_actions(s), 0);
            table_.insert(std::make_pair(std::make_pair(depth, s), data_t(values, counts)));
            float value = evaluate(s, depth);
#ifdef DEBUG
            std::cout << " insert in tree w/ value=" << value << std::endl;
#endif
            return value;
        } else {
            // select action for this node and increase counts
            Problem::action_t a = select_action(s, it->second, depth, true, random_ties_);
            ++it->second.counts_[0];
            ++it->second.counts_[1+a];

            // sample next state
            std::pair<const T, bool> p = problem_.sample(s, a);
            float cost = problem_.cost(s, a);

#ifdef DEBUG
            std::cout << " count=" << it->second.counts_[0]-1
                      << " fetch " << std::setprecision(5) << it->second.values_[1+a]
                      << " a=" << a
                      << " next=" << p.first
                      << std::endl;
#endif

            // do recursion and update value
            float &old_value = it->second.values_[1+a];
            float n = it->second.counts_[1+a];
            float new_value = cost + problem_.discount() * search_tree(p.first, 1 + depth);
            old_value += (new_value - old_value) / n;
            return new_value;
        }
    }

    Problem::action_t select_action(const T &state,
                                    const data_t &data,
                                    int depth,
                                    bool add_bonus,
                                    bool random_ties) const {
        float log_ns = logf(data.counts_[0]);
        std::vector<Problem::action_t> best_actions;
        int nactions = problem_.number_actions(state);
        float best_value = std::numeric_limits<float>::max();

        best_actions.reserve(random_ties ? nactions : 1);
        for( Problem::action_t a = 0; a < nactions; ++a ) {
            if( problem_.applicable(state, a) ) {
                // if this action has never been taken in this node, select it
                if( data.counts_[1+a] == 0 ) {
                    return a;
                }

                // compute score of action adding bonus (if applicable)
                assert(data.counts_[0] > 0);
                float par = parameter_ == 0 ? -data.values_[1+a] : parameter_;
                float bonus = add_bonus ? par * sqrtf(2 * log_ns / data.counts_[1+a]) : 0;
                float value = data.values_[1+a] + bonus;

                // update best action so far
                if( value <= best_value ) {
                    if( value < best_value ) {
                        best_value = value;
                        best_actions.clear();
                    }
                    if( random_ties || best_actions.empty() )
                        best_actions.push_back(a);
                }
            }
        }
        assert(!best_actions.empty());
        return best_actions[Random::random(best_actions.size())];
    }

    float evaluate(const T &s, unsigned depth) const {
        assert(base_policy_ != 0);
        float start_time = Utils::read_time_in_seconds();
        float value = Evaluation::evaluation(*base_policy_, s, 1, horizon_ - depth);
        policy_t<T>::base_policy_time_ += Utils::read_time_in_seconds() - start_time;
        return value;
    }
};

}; // namespace UCT

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

