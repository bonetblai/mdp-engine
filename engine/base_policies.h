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

#ifndef BASE_POLICIES_H
#define BASE_POLICIES_H

#include "policy.h"

#include <cassert>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>

//#define DEBUG

namespace Online {

namespace Policy {

// Random base policy: select a random applicable action
template<typename T> class random_t : public policy_t<T> {
  using policy_t<T>::problem_;
  public:
    random_t(const Problem::problem_t<T> &problem) : policy_t<T>(problem) { }
    virtual ~random_t() { }
    virtual policy_t<T>* clone() const { return new random_t(problem_); }
    virtual std::string name() const { return std::string("random()"); }

    virtual Problem::action_t operator()(const T &s) const {
        ++policy_t<T>::decisions_;
        if( problem_.dead_end(s) ) return Problem::noop;
        std::vector<Problem::action_t> actions;
        actions.reserve(problem_.number_actions(s));
        for( Problem::action_t a = 0; a < problem_.number_actions(s); ++a ) {
            if( problem_.applicable(s, a) ) {
                actions.push_back(a);
            }
        }
        return actions.empty() ? Problem::noop : actions[Random::uniform(actions.size())];
    }
    virtual void reset_stats() const {
        problem_.clear_expansions();
    }
    virtual void print_other_stats(std::ostream &os, int indent) const {
        os << std::setw(indent) << ""
           << "other-stats: name=" << name()
           << " decisions=" << policy_t<T>::decisions_
           << std::endl;
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
    }
};

// Hash-based based policy: select best action using bestQValue method of hash table
template<typename T> class hash_policy_t : public policy_t<T> {
  using policy_t<T>::problem_;
  protected:
    Problem::hash_t<T> *hash_;

    hash_policy_t(const Problem::problem_t<T> &problem, Problem::hash_t<T> *hash = 0)
      : policy_t<T>(problem), hash_(hash) {
    }

  public:
    virtual ~hash_policy_t() { }
    virtual policy_t<T>* clone() const { return new hash_policy_t<T>(problem_, hash_); }
    virtual std::string name() const { return std::string("hash(hash-ptr=") + std::to_string(size_t(hash_)) + ")"; }

    virtual Problem::action_t operator()(const T &s) const {
        assert(hash_ != 0);
        ++policy_t<T>::decisions_;
        std::pair<Problem::action_t, float> p = hash_->bestQValue(s);
        assert(problem_.applicable(s, p.first));
        return p.first;
    }
    virtual void reset_stats() const {
        problem_.clear_expansions();
    }
    virtual void print_other_stats(std::ostream &os, int indent) const {
        os << std::setw(indent) << ""
           << "other-stats: name=" << name()
           << " decisions=" << policy_t<T>::decisions_
           << std::endl;
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
    }
};

template<typename T> class optimal_policy_t : public hash_policy_t<T> {
  using policy_t<T>::problem_;
  using hash_policy_t<T>::hash_;
  protected:
    const Algorithm::algorithm_t<T> *algorithm_;

    optimal_policy_t(const Problem::problem_t<T> &problem, const Problem::hash_t<T> *hash, const Algorithm::algorithm_t<T> *algorithm)
      : hash_policy_t<T>(problem, 0), algorithm_(0) {
        if( algorithm != 0 ) algorithm_ = algorithm->clone();
        if( hash != 0 ) hash_ = new Problem::hash_t<T>(*hash);
    }

  public:
    optimal_policy_t(const Problem::problem_t<T> &problem)
      : hash_policy_t<T>(problem), algorithm_(0) {
      hash_ = new Problem::hash_t<T>(problem_);
    }
    virtual ~optimal_policy_t() {
        delete hash_; }
    virtual policy_t<T>* clone() const {
        return new optimal_policy_t(problem_, hash_, algorithm_);
    }
    virtual std::string name() const {
        return std::string("optimal(algorithm=") + (algorithm_ == 0 ? std::string("null") : algorithm_->name()) + ")";
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("algorithm");
        if( it != parameters.end() ) {
            delete algorithm_;
            dispatcher.create_request(problem_, it->first, it->second);
            algorithm_ = dispatcher.fetch_algorithm(it->second);
        }
        solve_problem();
#ifdef DEBUG
        std::cout << "debug: optimal(): params:"
                  << " algorithm=" << (algorithm_ == 0 ? std::string("null") : algorithm_->name())
                  << std::endl;
#endif
    }

    void solve_problem() {
        if( algorithm_ == 0 ) {
            std::cout << Utils::error() << "algorithm must be specified for optimal() policy!" << std::endl;
            exit(1);
        }
#ifdef DEBUG
        std::cout << "debug: optimal(): solving problem with algorithm=" << algorithm_->name() << std::endl;
#endif
        algorithm_->solve(problem_.init(), *hash_);
    }
};

// Base class for greedy policies wrt 1-step-lookahead of heuristic
template<typename T> class base_greedy_t : public policy_t<T> {
  using policy_t<T>::problem_;
  protected:
    const Heuristic::heuristic_t<T> *heuristic_;
    bool optimistic_;
    bool random_ties_;
    bool caching_;

    mutable Hash::generic_hash_map_t<T, Problem::action_t> cache_;

    base_greedy_t(const Problem::problem_t<T> &problem,
                  const Heuristic::heuristic_t<T> *heuristic,
                  bool optimistic,
                  bool random_ties,
                  bool caching)
      : policy_t<T>(problem),
        heuristic_(heuristic),
        optimistic_(optimistic),
        random_ties_(random_ties),
        caching_(caching) {
    }

  public:
    base_greedy_t(const Problem::problem_t<T> &problem)
      : policy_t<T>(problem), heuristic_(0), optimistic_(false), random_ties_(false), caching_(false) {
    }
    virtual ~base_greedy_t() { }
    virtual policy_t<T>* clone() const {
        return new base_greedy_t(problem_, heuristic_, optimistic_, random_ties_, caching_);
    }
    virtual std::string name() const {
        return std::string("greedy(") +
          std::string("heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
          std::string(",optimistic=") + (optimistic_ ? "true" : "false") +
          std::string(",random-ties=") + (random_ties_ ? "true" : "false") +
          std::string(",caching=") + (caching_ ? "true" : "false") + ")";
    }

    virtual Problem::action_t operator()(const T &s) const {
        typename Hash::generic_hash_map_t<T, Problem::action_t>::const_iterator it = caching_ ? cache_.find(s) : cache_.end();
        if( it == cache_.end() ) {
            ++policy_t<T>::decisions_;
            std::vector<std::pair<T, float> > outcomes;
            std::vector<Problem::action_t> best_actions;
            int nactions = problem_.number_actions(s);
            float best_value = std::numeric_limits<float>::max();
            best_actions.reserve(random_ties_ ? nactions : 1);
            for( Problem::action_t a = 0; a < nactions; ++a ) {
                if( problem_.applicable(s, a) ) {
                    float value = optimistic_ ? std::numeric_limits<float>::max() : 0;
                    problem_.next(s, a, outcomes);
                    for( size_t i = 0, isz = outcomes.size(); i < isz; ++i ) {
                        float hval = heuristic_ == 0 ? 0 : heuristic_->value(outcomes[i].first);
                        if( optimistic_ )
                            value = hval < value ? hval : value;
                        else
                            value += outcomes[i].second * hval;
                    }
                    value += problem_.cost(s, a);

                    if( value <= best_value ) {
                        if( value < best_value ) {
                            best_value = value;
                            best_actions.clear();
                        }
                        if( random_ties_ || best_actions.empty() )
                            best_actions.push_back(a);
                    }
                }
            }
            Problem::action_t action = best_actions[Random::uniform(best_actions.size())];
            if( caching_ ) {
                //std::cout << "BASE: caching: state=" << s << ", action=" << action << std::endl;
                cache_.insert(std::make_pair(s, action));
            }
            return action;
        } else {
            //std::cout << "BASE: cached result: state=" << s << ", action=" << it->second << std::endl;
            return it->second;
        }
    }
    virtual void reset_stats() const {
        problem_.clear_expansions();
        if( heuristic_ != 0 ) heuristic_->reset_stats();
    }
    virtual void print_other_stats(std::ostream &os, int indent) const {
        os << std::setw(indent) << ""
           << "other-stats: name=" << name()
           << " decisions=" << policy_t<T>::decisions_
           << std::endl;
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("optimistic");
        if( it != parameters.end() ) optimistic_ = it->second == "true";
        it = parameters.find("random-ties");
        if( it != parameters.end() ) random_ties_ = it->second == "true";
        it = parameters.find("caching");
        if( it != parameters.end() ) caching_ = it->second == "true";
        it = parameters.find("heuristic");
        if( it != parameters.end() ) {
            delete heuristic_;
            dispatcher.create_request(problem_, it->first, it->second);
            heuristic_ = dispatcher.fetch_heuristic(it->second);
        }
#ifdef DEBUG
        std::cout << "debug: greedy(): params:"
                  << " optimistic= " << optimistic_
                  << " random-ties= " << random_ties_
                  << " caching= " << caching_
                  << " heuristic= " << (heuristic_ == 0 ? std::string("null") : heuristic_->name())
                  << std::endl;
#endif
    }
};

// Greedy policy with fixed tie breaking
template<typename T> class greedy_t : public base_greedy_t<T> {
  public:
    greedy_t(const Problem::problem_t<T> &problem, const Heuristic::heuristic_t<T> &heuristic, bool caching = false)
      : base_greedy_t<T>(problem, &heuristic, false, false, caching) { }
    virtual ~greedy_t() { }
};

// Greedy policy with random tie breaking
template<typename T> class random_greedy_t : public base_greedy_t<T> {
  public:
    random_greedy_t(const Problem::problem_t<T> &problem, const Heuristic::heuristic_t<T> &heuristic, bool caching = false)
      : base_greedy_t<T>(problem, &heuristic, false, true, caching) { }
    virtual ~random_greedy_t() { }
};

// Optimistic greedy policy with fixed tie breaking
template<typename T> class optimistic_greedy_t : public base_greedy_t<T> {
  public:
    optimistic_greedy_t(const Problem::problem_t<T> &problem, const Heuristic::heuristic_t<T> &heuristic, bool caching = false)
      : base_greedy_t<T>(problem, &heuristic, true, false, caching) { }
    virtual ~optimistic_greedy_t() { }
};

// Optimistic greedy policy with random tie breaking
template<typename T> class random_optimistic_greedy_t : public base_greedy_t<T> {
  public:
    random_optimistic_greedy_t(const Problem::problem_t<T> &problem, const Heuristic::heuristic_t<T> &heuristic, bool caching = false)
      : base_greedy_t<T>(problem, &heuristic, true, true, caching) { }
    virtual ~random_optimistic_greedy_t() { }
};

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

