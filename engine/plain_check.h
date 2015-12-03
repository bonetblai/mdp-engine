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

#ifndef PLAIN_CHECK_H
#define PLAIN_CHECK_H

#include "algorithm.h"

#include <list>
#include <string>
#include <vector>

//#define DEBUG

namespace Algorithm {

template<typename T>
bool check_solved(const Problem::problem_t<T> &problem,
                  const T &s,
                  Problem::hash_t<T> &hash,
                  float epsilon) {
    std::list<std::pair<T, Hash::data_t*> > open, closed;

    std::vector<std::pair<T, float> > outcomes;
    Hash::data_t *dptr = hash.data_ptr(s);
    if( !dptr->solved() ) {
        open.push_back(std::make_pair(s, dptr));
        dptr->mark();
    }

    bool rv = true;
    while( !open.empty() ) {
        std::pair<T, Hash::data_t*> n = open.back();
        closed.push_back(n);
        open.pop_back();
        if( problem.terminal(n.first) ) continue;

        std::pair<Problem::action_t, float> p = hash.bestQValue(n.first);
        if( fabs(p.second - n.second->value()) > epsilon ) {
            rv = false;
            continue;
        }

        problem.next(n.first, p.first, outcomes);
        unsigned osize = outcomes.size();

        for( unsigned i = 0; i < osize; ++ i ) {
            Hash::data_t *dptr = hash.data_ptr(outcomes[i].first);
            if( !dptr->solved() && !dptr->marked() ) {
                open.push_back(std::make_pair(outcomes[i].first, dptr));
                dptr->mark();
            }
        }
    }

    if( rv ) {
        while( !closed.empty() ) {
            closed.back().second->solve();
            closed.pop_back();
        }
    } else {
        while( !closed.empty() ) {
            std::pair<Problem::action_t, float> p = hash.bestQValue(closed.back().first);
            closed.back().second->update(p.second);
            closed.back().second->unmark();
            hash.inc_updates();
            closed.pop_back();
        }
    }
    return rv;
}

template<typename T> class plain_check_t : public algorithm_t<T> {
  using algorithm_t<T>::problem_;
  using algorithm_t<T>::heuristic_;
  using algorithm_t<T>::seed_;
  protected:
    float epsilon_;

    plain_check_t(const Problem::problem_t<T> &problem,
                  float epsilon,
                  const Heuristic::heuristic_t<T> *heuristic)
      : algorithm_t<T>(problem), epsilon_(epsilon) {
      heuristic_ = heuristic;
    }

  public:
    plain_check_t(const Problem::problem_t<T> &problem) : algorithm_t<T>(problem) { }
    virtual ~plain_check_t() { }
    virtual algorithm_t<T>* clone() const {
        return new plain_check_t(problem_, epsilon_, heuristic_);
    }
    virtual std::string name() const {
        return std::string("plain-check(heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
          std::string(",epsilon=") + std::to_string(epsilon_) +
          std::string(",seed=") + std::to_string(seed_) + ")";
    }

    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("epsilon");
        if( it != parameters.end() ) epsilon_ = strtof(it->second.c_str(), 0);
        it = parameters.find("heuristic");
        if( it != parameters.end() ) {
            delete heuristic_;
            dispatcher.create_request(problem_, it->first, it->second);
            heuristic_ = dispatcher.fetch_heuristic(it->second);
        }
        it = parameters.find("seed");
        if( it != parameters.end() ) seed_ = strtol(it->second.c_str(), 0, 0);
        std::cout << "PLAIN-CHECK: params: epsilon=" << epsilon_ << ", heuristic=" << (heuristic_ == 0 ? std::string("null") : heuristic_->name()) << ", seed=" << seed_ << std::endl;
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        Heuristic::wrapper_t<T> eval_function(heuristic_);
        if( heuristic_ != 0 ) heuristic_->reset_stats();
        hash.set_eval_function(&eval_function);
        hash.clear();

        size_t trials = 0;
        while( !hash.solved(s) ) {
            check_solved(problem_, s, hash, epsilon_);
            ++trials;
        }
        hash.set_eval_function(0);
    }
};

}; // namespace Algorithm

#undef DEBUG

#endif

