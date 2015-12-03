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

#ifndef VALUE_ITERATION_H
#define VALUE_ITERATION_H

#include "algorithm.h"

#include <list>
#include <string>
#include <vector>

//#define DEBUG

namespace Algorithm {

template<typename T> class value_iteration_t : public algorithm_t<T> {
  using algorithm_t<T>::problem_;
  using algorithm_t<T>::heuristic_;
  using algorithm_t<T>::seed_;
  protected:
    typedef typename Problem::hash_t<T>::iterator hash_iterator;

    float epsilon_;
    unsigned max_number_iterations_;

    value_iteration_t(const Problem::problem_t<T> &problem,
                      float epsilon,
                      unsigned max_number_iterations,
                      const Heuristic::heuristic_t<T> *heuristic)
      : algorithm_t<T>(problem),
        epsilon_(epsilon),
        max_number_iterations_(max_number_iterations) {
        heuristic_ = heuristic;
    }

  public:
    value_iteration_t(const Problem::problem_t<T> &problem)
      : algorithm_t<T>(problem),
        epsilon_(0),
        max_number_iterations_(std::numeric_limits<unsigned>::max()) {
    }
    virtual ~value_iteration_t() { }
    virtual algorithm_t<T>* clone() const {
        return new value_iteration_t(problem_, epsilon_, max_number_iterations_, heuristic_);
    }
    virtual std::string name() const {
        return std::string("value-iteration(heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
          std::string(",epsilon=") + std::to_string(epsilon_) +
          std::string(",max-number-iterations=") + std::to_string(max_number_iterations_) +
          std::string(",seed=") + std::to_string(seed_) + ")";
    }

    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("epsilon");
        if( it != parameters.end() ) epsilon_ = strtof(it->second.c_str(), 0);
        it = parameters.find("max-number-iterations");
        if( it != parameters.end() ) max_number_iterations_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("heuristic");
        if( it != parameters.end() ) {
            delete heuristic_;
            dispatcher.create_request(problem_, it->first, it->second);
            heuristic_ = dispatcher.fetch_heuristic(it->second);
        }
        it = parameters.find("seed");
        if( it != parameters.end() ) seed_ = strtol(it->second.c_str(), 0, 0);
        std::cout << "VI: params: epsilon=" << epsilon_ << ", max=" << max_number_iterations_ << ", heuristic=" << (heuristic_ == 0 ? std::string("null") : heuristic_->name()) << ", seed=" << seed_ << std::endl;
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        Heuristic::wrapper_t<T> eval_function(heuristic_);
        if( heuristic_ != 0 ) heuristic_->reset_stats();
        hash.set_eval_function(&eval_function);
        hash.clear();

        generate_space(s, hash);

#ifdef DEBUG
        std::cout << "value_iteration: state-space-size = " << hash.size() << std::endl;
#endif

        size_t iters = 0;
        float residual = 1 + epsilon_;
        while( residual > epsilon_ ) {
            if( iters > max_number_iterations_ ) break;
            residual = 0;
            for( hash_iterator hi = hash.begin(); hi != hash.end(); ++hi ) {
                float hv = hi->second->value();
                std::pair<Problem::action_t, float> p = hash.bestQValue(hi->first);
                float res = (float)fabs(p.second - hv);
                residual = Utils::max(residual, res);
                hi->second->update(p.second);
                hash.inc_updates();

#ifdef DEBUG
                if( res > epsilon_ ) {
                    std::cout << "value_iteration: value for " << hi->first
                              << " changed from " << hv << " to "
                              << p.second << std::endl;
                }
#endif
            }
            ++iters;

#ifdef DEBUG
            std::cout << "value_iteration: residual=" << residual << std::endl;
#endif
        }
        hash.set_eval_function(0);
    }

    void generate_space(const T &s, Problem::hash_t<T> &hash) const {
        std::list<std::pair<T, Hash::data_t*> > open;

        std::vector<std::pair<T, float> > outcomes;
        Hash::data_t *dptr = hash.data_ptr(s);
        open.push_back(std::make_pair(s, dptr));
        dptr->mark();

#ifdef DEBUG
        std::cout << "generate_space: marking " << s << std::endl;
#endif

        while( !open.empty() ) {
            std::pair<T, Hash::data_t*> n = open.front();
            open.pop_front();
            if( problem_.terminal(n.first) ) continue;

            for( Problem::action_t a = 0; a < problem_.number_actions(n.first); ++a ) {
                if( problem_.applicable(n.first, a) ) {
                    problem_.next(n.first, a, outcomes);
                    unsigned osize = outcomes.size();
                    for( unsigned i = 0; i < osize; ++i ) {
                        Hash::data_t *ptr = hash.data_ptr(outcomes[i].first);
                        if( !ptr->marked() ) {
                            open.push_back(std::make_pair(outcomes[i].first, ptr));
                            ptr->mark();
#ifdef DEBUG
                            std::cout << "generate_space: marking " << outcomes[i].first << std::endl;
#endif
                        }
                    }
                }
            }
        }
        hash.unmark_all();
    }
};

}; // namespace Algorithm

#undef DEBUG

#endif

