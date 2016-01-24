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

#ifndef HDP_H
#define HDP_H

#include "algorithm.h"

#include <cassert>
#include <list>
#include <string>
#include <vector>

//#define DEBUG

namespace Algorithm {

template<typename T> class hdp_t : public algorithm_t<T> {
  using algorithm_t<T>::problem_;
  using algorithm_t<T>::heuristic_;
  using algorithm_t<T>::seed_;
  protected:
    float epsilon_;

    hdp_t(const Problem::problem_t<T> &problem,
          float epsilon,
          const Heuristic::heuristic_t<T> *heuristic)
      : algorithm_t<T>(problem),
        epsilon_(epsilon){
        heuristic_ = heuristic;
    }

  public:
    hdp_t(const Problem::problem_t<T> &problem)
      : algorithm_t<T>(problem), epsilon_(0) {
    }
    virtual ~hdp_t() { }
    virtual algorithm_t<T>* clone() const {
        return new hdp_t(problem_, epsilon_, heuristic_);
    }
    virtual std::string name() const {
        return std::string("hdp(heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
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
#ifdef DEBUG
        std::cout << "debug: hdp(): params:"
                  << " epsilon= " << epsilon_
                  << " heuristic= " << (heuristic_ == 0 ? std::string("null") : heuristic_->name())
                  << " seed= " << seed_
                  << std::endl;
#endif
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        reset_stats(hash);
        Heuristic::wrapper_t<T> eval_function(heuristic_);
        hash.set_eval_function(&eval_function);

        std::list<Hash::data_t*> stack, visited;
        Hash::data_t *dptr = hash.data_ptr(s);
        size_t trials = 0;
        while( !dptr->solved() ) {
            size_t index = 0;
            hdp(s, hash, dptr, index, stack, visited);
            assert(stack.empty());
            while( !visited.empty() ) {
                visited.front()->unmark();
                assert(visited.front()->scc_low() == std::numeric_limits<unsigned>::max());
                assert(visited.front()->scc_idx() == std::numeric_limits<unsigned>::max());
                visited.pop_front();
            }
            ++trials;
        }
        hash.set_eval_function(0);
    }

    virtual void reset_stats(Problem::hash_t<T> &hash) const {
        algorithm_t<T>::problem_.clear_expansions();
        if( heuristic_ != 0 ) heuristic_->reset_stats();
        hash.clear();
    }

    bool hdp(const T &s,
             Problem::hash_t<T> &hash,
             Hash::data_t* dptr,
             size_t &index,
             std::list<Hash::data_t*> &stack,
             std::list<Hash::data_t*> &visited) const {
        std::vector<std::pair<T, float> > outcomes;

        // base cases
        if( dptr->solved() || problem_.terminal(s) ) {
            dptr->solve();
            return true;
        } else if( dptr->marked() ) {
            return false;
        }

        // if residual > epsilon, update and return
        std::pair<Problem::action_t, float> p = hash.bestQValue(s);
        if( fabs(p.second - dptr->value()) > epsilon_ ) {
            dptr->update(p.second);
            hash.inc_updates();
            return false;
        }

        // Tarjan's
        visited.push_front(dptr);
        stack.push_front(dptr);
        size_t idx = index++;
        dptr->set_scc_low(idx);
        dptr->set_scc_idx(idx);
        dptr->mark();

        // expansion
        problem_.next(s, p.first, outcomes);
        unsigned osize = outcomes.size();

        bool flag = true;
        for( unsigned i = 0; i < osize; ++i ) {
            Hash::data_t *ptr = hash.data_ptr(outcomes[i].first);
            if( ptr->scc_idx() == std::numeric_limits<unsigned>::max() ) {
                bool rv = hdp(outcomes[i].first, hash, ptr, index, stack, visited);
                flag = flag && rv;
                dptr->set_scc_low(Utils::min(dptr->scc_low(), hash.scc_low(outcomes[i].first)));
            } else if( ptr->marked() ) {
                dptr->set_scc_low(Utils::min(dptr->scc_low(), ptr->scc_idx()));
            }
        }

        // update
        if( !flag ) {
            std::pair<Problem::action_t, float> p = hash.bestQValue(s);
            dptr->update(p.second);
            hash.inc_updates();
            while( !stack.empty() && (stack.front()->scc_idx() >= idx) ) {
                stack.front()->set_scc_low(std::numeric_limits<unsigned>::max());
                stack.front()->set_scc_idx(std::numeric_limits<unsigned>::max());
                stack.pop_front();
            }
        } else if( dptr->scc_low() == dptr->scc_idx() ) {
            while( !stack.empty() && (stack.front()->scc_idx() >= idx) ) {
                stack.front()->solve();
                stack.front()->set_scc_low(std::numeric_limits<unsigned>::max());
                stack.front()->set_scc_idx(std::numeric_limits<unsigned>::max());
                stack.pop_front();
            }
        }
        return flag;
    }
};

}; // namespace Algorithm

#undef DEBUG

#endif

