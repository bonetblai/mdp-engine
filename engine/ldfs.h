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

#ifndef LDFS_H
#define LDFS_H

#include "algorithm.h"

#include <cassert>
#include <list>
#include <string>
#include <vector>

//#define DEBUG

namespace Algorithm {

template<typename T> class ldfs_base_t : public algorithm_t<T> {
  using algorithm_t<T>::problem_;
  using algorithm_t<T>::heuristic_;
  using algorithm_t<T>::seed_;
  protected:
    int type_;
    float epsilon_;

    ldfs_base_t(const Problem::problem_t<T> &problem,
                int type,
                float epsilon,
                const Heuristic::heuristic_t<T> *heuristic)
      : algorithm_t<T>(problem),
        type_(type),
        epsilon_(epsilon) {
        heuristic_ = heuristic;
    }

  public:
    ldfs_base_t(const Problem::problem_t<T> &problem, int type)
      : algorithm_t<T>(problem),
        type_(type), epsilon_(0) {
    }
    virtual ~ldfs_base_t() { }
    virtual std::string name() const {
        return std::string("ldfs(heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
          std::string(",epsilon=") + std::to_string(epsilon_) +
          std::string(",type=") + (type_ == 0 ? "regular" : "plus") +
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
        std::cout << "debug: ldfs-base(): params:"
                  << " epsilon= " << epsilon_
                  << " heuristic= " << (heuristic_ == 0 ? std::string("null") : heuristic_->name())
                  << " seed= " << seed_
                  << std::endl;
#endif
    }

    bool ldfs(const T &s,
              Problem::hash_t<T> &hash,
              Hash::data_t *dptr,
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

        // Tarjan's
        visited.push_front(dptr);
        stack.push_front(dptr);
        size_t idx = index++;
        dptr->set_scc_low(idx);
        dptr->set_scc_idx(idx);
        //dptr->mark();

        if( type_ == 1 ) {
            std::pair<Problem::action_t, float> p = hash.bestQValue(s);
            dptr->update(p.second);
            hash.inc_updates();
        }

        // expansion
        bool flag = false;
        float bqv = std::numeric_limits<float>::max();
        for( Problem::action_t a = 0; a < problem_.number_actions(s); ++a ) {
            if( problem_.applicable(s, a) ) {
                problem_.next(s, a, outcomes);
                unsigned osize = outcomes.size();

                float qv = 0.0;
                for( unsigned i = 0; i < osize; ++i )
                    qv += outcomes[i].second * hash.value(outcomes[i].first);
                qv = problem_.cost(s, a) + problem_.discount() * qv;
                bqv = Utils::min(bqv, qv);
                if( fabs(qv - dptr->value()) > epsilon_ ) continue;
                dptr->mark();
                flag = true;
                for( unsigned i = 0; i < osize; ++i ) {
                    Hash::data_t *ptr = hash.data_ptr(outcomes[i].first);
                    if( ptr->scc_idx() == std::numeric_limits<unsigned>::max() ) {
                        bool rv = ldfs(outcomes[i].first, hash, ptr, index, stack, visited);
                        flag = flag && rv;
                        dptr->set_scc_low(Utils::min(dptr->scc_low(), ptr->scc_low()));
                    } else if( ptr->marked() ) {
                        dptr->set_scc_low(Utils::min(dptr->scc_low(), ptr->scc_idx()));
                    }
                }
                if( (type_ == 1) && flag && (hash.QValue(s, a) - dptr->value() > epsilon_) ) flag = false;
                if( flag ) break;
                while( stack.front()->scc_idx() > idx ) {
                    stack.front()->set_scc_low(std::numeric_limits<unsigned>::max());
                    stack.front()->set_scc_idx(std::numeric_limits<unsigned>::max());
                    stack.pop_front();
                }
            }
        }

        // update
        if( !flag ) {
            if( !dptr->marked() ) {
                dptr->update(bqv);
            } else {
                std::pair<Problem::action_t, float> p = hash.bestQValue(s);
                dptr->update(p.second);
            }
            hash.inc_updates();
            dptr->set_scc_low(std::numeric_limits<unsigned>::max());
            dptr->set_scc_idx(std::numeric_limits<unsigned>::max());
            stack.pop_front();
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

template<typename T> class ldfs_t : public ldfs_base_t<T> {
  using algorithm_t<T>::heuristic_;
  protected:
    ldfs_t(const Problem::problem_t<T> &problem,
           float epsilon,
           const Heuristic::heuristic_t<T> *heuristic)
      : ldfs_base_t<T>(problem, 0, epsilon, heuristic) {
    }

  public:
    ldfs_t(const Problem::problem_t<T> &problem)
      : ldfs_base_t<T>(problem, 0) {
    }
    virtual ~ldfs_t() { }
    virtual algorithm_t<T>* clone() const {
        return new ldfs_t(algorithm_t<T>::problem_, ldfs_base_t<T>::epsilon_, algorithm_t<T>::heuristic_);
    }

    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        ldfs_base_t<T>::set_parameters(parameters, dispatcher);
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        reset_stats(hash);
        Heuristic::wrapper_t<T> eval_function(heuristic_);
        hash.set_eval_function(&eval_function);

        std::list<Hash::data_t*> stack, visited;
        size_t trials = 0;
        Hash::data_t *dptr = hash.data_ptr(s);
        while( !dptr->solved() ) {
            size_t index = 0;
            ldfs_base_t<T>::ldfs(s, hash, dptr, index, stack, visited);
            assert(stack.empty());
            while( !visited.empty() ) {
                visited.front()->unmark();
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
};

template<typename T> class ldfs_plus_t : public ldfs_base_t<T> {
  using algorithm_t<T>::heuristic_;
  protected:
    ldfs_plus_t(const Problem::problem_t<T> &problem,
                float epsilon,
                const Heuristic::heuristic_t<T> *heuristic)
      : ldfs_base_t<T>(problem, 1, epsilon, heuristic) {
    }

  public:
    ldfs_plus_t(const Problem::problem_t<T> &problem)
      : ldfs_base_t<T>(problem, 1) {
    }
    virtual ~ldfs_plus_t() { }
    virtual algorithm_t<T>* clone() const {
        return new ldfs_plus_t(algorithm_t<T>::problem_, ldfs_base_t<T>::epsilon_, algorithm_t<T>::heuristic_);
    }

    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        ldfs_base_t<T>::set_parameters(parameters, dispatcher);
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        reset_stats(hash);
        Heuristic::wrapper_t<T> eval_function(heuristic_);
        hash.set_eval_function(&eval_function);

        std::list<Hash::data_t*> stack, visited;
        size_t trials = 0;
        Hash::data_t *dptr = hash.data_ptr(s);
        while( !dptr->solved() ) {
            size_t index = 0;
            ldfs_base_t<T>::ldfs(s, hash, dptr, index, stack, visited);
            assert(stack.empty());
            while( !visited.empty() ) {
                visited.front()->unmark();
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
};

}; // namespace Algorithm

#undef DEBUG

#endif

