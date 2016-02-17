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

#ifndef LRTDP_H
#define LRTDP_H

#include "algorithm.h"
#include "plain_check.h"

#include <list>
#include <string>

//#define DEBUG

namespace Algorithm {

template<typename T> class lrtdp_base_t : public algorithm_t<T> {
  protected:
  using algorithm_t<T>::problem_;
  using algorithm_t<T>::heuristic_;
  using algorithm_t<T>::seed_;

  protected:
    int type_;
    float epsilon_;
    unsigned bound_;
    float epsilon_greedy_;

    lrtdp_base_t(const Problem::problem_t<T> &problem,
                 int type,
                 float epsilon,
                 unsigned bound,
                 float epsilon_greedy,
                 const Heuristic::heuristic_t<T> *heuristic)
      : algorithm_t<T>(problem),
        type_(type),
        epsilon_(epsilon),
        bound_(bound),
        epsilon_greedy_(epsilon_greedy) {
        heuristic_ = heuristic;
    }

  public:
    lrtdp_base_t(const Problem::problem_t<T> &problem, int type)
      : algorithm_t<T>(problem),
        type_(type), epsilon_(0),
        bound_(std::numeric_limits<unsigned>::max()), epsilon_greedy_(0) {
    }
    virtual ~lrtdp_base_t() { }
    virtual std::string name() const {
        return std::string("lrtdp(heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
          std::string(",epsilon=") + std::to_string(epsilon_) +
          std::string(",type=") + (type_ == 0 ? "standard" : (type_ == 1 ? "uniform" : "bounded")) +
          std::string(",bound=") + std::to_string(bound_) +
          std::string(",epsilon-greedy=") + std::to_string(epsilon_greedy_) +
          std::string(",seed=") + std::to_string(seed_) + ")";
    }

    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("epsilon");
        if( it != parameters.end() ) epsilon_ = strtof(it->second.c_str(), 0);
        it = parameters.find("bound");
        if( it != parameters.end() ) bound_ = strtoul(it->second.c_str(), 0, 0);
        it = parameters.find("epsilon-greedy");
        if( it != parameters.end() ) epsilon_greedy_ = strtof(it->second.c_str(), 0);
        it = parameters.find("heuristic");
        if( it != parameters.end() ) {
            delete heuristic_;
            dispatcher.create_request(problem_, it->first, it->second);
            heuristic_ = dispatcher.fetch_heuristic(it->second);
        }
        it = parameters.find("seed");
        if( it != parameters.end() ) seed_ = strtol(it->second.c_str(), 0, 0);
#ifdef DEBUG
        std::cout << "debug: lrtdp-base(): params:"
                  << " epsilon=" << epsilon_
                  << " bound=" << bound_
                  << " epsilon-greedy=" << epsilon_greedy_
                  << " heuristic=" << (heuristic_ == 0 ? std::string("null") : heuristic_->name())
                  << " seed=" << seed_
                  << std::endl;
#endif
    }

    size_t lrtdp_trial(const T &s, Problem::hash_t<T> &hash) const {
        std::list<T> states;
        std::pair<T, bool> n;

        Hash::data_t *dptr = hash.data_ptr(s);
        states.push_back(s);
        dptr->inc_count();
        T t = s;

#ifdef DEBUG
        std::cout << "debug: lrtdp-base(): trial: begin" << std::endl;
#endif

        size_t steps = 1;
        while( !problem_.terminal(t) && !dptr->solved() && (dptr->count() <= bound_) ) {

#ifdef DEBUG
            std::cout << "debug: lrtdp-base():   " << t << " = " << dptr->value() << std::endl;
#endif

            std::pair<Problem::action_t, float> p = hash.bestQValue(t);
            dptr->update(p.second);
            hash.inc_updates();

            if( Random::real() < epsilon_greedy_ ) {
                n = problem_.usample(t, p.first);
            } else {
                if( type_ == 0 ) {
                    n = problem_.sample(t, p.first);
                } else if( type_ == 1 ) {
                    n = problem_.usample(t, p.first);
                } else if( type_ == 2 ) {
                    n = problem_.nsample(t, p.first, hash);
                }
            }
            if( !n.second ) break;

            t = n.first;
            dptr = hash.data_ptr(t);
            states.push_back(t);
            dptr->inc_count();
            ++steps;
        }

#ifdef DEBUG
        std::cout << "debug: lrtdp-base(): trial: end" << std::endl;
#endif

        while( !states.empty() ) {
            hash.clear_count(states.back());
            bool solved = check_solved(problem_, states.back(), hash, epsilon_);
            states.pop_back();
            if( !solved ) break;
        }

        while( !states.empty() ) {
            hash.clear_count(states.back());
            states.pop_back();
        }
        return steps;
    }

    size_t lrtdp(const T &s, Problem::hash_t<T> &hash) const {
        Heuristic::wrapper_t<T> eval_function(heuristic_);
        hash.set_eval_function(&eval_function);

        size_t trials = 0, max_steps = 0;
        while( !hash.solved(s) ) {
            size_t steps = lrtdp_trial(s, hash);
            max_steps = Utils::max(max_steps, steps);
            ++trials;
        }

        hash.set_eval_function(0);
        return trials;
    }
};

template<typename T> class standard_lrtdp_t : public lrtdp_base_t<T> {
  protected:
  using lrtdp_base_t<T>::heuristic_;

  protected:
    standard_lrtdp_t(const Problem::problem_t<T> &problem,
                     float epsilon,
                     unsigned bound,
                     float epsilon_greedy,
                     const Heuristic::heuristic_t<T> *heuristic)
      : lrtdp_base_t<T>(problem, 0, epsilon, bound, epsilon_greedy, heuristic) {
    }

  public:
    standard_lrtdp_t(const Problem::problem_t<T> &problem) : lrtdp_base_t<T>(problem, 0) { }
    virtual ~standard_lrtdp_t() { }
    virtual algorithm_t<T>* clone() const {
        return new standard_lrtdp_t(algorithm_t<T>::problem_, lrtdp_base_t<T>::epsilon_, lrtdp_base_t<T>::bound_, lrtdp_base_t<T>::epsilon_greedy_, algorithm_t<T>::heuristic_);
    }

    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        lrtdp_base_t<T>::set_parameters(parameters, dispatcher);
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        reset_stats(hash);
        lrtdp_base_t<T>::lrtdp(s, hash);
    }

    virtual void reset_stats(Problem::hash_t<T> &hash) const {
        algorithm_t<T>::problem_.clear_expansions();
        if( heuristic_ != 0 ) heuristic_->reset_stats();
        hash.clear();
    }
};

template<typename T> class uniform_lrtdp_t : public lrtdp_base_t<T> {
  protected:
  using lrtdp_base_t<T>::heuristic_;

  protected:
    uniform_lrtdp_t(const Problem::problem_t<T> &problem,
                    float epsilon,
                    unsigned bound,
                    float epsilon_greedy,
                    const Heuristic::heuristic_t<T> *heuristic)
      : lrtdp_base_t<T>(problem, 1, epsilon, bound, epsilon_greedy, heuristic) {
    }

  public:
    uniform_lrtdp_t(const Problem::problem_t<T> &problem) : lrtdp_base_t<T>(problem, 1) { }
    virtual ~uniform_lrtdp_t() { }
    virtual algorithm_t<T>* clone() const {
        return new uniform_lrtdp_t(algorithm_t<T>::problem_, lrtdp_base_t<T>::epsilon_, lrtdp_base_t<T>::bound_, lrtdp_base_t<T>::epsilon_greedy_, algorithm_t<T>::heuristic_);
    }

    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        lrtdp_base_t<T>::set_parameters(parameters, dispatcher);
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        reset_stats(hash);
        lrtdp_base_t<T>::lrtdp(s, hash);
    }

    virtual void reset_stats(Problem::hash_t<T> &hash) const {
        algorithm_t<T>::problem_.clear_expansions();
        if( heuristic_ != 0 ) heuristic_->reset_stats();
        hash.clear();
    }
};

template<typename T> class bounded_lrtdp_t : public lrtdp_base_t<T> {
  protected:
  using lrtdp_base_t<T>::heuristic_;

  protected:
    bounded_lrtdp_t(const Problem::problem_t<T> &problem,
                    float epsilon,
                    unsigned bound,
                    float epsilon_greedy,
                    const Heuristic::heuristic_t<T> *heuristic)
      : lrtdp_base_t<T>(problem, 2, epsilon, bound, epsilon_greedy, heuristic) {
    }

  public:
    bounded_lrtdp_t(const Problem::problem_t<T> &problem) : lrtdp_base_t<T>(problem, 2) { }
    virtual ~bounded_lrtdp_t() { }
    virtual algorithm_t<T>* clone() const {
        return new bounded_lrtdp_t(algorithm_t<T>::problem_, lrtdp_base_t<T>::epsilon_, lrtdp_base_t<T>::bound_, lrtdp_base_t<T>::epsilon_greedy_, algorithm_t<T>::heuristic_);
    }

    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        lrtdp_base_t<T>::set_parameters(parameters, dispatcher);
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        reset_stats(hash);
        lrtdp_base_t<T>::lrtdp(s, hash);
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

