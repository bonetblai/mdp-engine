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

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "dispatcher.h"
#include "heuristic.h"
#include "problem.h"

#include <map>
#include <string>

//#define DEBUG

namespace Algorithm {

extern unsigned g_seed;

template<typename T> class algorithm_t {
  protected:
    const Problem::problem_t<T> &problem_;
    const Heuristic::heuristic_t<T> *heuristic_;
    unsigned seed_;

  public:
    algorithm_t(const Problem::problem_t<T> &problem)
      : problem_(problem), heuristic_(0), seed_(g_seed) {
    }
    virtual ~algorithm_t() { }
    virtual algorithm_t<T>* clone() const = 0;
    virtual std::string name() const = 0;
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) = 0;
    virtual void solve(const T &s, Problem::hash_t<T> &hash) const = 0;
    unsigned seed() const { return seed_; }
    const Problem::problem_t<T>& problem() const { return problem_; }
    const Heuristic::heuristic_t<T>* heuristic() const { return heuristic_; }
};

}; // namespace Algorithm

#undef DEBUG

#endif

