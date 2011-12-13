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

#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "hash.h"
#include "problem.h"
#include "parameters.h"
#include "utils.h"

#include <iostream>
#include <cassert>
#include <limits>
#include <limits.h>

//#define DEBUG

// forward reference
namespace Algorithm {
  template<typename T> size_t value_iteration(const Problem::problem_t<T>&, const T &s, Problem::hash_t<T>&, const parameters_t&);
};

namespace Heuristic {

template<typename T> class heuristic_t {
  public:
    heuristic_t() { }
    virtual ~heuristic_t() { }
    virtual float value(const T &s) const = 0;
    virtual void reset_stats() const = 0;
    virtual float setup_time() const = 0;
    virtual float eval_time() const = 0;
    virtual size_t size() const = 0;
    virtual void dump(std::ostream &os) const = 0;
    float total_time() const { return setup_time() + eval_time(); }
};

template<typename T> class min_min_heuristic_t : public heuristic_t<T> {
  protected:
    const Problem::problem_t<T> &problem_;
    mutable Problem::min_hash_t<T> hash_;
    mutable float time_;

  public:
    min_min_heuristic_t(const Problem::problem_t<T> &problem)
      : problem_(problem), hash_(problem), time_(0) {

      Algorithm::parameters_t parameters;
      parameters.vi.max_number_iterations_ = std::numeric_limits<unsigned>::max();

      float start_time = Utils::read_time_in_seconds();
      Algorithm::value_iteration<T>(problem_, problem.init(), hash_, parameters);
      float end_time = Utils::read_time_in_seconds();
      time_ = end_time - start_time;
    } 
    virtual ~min_min_heuristic_t() { }

    virtual float value(const T &s) const { return hash_.value(s); }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return time_; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return hash_.size(); }
    virtual void dump(std::ostream &os) const { hash_.dump(os); }
};

#if 0
template<typename T> class hdp_heuristic_t : public heuristic_t<T> {
  protected:
    const Problem::problem_t<T> &problem_;
    mutable Problem::hash_t<T> hash_;
    float epsilon_;
    size_t kappa_;
    mutable float time_;

  public:
    hdp_heuristic_t(const Problem::problem_t<T> &problem, float epsilon = 0, size_t kappa = 0)
      : problem_(problem), epsilon_(epsilon), kappa_(kappa), time_(0) {
    }
    virtual ~hdp_heuristic_t() { }

    virtual float value(const T &s) const {
        std::list<Hash::data_t*> visited;
        std::list<Hash::data_t*> stack;
        float start_time = Utils::read_time_in_seconds();
        Hash::data_t *dptr = hash_.data_ptr(s);
        while( !dptr->solved() ) {
            size_t index = 0;
            Algorithm::lenforce(problem_, hash_, s, dptr, epsilon_, index, kappa_, stack, visited);
            assert(stack.empty());
            while( !visited.empty() ) {
                visited.front()->unmark();
                assert(visited.front()->scc_low() == std::numeric_limits<unsigned>::max());
                assert(visited.front()->scc_idx() == std::numeric_limits<unsigned>::max());
                visited.pop_front();
            }
        }
        float end_time = Utils::read_time_in_seconds();
        time_ += end_time - start_time;
        return dptr->value();
    }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return time_; }
    virtual size_t size() const { return hash_.size(); }
    virtual void dump(std::ostream &os) const { hash_.dump(os); }
};
#endif

template<typename T> class hash_heuristic_t : public heuristic_t<T> {
  protected:
    const Problem::hash_t<T> &hash_;

  public:
    hash_heuristic_t(const Problem::hash_t<T> &hash) : hash_(hash) { }
    virtual ~hash_heuristic_t() { }
    virtual float value(const T &s) const { return hash_.value(s); }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return hash_.size(); }
    virtual void dump(std::ostream &os) const { hash_.dump(os); }
};

template<typename T> struct wrapper_t : public Hash::hash_map_t<T>::eval_function_t {
    const heuristic_t<T> *heuristic_;
    wrapper_t(const heuristic_t<T> *heuristic = 0) : heuristic_(heuristic) { }
    virtual ~wrapper_t() { }
    float operator()(const T &s) const {
        return heuristic_ == 0 ? 0 : heuristic_->value(s);
    }
};

}; // namespace Heuristic

#undef DEBUG

#endif

