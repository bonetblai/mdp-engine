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

#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "hash.h"
#include "problem.h"
#include "utils.h"

#include <cassert>
#include <iostream>
#include <limits>
#include <limits.h>
#include <map>
#include <string>

//#define DEBUG

namespace Heuristic {

template<typename T> class heuristic_t {
  protected:
    const Problem::problem_t<T> &problem_;
    mutable float eval_time_;
    mutable float setup_time_;
    mutable unsigned evaluations_;

  public:
    heuristic_t(const Problem::problem_t<T> &problem)
      : problem_(problem),
        eval_time_(0),
        setup_time_(0),
        evaluations_(0) {
    }
    virtual ~heuristic_t() { }
    virtual heuristic_t<T>* clone() const = 0;
    virtual std::string name() const = 0;
    virtual float value(const T &s) const = 0;
    virtual size_t size() const = 0;
    virtual void dump(std::ostream &os) const = 0;
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) = 0;

    void reset_stats() const {
        eval_time_ = 0;
        setup_time_ = 0;
        evaluations_ = 0;
    }
    float setup_time() const { return setup_time_; }
    float eval_time() const { return eval_time_; }
    float total_time() const {
        return setup_time() + eval_time();
    }
    unsigned evaluations() const { return evaluations_; }
};

template<typename T> class zero_heuristic_t : public heuristic_t<T> {
  using heuristic_t<T>::evaluations_;
  public:
    zero_heuristic_t(const Problem::problem_t<T> &problem) : heuristic_t<T>(problem) { }
    virtual ~zero_heuristic_t() { }
    virtual heuristic_t<T>* clone() const { return new zero_heuristic_t<T>(heuristic_t<T>::problem_); }
    virtual std::string name() const { return std::string("zero()"); }
    virtual float value(const T &s) const {
        ++evaluations_;
        return 0;
    }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) { }
};

template<typename T> class min_min_heuristic_t : public heuristic_t<T> {
  using heuristic_t<T>::problem_;
  using heuristic_t<T>::eval_time_;
  using heuristic_t<T>::setup_time_;
  using heuristic_t<T>::evaluations_;
  protected:
    Problem::min_hash_t<T> *hash_;
    const Algorithm::algorithm_t<T> *algorithm_;

    min_min_heuristic_t(const Problem::problem_t<T> &problem,
                        const Problem::min_hash_t<T> *hash,
                        const Algorithm::algorithm_t<T> *algorithm,
                        float setup_time)
      : heuristic_t<T>(problem), hash_(0), algorithm_(0) {
        if( hash != 0 ) hash_ = new Problem::min_hash_t<T>(*hash);
        if( algorithm != 0 ) algorithm_ = algorithm->clone();
    }

  public:
    min_min_heuristic_t(const Problem::problem_t<T> &problem)
      : heuristic_t<T>(problem), hash_(0), algorithm_(0) {
        hash_ = new Problem::min_hash_t<T>(problem);
    } 
    virtual ~min_min_heuristic_t() { delete hash_; }
    virtual heuristic_t<T>* clone() const {
        return new min_min_heuristic_t<T>(problem_, hash_, algorithm_, setup_time_);
    }
    virtual std::string name() const {
        return std::string("min-min(algorithm=") + (algorithm_ == 0 ? std::string("null") : algorithm_->name()) + ")";
    }
    virtual float value(const T &s) const {
        float start_time = Utils::read_time_in_seconds();
        ++evaluations_;
        float hvalue = hash_ == 0 ? 0 : hash_->value(s);
        eval_time_ += Utils::read_time_in_seconds() - start_time;
        return hvalue;
    }
    virtual size_t size() const { return hash_->size(); }
    virtual void dump(std::ostream &os) const { hash_->dump(os); }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("algorithm");
        if( it != parameters.end() ) {
            delete algorithm_;
            dispatcher.create_request(problem_, it->first, it->second);
            algorithm_ = dispatcher.fetch_algorithm(it->second);
        }
#ifdef DEBUG
        std::cout << "debug: min-min(): params:"
                  << " algorithm=" << (algorithm_ == 0 ? std::string("null") : algorithm_->name())
                  << std::endl;
#endif
        solve_problem();
    }

    void solve_problem() {
        if( algorithm_ == 0 ) {
            std::cout << Utils::error() << "algorithm must be specified for min-min() heuristic!" << std::endl;
            exit(1);
        }
#ifdef DEBUG
        std::cout << "debug: min-min(): solving problem with algorithm=" << algorithm_->name() << std::endl;
#endif
        float start_time = Utils::read_time_in_seconds();
        algorithm_->solve(problem_.init(), *hash_);
        setup_time_ = Utils::read_time_in_seconds() - start_time;
    }
};

template<typename T> class hash_heuristic_t : public heuristic_t<T> {
  using heuristic_t<T>::problem_;
  using heuristic_t<T>::eval_time_;
  using heuristic_t<T>::evaluations_;
  protected:
    Problem::hash_t<T> *hash_;

    hash_heuristic_t(const Problem::problem_t<T> &problem, const Problem::hash_t<T> *hash)
      : heuristic_t<T>(problem), hash_(0) {
        if( hash != 0 ) hash_ = new Problem::hash_t<T>(*hash);
    }

  public:
    hash_heuristic_t(const Problem::problem_t<T> &problem)
      : heuristic_t<T>(problem), hash_(0) { }
    virtual ~hash_heuristic_t() { }
    virtual heuristic_t<T>* clone() const { return new hash_heuristic_t<T>(problem_, hash_); }
    virtual std::string name() const {
        return std::string("hash(hash-ptr=") + std::to_string(size_t(hash_)) + ")";
    }
    virtual float value(const T &s) const {
        float start_time = Utils::read_time_in_seconds();
        ++evaluations_;
        float hvalue = hash_ == 0 ? 0 : hash_->value(s);
        eval_time_ += Utils::read_time_in_seconds() - start_time;
        return hvalue;
    }
    virtual size_t size() const { return hash_->size(); }
    virtual void dump(std::ostream &os) const { hash_->dump(os); }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::cout << Utils::error() << "hash_heuristic_t: set_parameters not supported" << std::endl;
    }
};

template<typename T> class optimal_heuristic_t : public hash_heuristic_t<T> {
  using heuristic_t<T>::problem_;
  using hash_heuristic_t<T>::hash_;
  using heuristic_t<T>::setup_time_;
  protected:
    const Algorithm::algorithm_t<T> *algorithm_;

    optimal_heuristic_t(const Problem::problem_t<T> &problem, const Problem::hash_t<T> *hash, const Algorithm::algorithm_t<T> *algorithm)
      : hash_heuristic_t<T>(problem, 0), algorithm_(0) {
        if( algorithm != 0 ) algorithm_ = algorithm->clone();
        if( hash != 0 ) hash_ = new Problem::hash_t<T>(*hash);
    }

  public:
    optimal_heuristic_t(const Problem::problem_t<T> &problem)
      : hash_heuristic_t<T>(problem), algorithm_(0) {
      hash_ = new Problem::hash_t<T>(problem_);
    }
    virtual ~optimal_heuristic_t() { delete hash_; }
    virtual heuristic_t<T>* clone() const { return new optimal_heuristic_t(problem_, hash_, algorithm_); }
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
#ifdef DEBUG
        std::cout << "debug: optimal(): params:"
                  << " algorithm=" << (algorithm_ == 0 ? std::string("null") : algorithm_->name())
                  << std::endl;
#endif
        solve_problem();
    }

    void solve_problem() {
        if( algorithm_ == 0 ) {
            std::cout << Utils::error() << "algorithm must be specified for optimal() heuristic!" << std::endl;
            exit(1);
        }
#ifdef DEBUG
        std::cout << "debug: optimal(): solving problem with algorithm=" << algorithm_->name() << std::endl;
#endif
        float start_time = Utils::read_time_in_seconds();
        assert(algorithm_ != 0);
        algorithm_->solve(problem_.init(), *hash_);
        setup_time_ = Utils::read_time_in_seconds() - start_time;
    }
};

template<typename T> class scaled_heuristic_t : public heuristic_t<T> {
  using heuristic_t<T>::problem_;
  using heuristic_t<T>::eval_time_;
  using heuristic_t<T>::setup_time_;
  using heuristic_t<T>::evaluations_;
  protected:
    const heuristic_t<T> *heuristic_;
    float weight_;

    scaled_heuristic_t(const Problem::problem_t<T> &problem, const heuristic_t<T> *heuristic, float weight)
      : heuristic_t<T>(problem), heuristic_(heuristic), weight_(weight) {
    }

  public:
    scaled_heuristic_t(const Problem::problem_t<T> &problem)
      : heuristic_t<T>(problem), heuristic_(0), weight_(1) {
    }
    virtual ~scaled_heuristic_t() { }
    virtual heuristic_t<T>* clone() const { return new scaled_heuristic_t<T>(problem_, heuristic_, weight_); }
    virtual std::string name() const {
        return std::string("scaled(") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) + ",weight=" + std::to_string(weight_) + ")";
    }
    virtual float value(const T &s) const {
        ++evaluations_;
        float hvalue = heuristic_ == 0 ? 0 : weight_ * heuristic_->value(s);
        eval_time_ = heuristic_ == 0 ? 0 : heuristic_->eval_time();
        return hvalue;
    }
    virtual size_t size() const { return heuristic_ == 0 ? 0 : heuristic_->size(); }
    virtual void dump(std::ostream &os) const { heuristic_->dump(os); }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("heuristic");
        if( it != parameters.end() ) {
            delete heuristic_;
            dispatcher.create_request(problem_, it->first, it->second);
            heuristic_ = dispatcher.fetch_heuristic(it->second);
        }
        it = parameters.find("weight");
        if( it != parameters.end() ) weight_ = strtof(it->second.c_str(), 0);
        it = parameters.find("heuristic");
#ifdef DEBUG
        std::cout << "debug: scaled(): params:"
                  << " heuristic=" << (heuristic_ == 0 ? std::string("null") : heuristic_->name())
                  << " weight=" << weight_
                  << std::endl;
#endif
    }
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

