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

#ifndef DISPATCHER_H
#define DISPATCHER_H

#include "problem.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cassert>
#include <map>
#include <string>
#include <strings.h>

// forward references

namespace Algorithm {
  template<typename T> class algorithm_t;
}; 

namespace Heuristic {
  template<typename T> class heuristic_t;
};

namespace Online {
  namespace Policy {
    template<typename T> class policy_t;
  };
};

namespace Dispatcher {

template<typename T> class dispatcher_t {
    std::map<std::string, Algorithm::algorithm_t<T>*> algorithms_;
    std::map<std::string, Heuristic::heuristic_t<T>*> heuristics_;
    std::map<std::string, Online::Policy::policy_t<T>*> policies_;

  public:
    struct solve_result_t {
        std::string name_;
        const Algorithm::algorithm_t<T> *algorithm_;
        unsigned seed_;

        unsigned problem_expansions_;

        const Problem::hash_t<T> *hash_;
        float hash_value_;
        unsigned hash_updates_;
        unsigned hash_policy_size_;

        const Heuristic::heuristic_t<T> *heuristic_;
        float heuristic_eval_time_;
        float heuristic_setup_time_;
        unsigned heuristic_evaluations_;
        unsigned heuristic_size_;

        float time_raw_;
        float time_heuristic_;
        float time_algorithm_;
    };

    struct evaluate_result_t {
        std::string name_;
        const Online::Policy::policy_t<T> *policy_;
        unsigned seed_;

        unsigned problem_expansions_;

        float eval_value_;
        float eval_stdev_;

        float time_raw_;
        float time_policy_;
        float time_heuristic_;
        float time_algorithm_;
    };

    dispatcher_t() { }
    virtual ~dispatcher_t() {
        for( typename std::map<std::string, Algorithm::algorithm_t<T>*>::const_iterator it = algorithms_.begin(); it != algorithms_.end(); ++it )
            delete it->second;
        for( typename std::map<std::string, Heuristic::heuristic_t<T>*>::const_iterator it = heuristics_.begin(); it != heuristics_.end(); ++it )
            delete it->second;
        for( typename std::map<std::string, Online::Policy::policy_t<T>*>::const_iterator it = policies_.begin(); it != policies_.end(); ++it )
            delete it->second;
    }

    void insert_algorithm(const std::string &request, Algorithm::algorithm_t<T> *algorithm) {
        algorithms_.insert(std::make_pair(request, algorithm));
    }
    Algorithm::algorithm_t<T>* fetch_algorithm(const std::string &request) {
        typename std::map<std::string, Algorithm::algorithm_t<T>*>::const_iterator it = algorithms_.find(request);
        return it == algorithms_.end() ? 0 : it->second;
    }

    void insert_heuristic(const std::string &request, Heuristic::heuristic_t<T> *heuristic) {
        heuristics_.insert(std::make_pair(request, heuristic));
    }
    Heuristic::heuristic_t<T>* fetch_heuristic(const std::string &request) {
        typename std::map<std::string, Heuristic::heuristic_t<T>*>::const_iterator it = heuristics_.find(request);
        return it == heuristics_.end() ? 0 : it->second;
    }

    void insert_policy(const std::string &request, Online::Policy::policy_t<T> *policy) {
        policies_.insert(std::make_pair(request, policy));
    }
    Online::Policy::policy_t<T>* fetch_policy(const std::string &request) {
        typename std::map<std::string, Online::Policy::policy_t<T>*>::const_iterator it = policies_.find(request);
        return it == policies_.end() ? 0 : it->second;
    }

    void create_request(const Problem::problem_t<T> &problem, const std::string &request_str);
    void create_request(const Problem::problem_t<T> &problem, const std::string &type, const std::string &request);
    void solve(const std::string &name, const Algorithm::algorithm_t<T> &algorithm, const T &s, solve_result_t &result) const;
    void print(std::ostream &os, const solve_result_t &result) const;
    void evaluate(const std::string &name, const Online::Policy::policy_t<T> &policy, const T &s, evaluate_result_t &result, unsigned num_trials, unsigned max_evaluation_depth, bool verbose) const;
    void print(std::ostream &os, const evaluate_result_t &result) const;
};

}; // namespace Dispatcher

#include "algorithm.h"
#include "heuristic.h"
#include "base_policies.h"

#include "hdp.h"
#include "improved_lao.h"
#include "ldfs.h"
#include "lrtdp.h"
#include "plain_check.h"
#include "simple_astar.h"
#include "value_iteration.h"

#include "rollout.h"
#include "uct.h"
#include "aot.h"
#include "aot_gh.h"
#include "aot_path.h"
#include "pac.h"
#include "online_rtdp.h"

//#define DEBUG

namespace Dispatcher {

template<typename T> void dispatcher_t<T>::create_request(const Problem::problem_t<T> &problem, const std::string &request_str) {
    std::multimap<std::string, std::string> request;
    Utils::tokenize(request_str, request);
    for( std::multimap<std::string, std::string>::const_iterator it = request.begin(); it != request.end(); ++it )
        create_request(problem, it->first, it->second);
}

template<typename T> void dispatcher_t<T>::create_request(const Problem::problem_t<T> &problem, const std::string &type, const std::string &request) {
    if( (type != "algorithm") && (type != "heuristic") && (type != "policy") ) {
        std::cout << "error: dispatcher: create-request: invalid '" << type << "=" << request << "'" << std::endl;
        return;
    }

    // parse request
    std::string name;
    std::string parameter_str;
    std::multimap<std::string, std::string> parameters;
    Utils::split_request(request, name, parameter_str);
    Utils::tokenize(parameter_str, parameters);

    // algorithms
    if( type == "algorithm" ) {
        if( fetch_algorithm(request) != 0 ) {
#ifdef DEBUG
            std::cout << "dispatcher: create-request: found algorithm '" << request << "'" << std::endl;
#endif
            return;
        }
        std::cout << "dispatcher: create-request: creating: type=" << type << ", request=" << request << std::endl;

        Algorithm::algorithm_t<T> *algorithm = 0;
        if( name == "hdp" )
            algorithm = new Algorithm::hdp_t<T>(problem);
        else if( name == "improved-lao" )
            algorithm = new Algorithm::improved_lao_t<T>(problem);
        else if( name == "ldfs" )
            algorithm = new Algorithm::ldfs_t<T>(problem);
        else if( name == "ldfs-plus" )
            algorithm = new Algorithm::ldfs_plus_t<T>(problem);
        else if( (name == "standard-lrtdp") || (name == "lrtdp") )
            algorithm = new Algorithm::standard_lrtdp_t<T>(problem);
        else if( name == "uniform-lrtdp" )
            algorithm = new Algorithm::uniform_lrtdp_t<T>(problem);
        else if( name == "bounded-lrtdp" )
            algorithm = new Algorithm::bounded_lrtdp_t<T>(problem);
        else if( name == "plain-check" )
            algorithm = new Algorithm::plain_check_t<T>(problem);
        else if( (name == "simple-a*" ) || (name == "simple-astar") )
            algorithm = new Algorithm::simple_astar_t<T>(problem);
        else if( name == "value-iteration" )
            algorithm = new Algorithm::value_iteration_t<T>(problem);

        if( algorithm != 0 ) {
            algorithm->set_parameters(parameters, *this);
            algorithms_.insert(std::make_pair(request, algorithm));
            return;
        }
    }

    // heuristics
    if( type == "heuristic" ) {
        if( fetch_heuristic(request) != 0 ) {
#ifdef DEBUG
            std::cout << "dispatcher: create-request: found heuristic '" << request << "'" << std::endl;
#endif
            return;
        }
        std::cout << "dispatcher: create-request: creating: type=" << type << ", request=" << request << std::endl;

        Heuristic::heuristic_t<T> *heuristic = 0;
        if( name == "zero" )
            heuristic = new Heuristic::zero_heuristic_t<T>(problem);
        else if( name == "min-min" )
            heuristic = new Heuristic::min_min_heuristic_t<T>(problem);
        else if( name == "optimal" )
            heuristic = new Heuristic::optimal_heuristic_t<T>(problem);
        else if( name == "scaled" )
            heuristic = new Heuristic::scaled_heuristic_t<T>(problem);

        if( heuristic != 0 ) {
            heuristic->set_parameters(parameters, *this);
            heuristics_.insert(std::make_pair(request, heuristic));
            return;
        }
    }

    // policies
    if( type == "policy" ) {
        if( fetch_policy(request) != 0 ) {
#ifdef DEBUG
            std::cout << "dispatcher: create-request: found policy '" << request << "'" << std::endl;
#endif
            return;
        }
        std::cout << "dispatcher: create-request: creating: type=" << type << ", request=" << request << std::endl;

        Online::Policy::policy_t<T> *policy = 0;
        if( name == "optimal" )
            policy = new Online::Policy::optimal_policy_t<T>(problem);
        else if( name == "greedy" )
            policy = new Online::Policy::base_greedy_t<T>(problem);
        else if( name == "random" )
            policy = new Online::Policy::random_t<T>(problem);
        else if( name == "rollout" )
            policy = new Online::Policy::Rollout::nested_rollout_t<T>(problem);
        else if( name == "uct" )
            policy = new Online::Policy::UCT::uct_t<T>(problem);
        else if( name == "aot" )
            policy = new Online::Policy::AOT::aot_t<T>(problem);
        else if( name == "aot-gh" )
            assert(0);
        else if( name == "aot-path" )
            assert(0);
        else if( name == "pac-tree" )
            policy = new Online::Policy::PAC::pac_tree_t<T>(problem);
        else if( name == "finite-horizon-lrtdp" )
            policy = new Online::Policy::RTDP::finite_horizon_lrtdp_t<T>(problem);

        if( policy != 0 ) {
            policy->set_parameters(parameters, *this);
            policies_.insert(std::make_pair(request, policy));
            return;
        }
    }

    // if this far, request is not recognized
    std::cout << "error: dispatcher: create-request: unrecognized: type=" << type << ", request=" << request << std::endl;
}

template<typename T> void dispatcher_t<T>::solve(const std::string &name, const Algorithm::algorithm_t<T> &algorithm, const T &s, solve_result_t &result) const {
    std::cout << "dispatcher: solve: " << name << std::endl;
    const Problem::problem_t<T> &problem = algorithm.problem();

    result.name_ = name;
    result.algorithm_ = &algorithm;
    result.seed_ = algorithm.seed();
    Random::set_seed(result.seed_);

    float start_time = Utils::read_time_in_seconds();
    Problem::hash_t<T> *hash = new Problem::hash_t<T>(problem);
    algorithm.solve(s, *hash);
    float end_time = Utils::read_time_in_seconds();

    // extract states from problem
    result.problem_expansions_ = problem.expansions();

    // extract stats from hash
    result.hash_ = hash;
    result.hash_value_ = hash->value(s);
    result.hash_updates_ = hash->updates();
    result.hash_policy_size_ = std::numeric_limits<unsigned>::max();
    if( (name.substr(0, 12) != "simple_astar") && (name.substr(0, 9) == "simple_a*") )
        result.hash_policy_size_ = problem.policy_size(*hash, s);

    // extract stats from heuristic (stats must be extract here because heuristic mabe shared)
    const Heuristic::heuristic_t<T> *heuristic = algorithm.heuristic();
    result.heuristic_ = heuristic;
    result.heuristic_eval_time_ = heuristic == 0 ? -1 : heuristic->eval_time();
    result.heuristic_setup_time_ = heuristic == 0 ? -1 : heuristic->setup_time();
    result.heuristic_evaluations_ = heuristic == 0 ? -1 : heuristic->evaluations();
    result.heuristic_size_ = heuristic == 0 ? -1 : heuristic->size();

    // time stats
    result.time_raw_ = end_time - start_time;
    result.time_heuristic_ = algorithm.heuristic() == 0 ? 0 : algorithm.heuristic()->eval_time();
    result.time_algorithm_ = result.time_raw_ - result.time_heuristic_;
}

template<typename T> void dispatcher_t<T>::print(std::ostream &os, const solve_result_t &result) const {
    std::cout << "solve-stats: name= " << result.name_
              << " seed= " << result.seed_
              << " problem.expansions= " << result.problem_expansions_
              << " hash.value= " << result.hash_value_
              << " hash.updates= " << result.hash_updates_
              << " hash.policy-size= " << result.hash_policy_size_
              << " heuristic.eval-time= " << result.heuristic_eval_time_
              << " heuristic.setup-time= " << result.heuristic_setup_time_
              << " heuristic.evaluations= " << result.heuristic_evaluations_
              << " heuristic.size= " << result.heuristic_size_
              << " time.raw= " << result.time_raw_
              << " time.heuristic= " << result.time_heuristic_
              << " time.algorithm= " << result.time_algorithm_
              << std::endl;
}

template<typename T> void dispatcher_t<T>::evaluate(const std::string &name, const Online::Policy::policy_t<T> &policy, const T &s, evaluate_result_t &result, unsigned num_trials, unsigned max_evaluation_depth, bool verbose) const {
    std::cout << "dispatcher: evaluate: " << name << std::endl;
    const Problem::problem_t<T> &problem = policy.problem();
    policy.reset_stats();

    result.name_ = name;
    result.policy_ = &policy;
    result.seed_ = policy.seed();
    Random::set_seed(result.seed_);

    float start_time = Utils::read_time_in_seconds();
    std::pair<float, float> p = Online::Evaluation::evaluation_with_stdev(policy, problem.init(), num_trials, max_evaluation_depth, verbose);
    result.eval_value_ = p.first;
    result.eval_stdev_ = p.second;
    float end_time = Utils::read_time_in_seconds();

    // extract states from problem
    result.problem_expansions_ = problem.expansions();

    result.time_raw_ = end_time - start_time;
    result.time_policy_ = 0; //XXXX time for policy
    result.time_heuristic_ = 0; //XXXX time for heuristic
    result.time_algorithm_ = result.time_raw_ - result.time_policy_ - result.time_heuristic_;
}

template<typename T> void dispatcher_t<T>::print(std::ostream &os, const evaluate_result_t &result) const {
    std::cout << "evaluate-stats: name= " << result.name_
              << " seed= " << result.seed_
              << " problem.expansions= " << result.problem_expansions_
              << " eval.value= " << result.eval_value_
              << " eval.stdev= " << result.eval_stdev_
#if 0
              << " hash.value= " << result.hash_value_
              << " hash.updates= " << result.hash_updates_
              << " hash.policy-size= " << result.hash_policy_size_
              << " heuristic.eval-time= " << result.heuristic_eval_time_
              << " heuristic.setup-time= " << result.heuristic_setup_time_
              << " heuristic.evaluations= " << result.heuristic_evaluations_
              << " heuristic.size= " << result.heuristic_size_
#endif
              << " time.raw= " << result.time_raw_
              << " time.policy= " << result.time_policy_
              << " time.heuristic= " << result.time_heuristic_
              << " time.algorithm= " << result.time_algorithm_
              << std::endl;
}

}; // namespace Dispatcher


namespace Online {

namespace Evaluation {

}; // namespace Evaluation

}; // namespace Online

#undef DEBUG

#endif

