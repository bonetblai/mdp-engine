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

        float time_raw_;
        float time_heuristic_;
        float time_algorithm_;

#if 0
        float value_;
        unsigned trials_;
        unsigned updates_;
        unsigned expansions_;
#endif

        const Problem::hash_t<T> *hash_;
        unsigned policy_size_;
    };

    struct evaluate_result_t {
        std::string name_;
        const Online::Policy::policy_t<T> *policy_;
        unsigned seed_;

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

    Algorithm::algorithm_t<T>* fetch_algorithm(const std::string &request) {
        typename std::map<std::string, Algorithm::algorithm_t<T>*>::const_iterator it = algorithms_.find(request);
        return it == algorithms_.end() ? 0 : it->second;
    }
    Heuristic::heuristic_t<T>* fetch_heuristic(const std::string &request) {
        typename std::map<std::string, Heuristic::heuristic_t<T>*>::const_iterator it = heuristics_.find(request);
        return it == heuristics_.end() ? 0 : it->second;
    }
    Online::Policy::policy_t<T>* fetch_policy(const std::string &request) {
        typename std::map<std::string, Online::Policy::policy_t<T>*>::const_iterator it = policies_.find(request);
        return it == policies_.end() ? 0 : it->second;
    }

    void create_request(const Problem::problem_t<T> &problem, const std::string &request_str);
    void create_request(const Problem::problem_t<T> &problem, const std::string &type, const std::string &request);
    void solve(const std::string &name, const Algorithm::algorithm_t<T> &algorithm, const T &s, solve_result_t &result) const;
    void print(std::ostream &os, const solve_result_t &result) const;
    void evaluate(const std::string &name, const Online::Policy::policy_t<T> &policy, const T &s, evaluate_result_t &result) const;
    void evaluate(const Problem::problem_t<T> &problem, const T &s, const std::string &name, const Online::Policy::policy_t<T> &policy, evaluate_result_t &result) const;
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
#include "pac.h"
#include "online_rtdp.h"

#if 1
//#include "hash.h"
#include "parameters.h"
#include "aot_gh.h"
#include "aot_path.h"
#endif

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
        std::cout << "error: dispatcher: create_request: invalid '" << type << "=" << request << "'" << std::endl;
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
            std::cout << "dispatcher: create_request: found '" << request << "'" << std::endl;
            return;
        }
        std::cout << "dispatcher: create_request: creating '" << request << "'" << std::endl;

        Algorithm::algorithm_t<T> *algorithm = 0;
        if( name == "hdp" )
            algorithm = new Algorithm::hdp_t<T>(problem);
        else if( name == "improved-lao" )
            algorithm = new Algorithm::improved_lao_t<T>(problem);
        else if( name == "ldfs" )
            algorithm = new Algorithm::ldfs_t<T>(problem);
        else if( name == "ldfs-plus" )
            algorithm = new Algorithm::ldfs_plus_t<T>(problem);
        else if( name == "standard-lrtdp" )
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
            std::cout << "dispatcher: create_request: found '" << request << "'" << std::endl;
            return;
        }
        std::cout << "dispatcher: create_request: creating '" << request << "'" << std::endl;

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
        if( fetch_heuristic(request) != 0 ) {
            std::cout << "dispatcher: create_request: found '" << request << "'" << std::endl;
            return;
        }
        std::cout << "dispatcher: create_request: creating '" << request << "'" << std::endl;

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
    std::cout << "error: dispatcher: create_request: unrecognized '" << type << "=" << request << "'" << std::endl;
}

template<typename T> void dispatcher_t<T>::solve(const std::string &name, const Algorithm::algorithm_t<T> &algorithm, const T &s, solve_result_t &result) const {
    std::cout << "solving " << name << std::endl;
    const Problem::problem_t<T> &problem = algorithm.problem();

    result.name_ = name;
    result.algorithm_ = &algorithm;
    result.seed_ = algorithm.seed();
    Random::set_seed(result.seed_);

    float start_time = Utils::read_time_in_seconds();
    Problem::hash_t<T> *hash = new Problem::hash_t<T>(problem);
    algorithm.solve(s, *hash);
    float end_time = Utils::read_time_in_seconds();

    result.hash_ = hash;
    result.policy_size_ = std::numeric_limits<unsigned>::max();
    if( (name.substr(0, 12) != "simple_astar") && (name.substr(0, 9) == "simple_a*") )
        result.policy_size_ = problem.policy_size(*hash, s);

#if 0
    result.hash_ = new Problem::hash_t<T>(problem, new Heuristic::wrapper_t<T>(heuristic));
    problem.clear_expansions();
    if( heuristic != 0 ) heuristic->reset_stats();
    //result.value_ = result.hash_->value(s);
    //result.updates_ = result.hash_->updates();
    //result.expansions_ = problem.expansions();
    //result.policy_size_ = std::numeric_limits<unsigned>::max();
    if( (name.substr(0, 12) != "simple_astar") && (name.substr(0, 9) == "simple_a*") )
        ;//result.policy_size = problem.policy_size(*result.hash_, s);
#endif

    result.time_raw_ = end_time - start_time;
    result.time_heuristic_ = algorithm.heuristic() == 0 ? 0 : algorithm.heuristic()->eval_time();
    result.time_algorithm_ = result.time_raw_ - result.time_heuristic_;
}

template<typename T> void dispatcher_t<T>::print(std::ostream &os, const solve_result_t &result) const {
    std::cout << "HOLA: " << result.name_ << std::endl;
    std::cout << "updates=" << result.hash_->updates() << std::endl;
}

template<typename T> void dispatcher_t<T>::evaluate(const std::string &name, const Online::Policy::policy_t<T> &policy, const T &s, evaluate_result_t &result) const {
    std::cout << "evaluating " << name << std::endl;
    const Problem::problem_t<T> &problem = policy.problem();

    result.name_ = name;
    result.policy_ = &policy;
    result.seed_ = policy.seed();
    Random::set_seed(result.seed_);

    float start_time = Utils::read_time_in_seconds();
    //Problem::hash_t<T> *hash = new Problem::hash_t<T>(problem);
    //algorithm.solve(s, *hash);
    float end_time = Utils::read_time_in_seconds();
}

template<typename T> void dispatcher_t<T>::evaluate(const Problem::problem_t<T> &problem, const T &s, const std::string &name, const Online::Policy::policy_t<T> &policy, evaluate_result_t &result) const {
    const Heuristic::heuristic_t<T> *heuristic = 0;//policy.heuristic_;

    //result.name_ = name;
    //result.policy_ = &policy;
    //result.seed_ = policy.seed_;
    //Random::set_seed(result.seed_);

    float start_time = Utils::read_time_in_seconds();
    problem.clear_expansions();
    if( heuristic != 0 ) heuristic->reset_stats();

    //std::pair<std::pair<float, float>, float> eval = Online::Evaluation::evaluate_policy(policy, eval_pars, true);
    //result.mean_ = 0;
    //result.std_ = 0;

    //result.expansions_ = problem.expansions();
    //result.decisions_ = policy.decisions();
    //float end_time = Utils::read_time_in_seconds();
    //result.total_time_ = end_time - start_time;
}

template<typename T> void dispatcher_t<T>::print(std::ostream &os, const evaluate_result_t &result) const {
}

template<typename T> struct result_t {
    int algorithm_;
    const char *algorithm_name_;
    unsigned seed_;
    float value_;
    unsigned trials_;
    unsigned updates_;
    unsigned expansions_;
    unsigned psize_;
    float atime_;
    float htime_;
    Problem::hash_t<T> *hash_;
};

template<typename T>
inline void print_result(std::ostream &os, const result_t<T> *result) {
    os << std::fixed;
    if( result == 0 ) {
         os << std::setw(4) << "#" << " "
            << std::setw(7) << "alg" << " "
            << std::setw(12) << "V*(s0)" << " "
            << std::setw(12) << "trials" << " "
            << std::setw(12) << "updates" << " "
            << std::setw(12) << "expansions" << " "
            << std::setw(12) << "hashsz" << " "
            << std::setw(12) << "psize" << " "
            << std::setw(12) << "atime" << " "
            << std::setw(12) << "htime"
            << std::endl;
    } else {
         os << std::setw(4) << result->algorithm_ << " "
            << std::setw(7) << result->algorithm_name_ << " "
            << std::setw(12) << std::setprecision(5) << result->value_ << std::setprecision(2) << " "
            << std::setw(12) << result->trials_ << " "
            << std::setw(12) << result->updates_ << " "
            << std::setw(12) << result->expansions_ << " "
            << std::setw(12) << result->hash_->size() << " "
            << std::setw(12) << result->psize_ << " "
            << std::setw(12) << result->atime_ << " "
            << std::setw(12) << result->htime_
            << std::endl;
    }
}

}; // namespace Dispatcher


namespace Online {

namespace Evaluation {

template<typename T>
const Policy::policy_t<T>*
  fetch_policy(const std::string &name,
               std::vector<std::pair<const Policy::policy_t<T>*, std::string> > &base_policies) {
    const Policy::policy_t<T> *policy = 0;
    for( unsigned i = 0; i < base_policies.size(); ++i ) {
        if( base_policies[i].second == name ) {
            policy = base_policies[i].first;
            break;
        }
    }
    return policy;
}

template<typename T>
const Heuristic::heuristic_t<T>*
  fetch_heuristic(const std::string &name,
                  std::vector<std::pair<const Heuristic::heuristic_t<T>*, std::string> > &heuristics) {
    const Heuristic::heuristic_t<T> *heuristic = 0;
    for( unsigned i = 0; i < heuristics.size(); ++i ) {
        if( heuristics[i].second == name ) {
            heuristic = heuristics[i].first;
            break;
        }
    }
    return heuristic;
}

inline bool policy_requires_base_policy(const std::string &policy_type) {
    if( policy_type == "finite-horizon-lrtdp" ) {
        return false;
    } else if( !policy_type.compare(0, 3, "aot") ) {
        return (policy_type.find("heuristic") == std::string::npos) &&
               (policy_type.find("g+h") == std::string::npos) &&
               (policy_type.find("path") == std::string::npos);
    } else {
        return true;
    }
}

inline bool policy_requires_heuristic(const std::string &policy_type) {
    if( (policy_type == "finite-horizon-lrtdp") || !policy_type.compare(0, 3, "pac") ) {
        return true;
    } else if( !policy_type.compare(0, 3, "aot") ) {
        return (policy_type.find("heuristic") != std::string::npos) ||
               (policy_type.find("g+h") != std::string::npos) ||
               (policy_type.find("path") != std::string::npos);
    } else {
        return false;
    }
}

template<typename T>
inline std::pair<const Policy::policy_t<T>*, std::string>
  select_policy(const Problem::problem_t<T> &problem,
                const std::string &base_name,
                const std::string &policy_type,
                std::vector<std::pair<const Policy::policy_t<T>*, std::string> > &base_policies,
                std::vector<std::pair<const Heuristic::heuristic_t<T>*, std::string> > &heuristics,
                const parameters_t &par) {

    std::stringstream ss;

    // fetch base policy/heuristic
    const Policy::policy_t<T> *base_policy = 0;
    if( policy_requires_base_policy(policy_type) ) {
        base_policy = fetch_policy(base_name, base_policies);
        if( base_policy == 0 ) {
            ss << "error: inexistent base policy: " << base_name;
            return std::make_pair(base_policy, ss.str());
        }
    }

    const Heuristic::heuristic_t<T> *heuristic = 0;
    if( policy_requires_heuristic(policy_type) ) {
        heuristic = fetch_heuristic(base_name, heuristics);
        if( heuristic == 0 ) {
            ss << "error: inexistent heuristic: " << base_name;
            return std::make_pair(base_policy, ss.str());
        }
    }

    // make compound policy
    const Policy::policy_t<T> *policy = 0;

    if( policy_type == "direct" ) {
        ss << base_name;
        policy = base_policy->clone();
    } else if( policy_type == "rollout" ) {
        ss << policy_type << "(" << base_name
           << ",width=" << par.width_
           << ",depth=" << par.depth_
           << ",nesting=" << par.par1_
           << ")";
        //policy = Policy::make_nested_rollout(*base_policy, par.width_, par.depth_, par.par1_);
    } else if( (policy_type.length() >= 3) && !policy_type.compare(0, 3, "uct") ) {
        // UCT family
        ss << policy_type << "(" << base_name
           << ",width=" << par.width_
           << ",depth=" << par.depth_
           << ",par=" << par.par1_
           << ")";
        bool random_ties = policy_type == "uct/random-ties";
        //policy = Policy::make_uct(*base_policy, par.width_, par.depth_, par.par1_, random_ties);
    } else if( (policy_type.length() >= 3) && !policy_type.compare(0, 3, "pac") ) {
        // Determine type and modifiers
        bool random_ties = false;
        bool pac_tree = false;
        if( policy_type.length() > 3 ) {
            random_ties = policy_type.find("random-ties") != std::string::npos;
            pac_tree = policy_type.find("tree") != std::string::npos;
        }

        // PAC family
        std::stringstream pac_name;
        pac_name << "pac/";
        if( pac_tree ) pac_name << "tree,";
        if( random_ties ) pac_name << "random-ties,";
        std::string tmp_name = pac_name.str();
        tmp_name.erase(tmp_name.size() - 1, 1);

        ss << tmp_name << "(" << base_name
           << ",width=" << par.width_
           << ",depth=" << par.depth_
           << ",par=" << par.par1_
           << ")";

        // Make sure we have some base_policy to construct PAC
std::cout << "HOLA (dispatcher.h): base policy=" << base_policy << std::endl;
        if( base_policy == 0 ) base_policy = new Policy::random_t<T>(problem);
std::cout << "HOLA (dispatcher.h): " << ss.str() << std::endl;

        if( pac_tree ) {
            //policy = Policy::make_pac_tree(*base_policy, par.width_, par.depth_, par.par1_, random_ties);
            //dynamic_cast<const Online::Policy::PAC::pac_tree_t<T>*>(policy)->set_parameters(0.1, 0.1, 10, .8, heuristic); // epsilon, delta, max-num-samples, heuristic
        }
    } else if( (policy_type.length() >= 3) && !policy_type.compare(0, 3, "aot") ) {
        // Determine type and modifiers
        bool random_ties = false;
        bool delayed = false;
        bool random_leaf = false;
        bool g_plus_h = false;
        bool path = false;
        if( policy_type.length() > 3 ) {
            random_ties = policy_type.find("random-ties") != std::string::npos;
            delayed = policy_type.find("delayed") != std::string::npos;
            random_leaf = policy_type.find("random-leaf") != std::string::npos;
            g_plus_h = policy_type.find("g+h") != std::string::npos;
            path = policy_type.find("path") != std::string::npos;
        }

        // Constraint: delayed => not random-leaf, not heuristic, not g+h
        // Constraint: random-leaf => not delayed, not heuristic, not g+h
        // Constraint: g+h => not delayed, not random_leaf, heuristic
        if( delayed && random_leaf ) {
            ss << "error: AOT/delayed & AOT/random-leaf are incompatible.";
        } else if( delayed && (heuristic != 0) ) {
            ss << "error: AOT/delayed & AOT/heuristic are incompatible.";
        } else if( delayed && g_plus_h ) {
            ss << "error: AOT/delayed & AOT/g+h are incompatible.";
        } else if( random_leaf && heuristic ) {
            ss << "error: AOT/random-leaf & AOT/heuristic are incompatible.";
        } else if( random_leaf && g_plus_h ) {
            ss << "error: AOT/random-leaf & AOT/g+h are incompatible.";
        } else if( g_plus_h && (heuristic == 0) ) {
            ss << "error: AOT/g+h required AOT/heuristic.";
        }
        if( ss.str().length() > 0 ) return std::make_pair(policy, ss.str());
            
        // AOT family
        std::stringstream aot_name;
        aot_name << "aot/";
        if( heuristic ) aot_name << "heuristic,";
        if( random_leaf ) aot_name << "random-leaf,";
        if( g_plus_h ) aot_name << "g+h,";
        if( path ) aot_name << "path,";
        if( delayed ) aot_name << "delayed,";
        if( random_ties ) aot_name << "random-ties,";
        std::string tmp_name = aot_name.str();
        tmp_name.erase(tmp_name.size() - 1, 1);
        
        ss << tmp_name << "(" << base_name;
        if( g_plus_h ) ss << ",w=" << par.weight_;
        ss << ",width=" << par.width_
           << ",depth=" << par.depth_
           << ",p=" << par.par1_
           << ",exp=" << par.par2_
           << ")";

        // Make sure we have some base_policy to construct AOT
        if( base_policy == 0 ) base_policy = new Policy::random_t<T>(problem);
            
        if( random_leaf ) {
            //policy = Policy::make_aot(*base_policy, par.width_, par.depth_, par.par1_, random_ties, false, par.par2_, 1, 1, 1);
        } else if( g_plus_h ) {
            policy = Policy::make_aot_gh(*base_policy, par.weight_, par.width_, par.depth_, par.par1_, random_ties, false, par.par2_);
        } else if( path ) {
#ifdef EXPERIMENTAL
            extern const Policy::policy_t<T> *global_base_policy;
            policy = Policy::make_aot_path(*global_base_policy, par.width_, par.depth_, par.par1_, random_ties, false, par.par2_);
#else
            std::cout << "'path' option not supported (enable EXPERIMENTAL setup)" << std::endl;
#endif
        } else {
            //policy = Policy::make_aot(*base_policy, par.width_, par.depth_, par.par1_, random_ties, delayed, par.par2_);
        }

        if( heuristic != 0 ) {
            if( g_plus_h ) {
                dynamic_cast<const Online::Policy::AOT_GH::aot_t<T>*>(policy)->set_heuristic(heuristic);
            } else if( path ) {
                dynamic_cast<const Online::Policy::AOT_PATH::aot_t<T>*>(policy)->set_heuristic(heuristic);
            } else {
                dynamic_cast<const Online::Policy::AOT::aot_t<T>*>(policy)->set_heuristic(heuristic);
            }
        }
    } else if( policy_type == "finite-horizon-lrtdp" ) {
        ss << policy_type << "(" << base_name
           << ",horizon=" << par.depth_
           << ",max-trials=" << par.width_
           << ",labeling=" << (par.labeling_ ? "true" : "false")
           << ")";
        //policy = Policy::make_finite_horizon_lrtdp(problem, *heuristic, par.depth_, par.width_, par.labeling_, false);
    } else {
        ss << "inexistent policy: " << policy_type;
    }
    return std::make_pair(policy, ss.str());
}

template<typename T>
inline std::pair<std::pair<float, float>, float>
  evaluate_policy(const Policy::policy_t<T> &policy,
                  const parameters_t &par,
                  bool verbose = false) {
    float start_time = Utils::read_time_in_seconds();
    std::pair<float, float> value =
      Evaluation::evaluation_with_stdev(policy,
                                        policy.problem().init(),
                                        par.evaluation_trials_,
                                        par.evaluation_depth_,
                                        verbose);
    float time = Utils::read_time_in_seconds() - start_time;
    return std::make_pair(value, time);
}

}; // namespace Evaluation

}; // namespace Online

#undef DEBUG

#endif

