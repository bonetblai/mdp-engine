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

#ifndef SIMPLE_ASTAR_H
#define SIMPLE_ASTAR_H

#include "algorithm.h"

#include <cassert>
#include <queue>
#include <string>
#include <vector>

//#define DEBUG

namespace Algorithm {

template<typename T> class simple_astar_t : public algorithm_t<T> {
  using algorithm_t<T>::problem_;
  using algorithm_t<T>::heuristic_;
  using algorithm_t<T>::seed_;
  protected:
    struct min_priority_t {
        bool operator()(const std::pair<T, Hash::data_t*> &p1, const std::pair<T, Hash::data_t*> &p2) const {
            return p1.second->f() > p2.second->f();
        }
    };
    typedef typename std::priority_queue<std::pair<T, Hash::data_t*>,
                                         std::vector<std::pair<T, Hash::data_t*> >,
                                         min_priority_t>
                     priority_queue_t;

    simple_astar_t(const Problem::problem_t<T> &problem, const Heuristic::heuristic_t<T> *heuristic)
      : algorithm_t<T>(problem) {
        heuristic_ = heuristic;
    }

  public:
    simple_astar_t(const Problem::problem_t<T> &problem) : algorithm_t<T>(problem) { }
    virtual ~simple_astar_t() { }
    virtual algorithm_t<T>* clone() const {
        return new simple_astar_t(problem_, heuristic_);
    }
    virtual std::string name() const {
        return std::string("simple-astar(heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
          std::string(",seed=") + std::to_string(seed_) + ")";
    }

    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("heuristic");
        if( it != parameters.end() ) {
            delete heuristic_;
            dispatcher.create_request(problem_, it->first, it->second);
            heuristic_ = dispatcher.fetch_heuristic(it->second);
        }
        it = parameters.find("seed");
        if( it != parameters.end() ) seed_ = strtol(it->second.c_str(), 0, 0);
#ifdef DEBUG
        std::cout << "debug: simple-a*(): params:"
                  << " heuristic=" << (heuristic_ == 0 ? std::string("null") : heuristic_->name())
                  << " seed=" << seed_
                  << std::endl;
#endif
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        reset_stats(hash);
        Heuristic::wrapper_t<T> eval_function(heuristic_);
        hash.set_eval_function(&eval_function);

        priority_queue_t open;
        std::vector<std::pair<T, float> > outcomes;

        Hash::data_t *dptr = hash.data_ptr(s);
        dptr->set_g(0);
        dptr->set_parent(0);
        dptr->set_action(Problem::noop);
        dptr->mark();
        open.push(std::make_pair(s, dptr));

#ifdef DEBUG
        std::cout << "PUSH " << "state" ///s
                  << " w/ g=" << dptr->g()
                  << " and h=" << dptr->h()
                  << " => f=" << dptr->f()
                  << std::endl;
        std::cout << "queue.sz=" << open.size() << std::endl;
#endif

        while( !open.empty() ) {
            std::pair<T, Hash::data_t*> n = open.top();
            open.pop();

#ifdef DEBUG
            std::cout << "POP  " << "state: " << n.first
                      << " w/ g=" << n.second->g()
                      << " and h=" << n.second->h()
                      << " => f=" << n.second->f()
                      << std::endl;
            std::cout << "queue.sz=" << open.size() << std::endl;

            std::vector<const Hash::data_t*> path;
            path.reserve(n.second->g());
            for( const Hash::data_t *dptr = n.second; dptr != 0; dptr = dptr->parent() )
                path.push_back(dptr);

            std::cout << "path=<";
            for( size_t i = path.size() - 1; i > 0; --i )
                std::cout << path[i-1]->action() << ",";
            std::cout << ">" << std::endl;
#endif

            // check for termination
            if( problem_.terminal(n.first) ) {
#ifdef DEBUG
                std::cout << "GOAL FOUND!" << std::endl;
#endif

                std::vector<const Hash::data_t*> plan;
                plan.reserve(n.second->g());
                for( const Hash::data_t *dptr = n.second; dptr != 0; dptr = dptr->parent() )
                    plan.push_back(dptr);

                std::cout << "plan=<";
                for( size_t i = plan.size() - 1; i > 0; --i )
                    std::cout << plan[i-1]->action() << ",";
                std::cout << ">:" << plan.size() - 1 << std::endl;

                //return (int)n.second->g(); // NEED FIX
            }

            // expand state
            for( Problem::action_t a = 0; a < problem_.number_actions(n.first); ++a ) {
                if( problem_.applicable(n.first, a) ) {
                    problem_.next(n.first, a, outcomes);
                    assert(outcomes.size() == 1);
                    Hash::data_t *ptr = hash.data_ptr(outcomes[0].first);
                    float g = n.second->g() + problem_.cost(n.first, a);
                    if( !ptr->marked() || (g + ptr->h() < ptr->f()) ) {
                        ptr->set_g(g);
                        ptr->set_parent(n.second);
                        ptr->set_action(a);
                        ptr->mark();
                        open.push(std::make_pair(outcomes[0].first, ptr));
#ifdef DEBUG
                        std::cout << "PUSH " << "state: " <<outcomes[0].first
                                  << " w/ g=" << ptr->g()
                                  << " and h=" << ptr->h()
                                  << " => f=" << ptr->f()
                                  << std::endl;
                        std::cout << "queue.sz=" << open.size() << std::endl;
#endif
                    }
                }
            }
        }
        //return std::numeric_limits<size_t>::max(); // NEED FIX
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

