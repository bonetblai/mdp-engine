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

#ifndef IMPROVED_LAO_H
#define IMPROVED_LAO_H

#include "algorithm.h"

#include <cassert>
#include <list>
#include <string>
#include <vector>

//#define DEBUG

namespace Algorithm {

template<typename T> class improved_lao_t : public algorithm_t<T> {
  using algorithm_t<T>::problem_;
  using algorithm_t<T>::heuristic_;
  using algorithm_t<T>::seed_;
  protected:
    float epsilon_;

    class policy_graph_t {
      protected:
        typedef typename std::list<std::pair<T, Hash::data_t*> > pair_list;
        typedef typename std::vector<T>::iterator vector_iterator;
        typedef typename pair_list::iterator list_iterator;

        const Problem::problem_t<T> &problem_;
        Problem::hash_t<T> &hash_;
        size_t size_;
        std::vector<T> roots_;
        pair_list tips_;
        pair_list nodes_;

      public:
        policy_graph_t(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash)
          : problem_(problem), hash_(hash), size_(0) { }
        ~policy_graph_t() { }

        size_t size() const { return size_; }
        void add_root(const T &s) { roots_.push_back(s); }
        const pair_list& tips() const { return tips_; }
        const pair_list& nodes() const { return nodes_; }

        void recompute() {
            std::vector<std::pair<T, float> > outcomes;
            size_ = 0;
            tips_.clear();
            nodes_.clear();

            pair_list open;
            for( vector_iterator si = roots_.begin(); si != roots_.end(); ++si ) {
                Hash::data_t *dptr = hash_.data_ptr(*si);
                open.push_back(std::make_pair(*si, dptr));
                dptr->solve();
                dptr->mark();
            }

            while( !open.empty() ) {
                std::pair<T, Hash::data_t*> n = open.front();
                open.pop_front();

                if( problem_.terminal( n.first ) ) {
                    n.second->set_action(Problem::noop);
                    nodes_.push_front(n);
                    ++size_;
                } else {
                    bool unsolved = false;
                    std::pair<Problem::action_t, float> p = hash_.bestQValue(n.first);
                    assert(p.first != Problem::noop);
                    n.second->set_action(p.first);

                    problem_.next(n.first, p.first, outcomes);
                    unsigned osize = outcomes.size();

                    for( unsigned i = 0; i < osize; ++i ) {
                        if( !hash_.solved(outcomes[i].first) ) {
                            unsolved = true;
                            break;
                        }
                    }

                    ++size_;
                    if( !unsolved ) {
                        for( unsigned i = 0; i < osize; ++i ) {
                            Hash::data_t *dptr = hash_.data_ptr(outcomes[i].first);
                            if( !dptr->marked() ) {
                                open.push_back(std::make_pair(outcomes[i].first, dptr));
                                dptr->solve();
                                dptr->mark();
                            }
                        }
                        nodes_.push_front(n);
                    } else {
                        tips_.push_back(n);
                        n.second->set_action(Problem::noop);
                    }
                }
            }

            // unmark nodes
            for( list_iterator si = nodes_.begin(); si != nodes_.end(); ++si )
                si->second->unmark();
            for( list_iterator si = tips_.begin(); si != tips_.end(); ++si )
                si->second->unmark();
        }

        void postorder_dfs(const T &s, pair_list &visited) {
            pair_list open;

            std::vector<std::pair<T, float> > outcomes;
            Hash::data_t *dptr = hash_.data_ptr(s);
            open.push_back(std::make_pair(s, dptr ));
            dptr->mark();

            while( !open.empty() ) {
                std::pair<T, Hash::data_t*> n = open.back();
                visited.push_front(n);
                open.pop_back();
                assert(n.second->solved());

                if( n.second->action() != Problem::noop ) {
                    problem_.next(n.first, n.second->action(), outcomes);
                    unsigned osize = outcomes.size();
                    for( unsigned i = 0; i < osize; ++i ) {
                        Hash::data_t *dptr = hash_.data_ptr(outcomes[i].first);
                        if( !dptr->marked() ) {
                            open.push_back(std::make_pair(outcomes[i].first, dptr));
                            dptr->mark();
                        }
                    }
                } else {
                    std::pair<Problem::action_t, float> p = hash_.bestQValue(n.first);
                    problem_.next(n.first, p.first, outcomes);
                    unsigned osize = outcomes.size();
                    for( unsigned i = 0; i < osize; ++i )
                        hash_.solve(outcomes[i].first);
                }
            }

            // unmark nodes
            for( list_iterator si = visited.begin(); si != visited.end(); ++si )
                si->second->unmark();
        }

        void update(pair_list &visited) {
            for( list_iterator si = visited.begin(); si != visited.end(); ++si ) {
                std::pair<Problem::action_t, float> p = hash_.bestQValue(si->first);
                si->second->update(p.second);
                hash_.inc_updates();
            }
        }
    };

    improved_lao_t(const Problem::problem_t<T> &problem,
                   float epsilon,
                   const Heuristic::heuristic_t<T> *heuristic)
      : algorithm_t<T>(problem),
        epsilon_(epsilon) {
        heuristic_ = heuristic;
    }

  public:
    improved_lao_t(const Problem::problem_t<T> &problem) : algorithm_t<T>(problem) { }
    virtual ~improved_lao_t() { }
    virtual algorithm_t<T>* clone() const {
        return new improved_lao_t(problem_, epsilon_, heuristic_);
    }
    virtual std::string name() const {
        return std::string("improved-lao(heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
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
        std::cout << "ILAO: params: epsilon=" << epsilon_ << ", heuristic=" << (heuristic_ == 0 ? std::string("null") : heuristic_->name()) << ", seed=" << seed_ << std::endl;
    }

    virtual void solve(const T &s, Problem::hash_t<T> &hash) const {
        Heuristic::wrapper_t<T> eval_function(heuristic_);
        if( heuristic_ != 0 ) heuristic_->reset_stats();
        hash.set_eval_function(&eval_function);
        hash.clear();

        typedef typename std::list<std::pair<T, Hash::data_t*> > pair_list;
        typedef typename pair_list::const_iterator const_list_iterator;
        pair_list visited;

        policy_graph_t graph(problem_, hash);
        graph.add_root(s);
        graph.recompute();

        size_t iterations = 0;

      loop:
        ++iterations;
        while( !graph.tips().empty() ) {
            visited.clear();
            graph.postorder_dfs(s, visited);
            graph.update(visited);
            graph.recompute();
        }

        // convergence test
        float residual = 1.0 + epsilon_;
        while( residual > epsilon_ ) {
            residual = 0.0;
            for( const_list_iterator si = graph.nodes().begin(); si != graph.nodes().end(); ++si ) {
                float hv = si->second->value();
                std::pair<Problem::action_t, float> p = hash.bestQValue(si->first);
                residual = Utils::max(residual, (float)fabs(p.second - hv));
                si->second->update(p.second);
                hash.inc_updates();
            }
            graph.recompute();
            if( !graph.tips().empty() ) goto loop;
        }
        hash.set_eval_function(0);
    }
};

}; // namespace Algorithm

#undef DEBUG

#endif

