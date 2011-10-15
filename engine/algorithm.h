/*
 *  Copyright (C) 2006 Universidad Simon Bolivar
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

#include "problem.h"

#include <cassert>
#include <list>
#include <vector>
#include <math.h>

//#define DEBUG

namespace Algorithm {

template<typename T> class policy_graph_t {
  protected:
    typedef typename std::vector<T>::iterator vector_iterator;
    typedef typename std::list<std::pair<T, Hash::data_t*> >::iterator list_iterator;
    const Problem::problem_t<T> &problem_;
    Problem::hash_t<T> &hash_;
    size_t size_;
    std::vector<T> roots_;
    std::list<std::pair<T, Hash::data_t*> > tips_;
    std::list<std::pair<T, Hash::data_t*> > nodes_;

  public:
    policy_graph_t(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash)
      : problem_(problem), hash_(hash), size_(0) {
    }
    ~policy_graph_t() { }

    size_t size() const { return size_; }
    void add_root(const T &s) { roots_.push_back(s); }
    const std::list<std::pair<T, Hash::data_t*> >& tips() const { return tips_; }
    const std::list<std::pair<T, Hash::data_t*> >& nodes() const { return nodes_; }

    void recompute() {
        size_t osize = 0;
        std::pair<T, float> outcomes[MAXOUTCOMES];

        size_ = 0;
        tips_.clear();
        nodes_.clear();

        std::list<std::pair<T, Hash::data_t*> > open;
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
                problem_.next(n.first, p.first, outcomes, osize);
                for( size_t i = 0; i < osize; ++i ) {
                    if( !hash_.solved(outcomes[i].first) ) {
                        unsolved = true;
                        break;
                    }
                }

                ++size_;
                if( !unsolved ) {
                    for( size_t i = 0; i < osize; ++i ) {
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
            (*si).second->unmark();
        for( list_iterator si = tips_.begin(); si != tips_.end(); ++si )
            (*si).second->unmark();
    }

    void postorder_dfs(const T &s, std::list<std::pair<T, Hash::data_t*> > &visited) {
        size_t osize = 0;
        std::pair<T, float> outcomes[MAXOUTCOMES];

        std::list<std::pair<T, Hash::data_t*> > open;
        Hash::data_t *dptr = hash_.data_ptr(s);
        open.push_back(std::make_pair(s, dptr ));
        dptr->mark();

        while( !open.empty() ) {
            std::pair<T, Hash::data_t*> n = open.back();
            visited.push_front(n);
            open.pop_back();
            assert(n.second->solved());

            if( n.second->action() != Problem::noop ) {
                problem_.next(n.first, n.second->action(), outcomes, osize);
                for( size_t i = 0; i < osize; ++i ) {
                    Hash::data_t *dptr = hash_.data_ptr(outcomes[i].first);
                    if( !dptr->marked() ) {
                        open.push_back(std::make_pair(outcomes[i].first, dptr));
                        dptr->mark();
                    }
                }
            } else {
                std::pair<Problem::action_t, float> p = hash_.bestQValue(n.first);
                problem_.next(n.first, p.first, outcomes, osize);
                for( size_t i = 0; i < osize; ++i )
                    hash_.solve(outcomes[i].first);
            }
        }

        // unmark nodes
        for( list_iterator si = visited.begin(); si != visited.end(); ++si )
            (*si).second->unmark();
    }

    void update(std::list<std::pair<T, Hash::data_t*> > &visited) {
        for( list_iterator si = visited.begin(); si != visited.end(); ++si ) {
            std::pair<Problem::action_t, float> p = hash_.bestQValue((*si).first);
            (*si).second->update(p.second);
            hash_.inc_updates();
        }
    }
};

template<typename T>
void generate_space(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash) {
    size_t osize = 0;
    std::pair<T, float> outcomes[MAXOUTCOMES];
    std::list<std::pair<T, Hash::data_t*> > open;

    Hash::data_t *dptr = hash.data_ptr(problem.init());
    open.push_back(std::make_pair(problem.init(), dptr));
    dptr->mark();

#ifdef DEBUG
    std::cout << "marking " << problem.init() << std::endl;
#endif

    while( !open.empty() ) {
        std::pair<T, Hash::data_t*> n = open.front();
        open.pop_front();
        if( problem.terminal(n.first) ) continue;

        for( Problem::action_t a = 0; a < problem.last_action(); ++a ) {
            problem.next(n.first, a, outcomes, osize);
            for( size_t i = 0; i < osize; ++i ) {
                Hash::data_t *ptr = hash.data_ptr(outcomes[i].first);
                if( !ptr->marked() ) {
                    open.push_back(std::make_pair(outcomes[i].first, ptr));
                    ptr->mark();
#ifdef DEBUG
                    std::cout << "marking " << outcomes[i].first << std::endl;
#endif
                }
            }
        }
    }

    hash.unmark_all();
}

template<typename T>
size_t value_iteration(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t bound, float) {
    typedef typename Problem::hash_t<T>::iterator hash_iterator;
    generate_space(problem, hash);

#ifdef DEBUG
    std::cout << "state space = " << hash.size() << std::endl;
#endif

    size_t iters = 0;
    float residual = 1 + epsilon;
    while( residual > epsilon ) {
        if( iters >= bound ) break;
        residual = 0;
        for( hash_iterator hi = hash.begin(); hi != hash.end(); ++hi ) {
            float hv = (*hi).second->value();
            std::pair<Problem::action_t, float> p = hash.bestQValue((*hi).first);
            float res = (float)fabs(p.second - hv);
            residual = Utils::max(residual, res);
            (*hi).second->update(p.second);
            hash.inc_updates();

#ifdef DEBUG
            if( res > epsilon ) {
                std::cout << "value for " << (*hi).first
                          << " changed from " << hv << " to "
                          << p.second << std::endl;
            }
#endif
        }
        ++iters;

#ifdef DEBUG
        std::cout << "residual=" << residual << std::endl;
#endif
    }
    return iters;
}

template<typename T>
bool check_solved(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, const T &s, float epsilon) {
    std::list<std::pair<T, Hash::data_t*> > open, closed;
    std::pair<T, float> outcomes[MAXOUTCOMES];
    size_t osize = 0;

    Hash::data_t *dptr = hash.data_ptr(s);
    if( !dptr->solved() ) {
        open.push_back(std::make_pair(s, dptr));
        dptr->mark();
    }

    bool rv = true;
    while( !open.empty() ) {
        std::pair<T, Hash::data_t*> n = open.back();
        closed.push_back(n);
        open.pop_back();
        if( problem.terminal(n.first) ) continue;

        std::pair<Problem::action_t, float> p = hash.bestQValue(n.first);
        if( fabs(p.second - n.second->value()) > epsilon ) {
            rv = false;
            continue;
        }

        problem.next(n.first, p.first, outcomes, osize);
        for( size_t i = 0; i < osize; ++ i ) {
            Hash::data_t *dptr = hash.data_ptr(outcomes[i].first);
            if( !dptr->solved() && !dptr->marked() ) {
                open.push_back(std::make_pair(outcomes[i].first, dptr));
                dptr->mark();
            }
        }
    }

    if( rv ) {
        while( !closed.empty() ) {
            closed.back().second->solve();
            closed.pop_back();
        }
    } else {
        while( !closed.empty() ) {
            std::pair<Problem::action_t, float> p = hash.bestQValue(closed.back().first);
            closed.back().second->update(p.second);
            closed.back().second->unmark();
            hash.inc_updates();
            closed.pop_back();
        }
    }
    return rv;
}

template<typename T, int V>
size_t lrtdp_trial(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, const T &s, float epsilon, size_t bound, float greedy) {
    typedef std::pair<T, bool> (Problem::problem_t<T>::*sample_t)(const Problem::hash_t<T>&, const T&, Problem::action_t) const;
    sample_t sample = V == 0 ? &Problem::problem_t<T>::sample : (V == 1 ? &Problem::problem_t<T>::usample : &Problem::problem_t<T>::nsample);

    std::list<T> states;
    std::pair<T, bool> n;

    Hash::data_t *dptr = hash.data_ptr(s);
    states.push_back(s);
    dptr->inc_count();
    T t = s;

#ifdef DEBUG
    std::cout << "trial: begin" << std::endl;
#endif

    size_t steps = 1;
    while( !problem.terminal(t) && !dptr->solved() && (dptr->count() <= bound) ) {

#ifdef DEBUG
        std::cout << "  " << t << " = " << dptr->value() << ":" << hash.heuristic(t) << std::endl;
#endif

        std::pair<Problem::action_t, float> p = hash.bestQValue(t);
        dptr->update(p.second);
        hash.inc_updates();

        if( drand48() < greedy ) {
            n = problem.usample(hash, t, p.first);
        } else {
            n = (problem.*sample)(hash, t, p.first);
        }
        if( !n.second ) break;

        t = n.first;
        dptr = hash.data_ptr(t);
        states.push_back(t);
        dptr->inc_count();
        ++steps;
    }

#ifdef DEBUG
    std::cout << "trial: end" << std::endl;
#endif

    while( !states.empty() ) {
        hash.clear_count(states.back());
        bool solved = check_solved(problem, hash, states.back(), epsilon);
        states.pop_back();
        if( !solved ) break;
    }

    while( !states.empty() ) {
        hash.clear_count(states.back());
        states.pop_back();
    }
    return steps;
}

template<typename T, int V>
size_t lrtdp(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t bound, float greedy) {
    size_t steps = 0, trials = 0;
    const T &init = problem.init();
    while( !hash.solved(init) ) {
        size_t s = lrtdp_trial<T, V>(problem, hash, init, epsilon, bound, greedy);
        steps = Utils::max(steps, s);
        ++trials;
    }
    return trials;
}

template<typename T>
size_t standard_lrtdp(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t bound, float greedy) {
    return lrtdp<T,0>(problem, hash, epsilon, bound, greedy);
}

template<typename T>
size_t uniform_lrtdp(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t bound, float greedy) {
    return lrtdp<T,1>(problem, hash, epsilon, bound, greedy);
}

template<typename T>
size_t bounded_lrtdp(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t bound, float greedy) {
    return lrtdp<T,2>(problem, hash, epsilon, bound, greedy);
}

template<typename T>
size_t improved_lao(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t, float) {
    typedef typename std::list<std::pair<T, Hash::data_t*> >::const_iterator list_const_iterator;
    std::list<std::pair<T, Hash::data_t*> > visited;

    policy_graph_t<T> graph(problem, hash);
    graph.add_root(problem.init());
    graph.recompute();

    size_t iterations = 0;

  loop:
    ++iterations;
    while( !graph.tips().empty() ) {
        visited.clear();
        graph.postorder_dfs(problem.init(), visited);
        graph.update(visited);
        graph.recompute();
    }

    // convergence test
    float residual = 1.0 + epsilon;
    while( residual > epsilon ) {
        residual = 0.0;
        for( list_const_iterator si = graph.nodes().begin(); si != graph.nodes().end(); ++si ) {
            float hv = (*si).second->value();
            std::pair<Problem::action_t, float> p = hash.bestQValue((*si).first);
            residual = Utils::max(residual, (float)fabs(p.second - hv));
            (*si).second->update(p.second);
            hash.inc_updates();
        }
        graph.recompute();
        if( !graph.tips().empty() ) goto loop;
    }
    return iterations;
}

template<typename T>
size_t plain_check(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t, float) {
    size_t trials = 0;
    const T &init = problem.init();
    while( !hash.solved(init) ) {
        check_solved(problem, hash, init, epsilon);
        ++trials;
    }
    return trials;
}

#if 0
template<typename T>
bool lenforce(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, const T &s, Hash::data_t *dptr, float epsilon, size_t &index, size_t kappa, std::list<Hash::data_t*> &stack, std::list<Hash::data_t*> &visited) {
    size_t osize = 0;
    std::pair<T, float> outcomes[MAXOUTCOMES];

    // base cases
    if( dptr->solved() || problem.terminal(s) ) {
        dptr->solve();
        return true;
    }
    assert(!dptr->marked());

    // if residual > epsilon, update and return
    std::pair<Problem::action_t, float> p = hash.bestQValue(problem, s);
    if( fabs(p.second - dptr->value()) > epsilon ) {
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

    // compute normalization constant
    size_t normalization = std::numeric_limits<unsigned>::max();
    problem.next(s, p.first, outcomes, osize);
    for( size_t i = 0; i < osize; ++i ) {
        size_t kv = Utils::kappa_value(outcomes[i].second);
        normalization = Utils::min(normalization, kv);
    }

    // recursive call
    bool flag = true;
    for( size_t i = 0; i < osize; ++i ) {
        size_t normalized_kappa = Utils::kappa_value(outcomes[i].second) - normalization;
        if( normalized_kappa <= kappa ) {
            size_t nkappa = kappa - normalized_kappa;
            Hash::data_t *ptr = hash.data_ptr(outcomes[i].first);
            if( ptr->scc_idx() == std::numeric_limits<unsigned>::max() ) {
                bool rv = lenforce(problem, hash, outcomes[i].first, ptr, epsilon, index, nkappa, stack, visited);
                flag = flag && rv;
                dptr->set_scc_low(Utils::min(dptr->scc_low(), ptr->scc_low()));
            } else if( ptr->marked() ) {
                dptr->set_scc_low(Utils::min(dptr->scc_low(), ptr->scc_idx()));
            }
        }
    }

    // update
    if( !flag ) {
        std::pair<Problem::action_t, float> p = hash.bestQValue(s);
        dptr->update( p.second );
        hash.inc_updates();
        while( !stack.empty() && (stack.front()->scc_idx() >= idx) ) {
            stack.front()->set_scc_low(std::numeric_limits<unsigned>::max());
            stack.front()->set_scc_idx(std::numeric_limits<unsigned>::max());
            stack.pop_front();
        }
    } else if( dptr->scc_low() == dptr->scc_idx() ) {
        while( !stack.empty() && (stack.front()->scc_idx() >= idx) ) {
            stack.front()->solve();
            stack.front()->unmark();
            stack.front()->set_scc_low(std::numeric_limits<unsigned>::max());
            stack.front()->set_scc_idx(std::numeric_limits<unsigned>::max());
            stack.pop_front();
        }
    }
    return flag;
}
#endif

#if 0
template<typename T>
bool enforce(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, const T &s, Hash::data_t *dptr, float epsilon, size_t kappa, std::list<Hash::data_t*> &visited) {
    size_t osize = 0;
    std::pair<T, float> outcomes[MAXOUTCOMES];

    // base cases
    if( dptr->marked() || dptr->solved() || problem.terminal(s) ) {
        return true;
    }

    // mark node
    visited.push_front(dptr);
    dptr->mark();

    // if residual > epsilon, update and return
    std::pair<Problem::action_t, float> p = hash.bestQValue(s);
    if( fabs(p.second - dptr->value()) > epsilon ) {
        dptr->update(p.second);
        hash.inc_updates();
        return false;
    }

    // compute normalization constant
    size_t normalization = std::numeric_limits<unsigned>::max();
    problem.next(s, p.first, outcomes, osize);
    for( size_t i = 0; i < osize; ++i ) {
        size_t kv = Utils::kappa_value(outcomes[i].second);
        normalization = Utils::min(normalization, kv);
    }

    // recursive call
    bool flag = true;
    for( size_t i = 0; i < osize; ++i ) {
        size_t normalized_kappa = Utils::kappa_value(outcomes[i].second) - normalization;
        if( normalized_kappa <= kappa ) {
            size_t nkappa = kappa - normalized_kappa;
            bool rv = enforce(problem, hash, outcomes[i].first, hash.data_ptr( outcomes[i].first ), epsilon, nkappa, visited);
            flag = flag && rv;
        }
    }

    if( !flag ) {
        std::pair<Problem::action_t, float> p = hash.bestQValue(s);
        dptr->update(p.second);
        hash.inc_updates();
    }
    return flag;
}
#endif

#if 0
template<typename T>
size_t hdp_i(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t kappa, float) {
    std::list<Hash::data_t*> visited;
    const T &init = problem.init();
    bool status = false;
    size_t trials = 0;
    while( !status ) {
        status = enforce(problem, hash, init, hash.data_ptr( init ), epsilon, kappa, visited);
        while( !visited.empty() ) {
            visited.front()->unmark();
            visited.pop_front();
        }
        ++trials;
    }
    return trials;
}
#endif

template<typename T>
bool hdp(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, const T &s, Hash::data_t* dptr, float epsilon, size_t &index, std::list<Hash::data_t*> &stack, std::list<Hash::data_t*> &visited, size_t depth) {
    size_t osize = 0;
    std::pair<T, float> outcomes[MAXOUTCOMES];

    // base cases
    if( dptr->solved() || problem.terminal(s) ) {
        dptr->solve();
        return true;
    } else if( dptr->marked() ) {
        return false;
    }

    // if residual > epsilon, update and return
    std::pair<Problem::action_t, float> p = hash.bestQValue(s);
    if( fabs(p.second - dptr->value()) > epsilon ) {
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
    bool flag = true;
    problem.next(s, p.first, outcomes, osize);
    for( size_t i = 0; i < osize; ++i ) {
        Hash::data_t *ptr = hash.data_ptr(outcomes[i].first);
        if( ptr->scc_idx() == std::numeric_limits<unsigned>::max() ) {
            bool rv = hdp(problem, hash, outcomes[i].first, ptr, epsilon, index, stack, visited, 2+depth);
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

template<typename T>
size_t hdp_driver(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t, float) {
    std::list<Hash::data_t*> stack, visited;
    const T &init = problem.init();
    Hash::data_t *dptr = hash.data_ptr(init);
    size_t trials = 0;
    while( !dptr->solved() ) {
        size_t index = 0;
        hdp(problem, hash, init, dptr, epsilon, index, stack, visited, 0);
        assert(stack.empty());
        while( !visited.empty() ) {
            visited.front()->unmark();
            assert(visited.front()->scc_low() == std::numeric_limits<unsigned>::max());
            assert(visited.front()->scc_idx() == std::numeric_limits<unsigned>::max());
            visited.pop_front();
        }
        ++trials;
    }
    return trials;
}

template<typename T, int V>
bool ldfs(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, const T &s, Hash::data_t *dptr, float epsilon, size_t &index, std::list<Hash::data_t*> &stack, std::list<Hash::data_t*> &visited, size_t depth) {
    std::pair<T, float> outcomes[MAXOUTCOMES];
    size_t osize = 0;

    // base cases
    if( dptr->solved() || problem.terminal(s) ) {
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

    if( V == 1 ) {
        std::pair<Problem::action_t, float> p = hash.bestQValue(s);
        dptr->update(p.second);
        hash.inc_updates();
    }

    // expansion
    bool flag = false;
    float bqv = std::numeric_limits<float>::max();
    for( Problem::action_t a = 0; a < problem.last_action(); ++a ) {
        float qv = 0.0;
        problem.next(s, a, outcomes, osize);
        for( size_t i = 0; i < osize; ++i )
            qv += outcomes[i].second * hash.value(outcomes[i].first);
        qv += 1.0;
        bqv = Utils::min(bqv, qv);
        if( fabs(qv - dptr->value()) > epsilon ) continue;
        dptr->mark();
        flag = true;
        for( size_t i = 0; i < osize; ++i ) {
            Hash::data_t *ptr = hash.data_ptr(outcomes[i].first);
            if( ptr->scc_idx() == std::numeric_limits<unsigned>::max() ) {
                bool rv = ldfs<T, V>(problem, hash, outcomes[i].first, ptr, epsilon, index, stack, visited, 2+depth);
                flag = flag && rv;
                dptr->set_scc_low(Utils::min(dptr->scc_low(), ptr->scc_low()));
            } else if( ptr->marked() ) {
                dptr->set_scc_low(Utils::min(dptr->scc_low(), ptr->scc_idx()));
            }
        }
        if( (V == 1) && flag && (hash.QValue(s, a) - dptr->value() > epsilon) ) flag = false;
        if( flag ) break;
        while( stack.front()->scc_idx() > idx ) {
            stack.front()->set_scc_low(std::numeric_limits<unsigned>::max());
            stack.front()->set_scc_idx(std::numeric_limits<unsigned>::max());
            stack.pop_front();
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

template<typename T>
size_t ldfs_plus_driver(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t, float) {
    std::list<Hash::data_t*> stack, visited;
    size_t trials = 0;
    const T &init = problem.init();
    Hash::data_t *dptr = hash.data_ptr(init);
    while( !dptr->solved() ) {
        size_t index = 0;
        ldfs<T, 1>(problem, hash, init, dptr, epsilon, index, stack, visited, 0);
        assert(stack.empty());
        while( !visited.empty() ) {
            visited.front()->unmark();
            visited.pop_front();
        }
        ++trials;
    }
    return trials;
}

template<typename T>
size_t ldfs_driver(const Problem::problem_t<T> &problem, Problem::hash_t<T> &hash, float epsilon, size_t, float) {
    std::list<Hash::data_t*> stack, visited;
    size_t trials = 0;
    const T &init = problem.init();
    Hash::data_t *dptr = hash.data_ptr(init);
    while( !dptr->solved() ) {
        size_t index = 0;
        ldfs<T, 0>(problem, hash, init, dptr, epsilon, index, stack, visited, 0);
        assert(stack.empty());
        while( !visited.empty() ) {
            visited.front()->unmark();
            visited.pop_front();
        }
        ++trials;
    }
    return trials;
}

}; // namespace Algorithm

#undef DEBUG

#endif

