/*
 *  Copyright (c) 2015-2016 Universidad Simon Bolivar
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

#ifndef PAC_H
#define PAC_H

#include "policy.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <queue>
#include <vector>
#include <math.h>

//#define DEBUG
//#define DEBUG2

namespace Online {

namespace Policy {

namespace PAC {

////////////////////////////////////////////////
//
// Nodes
//

template<typename T> struct node_t {
    const T *state_;
    unsigned char depth_;

    float lower_bound_;
    float upper_bound_;
    bool pruned_;

    mutable float lower_bound_est_;
    mutable float upper_bound_est_;

    node_t *parent_;
    std::vector<node_t*> children_;

    node_t(const T &state, unsigned char depth, node_t<T> *parent)
      : state_(new T(state)), depth_(depth),
        lower_bound_(0), upper_bound_(0), pruned_(false),
        lower_bound_est_(0), upper_bound_est_(0),
        parent_(parent) {
    }
    virtual ~node_t() { delete state_; }

    float gap() const { return upper_bound_ - lower_bound_; }
    float gap_est() const { return upper_bound_est_ - lower_bound_est_; }
    virtual void print(std::ostream &os) const = 0;
};

template<typename T> struct action_node_t : public node_t<T> {
    using node_t<T>::state_;
    using node_t<T>::depth_;
    using node_t<T>::children_;

    Problem::action_t action_;
    std::vector<float> prob_;

    action_node_t(const T &state, unsigned char depth, Problem::action_t action, node_t<T> *parent)
      : node_t<T>(state, depth, parent), action_(action) {
    }
    virtual ~action_node_t() { }

    virtual void print(std::ostream &os) const {
        os << "[s=" << *state_
           << ",d=" << int(depth_)
           << ",a=" << action_
           << "]";
    }
};

template<typename T> struct state_node_t : public node_t<T> {
    using node_t<T>::state_;
    using node_t<T>::depth_;
    using node_t<T>::children_;

    mutable int best_child_;
    mutable float score_;

    state_node_t(const T &state, unsigned char depth, node_t<T> *parent)
      : node_t<T>(state, depth, parent), best_child_(-1), score_(0) {
    }
    virtual ~state_node_t() { }

    Problem::action_t select_action(bool random_ties, bool use_upper_bound = true) const {
        assert(!children_.empty());
        float best_value = use_upper_bound ? children_[0]->upper_bound_ : children_[0]->lower_bound_;
        std::vector<Problem::action_t> best_actions;
        best_actions.reserve(random_ties ? children_.size() : 1);
        best_actions.push_back(static_cast<const action_node_t<T>*>(children_[0])->action_);

        for( int i = 1; i < int(children_.size()); ++i ) {
            const action_node_t<T> *a_node = static_cast<const action_node_t<T>*>(children_[i]);
            if( (use_upper_bound && (a_node->upper_bound_ <= best_value)) || (!use_upper_bound && (a_node->lower_bound_ <= best_value)) ) {
                if( (use_upper_bound && (a_node->upper_bound_ < best_value)) || (!use_upper_bound && (a_node->lower_bound_ < best_value)) )
                    best_actions.clear();
                best_value = use_upper_bound ? a_node->upper_bound_ : a_node->lower_bound_;
                best_actions.push_back(a_node->action_);
            }
        }

        assert(!best_actions.empty());
        return best_actions[Random::uniform(best_actions.size())];
    }

    virtual void print(std::ostream &os) const {
        os << "[s=" << *state_
           << ",d=" << int(depth_)
           << "]";
    }
};

template<typename T> struct state_node_comparator_t {
    state_node_comparator_t() { }
    bool operator()(const state_node_t<T> *n1, const state_node_t<T> *n2) const {
        return n1->score_ < n2->score_;
    }
};

template<typename T> inline std::ostream& operator<<(std::ostream &os, const node_t<T> &node) {
    node.print(os);
    return os;
}

////////////////////////////////////////////////
//
// Policy
//

template<typename T> class pac_tree_t : public improvement_t<T> {
  using policy_t<T>::problem_;
  using improvement_t<T>::base_policy_;
  protected:
    unsigned width_;
    unsigned horizon_;
    bool random_ties_;
    float delta_;
    float epsilon_;
    float damping_;
    unsigned max_num_samples_;
    bool soft_pruning_;

    const Heuristic::heuristic_t<T> *heuristic_;

    const Algorithm::algorithm_t<T> *algorithm_; // CHECK
    Problem::hash_t<T> *hash_;
    float g_;

    // computed parameters
    float gamma_;
    std::vector<float> num_samples_;
    std::vector<float> solved_threshold_;

    // leaf nodes
    mutable std::vector<state_node_t<T>*> leaf_nodes_;

    // stats for single action selection
    mutable int num_nodes_pruned_;
    mutable int num_a_nodes_pruned_;
    mutable int num_policy_evaluations_;
    mutable int num_heuristic_evaluations_;

    // overall stats
    mutable int total_num_nodes_pruned_;
    mutable int total_num_a_nodes_pruned_;
    mutable int total_num_policy_evaluations_;
    mutable int total_num_heuristic_evaluations_;

    pac_tree_t(const Problem::problem_t<T> &problem,
               const policy_t<T> *base_policy,
               unsigned width,
               unsigned horizon,
               bool random_ties,
               float delta,
               float epsilon,
               float damping,
               unsigned max_num_samples,
               bool soft_pruning,
               const Heuristic::heuristic_t<T> *heuristic,
               const Algorithm::algorithm_t<T> *algorithm,
               Problem::hash_t<T> *hash,
               float g)
      : improvement_t<T>(problem, base_policy),
        width_(width),
        horizon_(horizon),
        random_ties_(random_ties),
        delta_(delta),
        epsilon_(epsilon),
        damping_(damping),
        max_num_samples_(max_num_samples),
        soft_pruning_(soft_pruning),
        heuristic_(heuristic),
        algorithm_(algorithm),
        hash_(hash),
        g_(g) {
        calculate_parameters();
    }

  public:
    pac_tree_t(const Problem::problem_t<T> &problem)
      : improvement_t<T>(problem, 0),
        width_(0),
        horizon_(0),
        random_ties_(false),
        delta_(.1),
        epsilon_(.1),
        damping_(.8),
        max_num_samples_(10),
        soft_pruning_(true),
        heuristic_(0),
        algorithm_(0),
        hash_(0),
        g_(.9) {
        gamma_ = problem_.discount();
        total_num_nodes_pruned_ = 0;
        total_num_a_nodes_pruned_ = 0;
        total_num_policy_evaluations_ = 0;
        total_num_heuristic_evaluations_ = 0;
    }
    virtual ~pac_tree_t() { }
    virtual policy_t<T>* clone() const {
        return new pac_tree_t(problem_, base_policy_, width_, horizon_, random_ties_, delta_, epsilon_, damping_, max_num_samples_, soft_pruning_, heuristic_, algorithm_, hash_, g_);
    }
    virtual std::string name() const {
        return std::string("pac-tree(") +
          std::string("width=") + std::to_string(width_) +
          std::string(",horizon=") + std::to_string(horizon_) +
          std::string(",random-ties=") + (random_ties_ ? "true" : "false") +
          std::string(",delta=") + std::to_string(delta_) +
          std::string(",epsilon=") + std::to_string(epsilon_) +
          std::string(",damping=") + std::to_string(damping_) +
          std::string(",max-num-samples=") + std::to_string(max_num_samples_) +
          std::string(",sort-pruning=") + (soft_pruning_ ? "true" : "false") +
          std::string(",policy=") + (base_policy_ == 0 ? std::string("null") : base_policy_->name()) +
          std::string(",heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
          std::string(",algorithm=") + (algorithm_ == 0 ? std::string("null") : algorithm_->name()) +
          std::string(",hash=") + std::to_string((long long unsigned)hash_) +
          std::string(",g=") + std::to_string(g_) +
          std::string(")");
    }

    void calculate_parameters() {
        total_num_nodes_pruned_ = 0;
        total_num_a_nodes_pruned_ = 0;
        total_num_policy_evaluations_ = 0;
        total_num_heuristic_evaluations_ = 0;

        gamma_ = problem_.discount();

        // threshold = epsilon / 2 * gamma^d
#ifdef DEBUG
        std::cout << "debug: pac(): thresholds:";
#endif
        solved_threshold_ = std::vector<float>(1 + horizon_, 0);
        for( int d = 0; d <= horizon_; ++d ) {
            float threshold = epsilon_ / (2 * powf(gamma_, d));
            solved_threshold_[d] = threshold;
#ifdef DEBUG
            std::cout << " " << std::setprecision(4) << threshold;
#endif
        }
#ifdef DEBUG
        std::cout << std::endl;
#endif

        // l(s,d) = 4 gamma^(2d) C_max (d ln b + d ln 2 - ln delta) / (1 - gamma) epsilon^2
#ifdef DEBUG
        std::cout << "debug: pac(): num-samples:";
#endif
        num_samples_ = std::vector<float>(1 + horizon_, 0);
        for( int d = 0; d <= horizon_; ++d ) {
            float lsd = 4 * powf(gamma_ * gamma_, d) * problem_.max_absolute_cost();
#ifdef DEBUG
            std::cout << " [" << lsd;
#endif
            lsd *= d * log(2 * problem_.max_combined_branching()) - log(delta_);
#ifdef DEBUG
            std::cout << " " << lsd;
#endif
            lsd /= (1 - gamma_) * epsilon_ * epsilon_;
#ifdef DEBUG
            std::cout << " " << lsd << "]";
#endif
            num_samples_[d] = std::min<float>(lsd, max_num_samples_);
        }
#ifdef DEBUG
        std::cout << std::endl;
#endif
    }

    void reset_stats_for_single_action_selection() const {
        num_nodes_pruned_ = 0;
        num_a_nodes_pruned_ = 0;
        num_policy_evaluations_ = 0;
        num_heuristic_evaluations_ = 0;
    }

    virtual Problem::action_t operator()(const T &s) const {
        std::priority_queue<state_node_t<T>*, std::vector<state_node_t<T>*>, state_node_comparator_t<T> > heap, heap_aux;

        ++policy_t<T>::decisions_;

        reset_stats_for_single_action_selection();
        state_node_t<T> *root = new state_node_t<T>(s, 0, 0);
        compute_bounds(*root);

        Problem::action_t action = 0;
        if( false && solved(*root) ) { // CHECK
#ifdef DEBUG
            std::cout << "debug: pac():"
                      << " root=" << *root
                      << " lb=" << root->lower_bound_
                      << " ub=" << root->upper_bound_
                      << " gap=" << root->gap()
                      << " threshold=" << solved_threshold_[root->depth_]
                      << std::endl;
#endif
            delete root;
            action = (*base_policy_)(s);
#ifdef DEBUG
            std::cout << "debug: pac(): selection:"
                      << " action=" << action
                      << " method=base-policy"
                      << std::endl;
#endif
        } else {
            compute_score(*root);
            heap.push(root);
            int last_heap_size = heap.size();

            leaf_nodes_.clear();
            for( int i = 0; !heap.empty() && (i < width_); ++i ) {
                state_node_t<T> &node = *heap.top();
                heap.pop();
#ifdef DEBUG
                std::cout << "debug: pac():"
                          << " pop=" << node
                          << " gap=" << node.gap()
                          << " score=" << node.score_
                          << " gap-at-root=" << root->gap()
                          << std::endl;
#endif

                // node must be active leaf
                if( node.pruned_ ) continue; // node can become pruned after it is inserted in heap
                assert(!solved(node));
                assert(node.children_.empty());

                // expand leaf node
                expand(node, leaf_nodes_);
                propagate_values(node);

                if( heap.empty() ) {
#ifdef DEBUG
                    std::cout << "debug: pac(): #leaves=" << leaf_nodes_.size() << std::endl;
#endif
                    int nsolved = 0, npruned = 0, nonleaf = 0;

                    // compute scores for all leaves
                    int n_leaf_nodes = leaf_nodes_.size();
                    for( int i = 0; i < int(leaf_nodes_.size()); ++i ) {
                        state_node_t<T> *node = leaf_nodes_[i];
                        bool is_leaf = node->children_.empty();
                        bool is_solved = solved(*node);
                        bool is_pruned = node->pruned_;
                        if( !is_solved && !is_pruned && is_leaf ) {
                            compute_score(*node);
                        } else {
                            if( is_leaf ) {
                                if( is_solved ) ++nsolved;
                                if( !is_solved && is_pruned ) ++npruned;
                            } else {
                                ++nonleaf;
                            }
                            leaf_nodes_[i--] = leaf_nodes_.back();
                            leaf_nodes_.pop_back();
                        }
                    }
                    assert(n_leaf_nodes == leaf_nodes_.size() + nsolved + npruned + nonleaf);

                    for( int i = 0; i < int(leaf_nodes_.size()); ++i ) {
                        state_node_t<T> *node = leaf_nodes_[i];
                        bool is_leaf = node->children_.empty();
                        bool is_solved = solved(*node);
                        bool is_pruned = node->pruned_;
                        assert(is_leaf && !is_solved && !is_pruned);
                    }

                    // insert all leaves in auxiliary heap and select best ones for new heap
                    build_max_heap(leaf_nodes_);
                    int min = std::min<int>(leaf_nodes_.size(), 2 * last_heap_size);
                    heap = std::priority_queue<state_node_t<T>*, std::vector<state_node_t<T>*>, state_node_comparator_t<T> >(&leaf_nodes_[0], &leaf_nodes_[min]);
                    last_heap_size = heap.size();

                    if( (min > 0) && (min - 1 < leaf_nodes_.size()) ) {
                        for( int i = min - 1; i >= 0; --i ) {
                            leaf_nodes_[i] = leaf_nodes_.back();
                            leaf_nodes_.pop_back();
                        }
                    }

#ifdef DEBUG
                    std::cout << "debug: pac():"
                              << " #selected=" << heap.size()
                              << " #remaining=" << leaf_nodes_.size()
                              << " #solved=" << nsolved
                              << " #pruned=" << npruned
                              << " #non-leaf=" << nonleaf
                              << std::endl;
#endif
                    assert(n_leaf_nodes == heap.size() + leaf_nodes_.size() + nsolved + npruned + nonleaf);
                }
            }
#ifdef DEBUG
            std::cout << "debug: pac(): main loop ended:"
                      << " heap-sz=" << heap.size()
                      << " #leaves=" << leaf_nodes_.size()
                      << std::endl;
#endif

#ifdef DEBUG
            std::cout << "debug: pac():"
                      << " root=" << *root
                      << " lb=" << root->lower_bound_
                      << " ub=" << root->upper_bound_
                      << " gap=" << root->gap()
                      << " threshold=" << solved_threshold_[root->depth_]
                      << std::endl;
#endif
            action = root->select_action(random_ties_);
            delete_tree(root);
#ifdef DEBUG
            std::cout << "debug: pac(): selection:"
                      << " action=" << action
                      << " method=tree"
                      << std::endl;
#endif
        }
#ifdef DEBUG
        std::cout << Utils::magenta()
                  << "debug: pac(): stats-single-call:"
                  << " #nodes-pruned=" << num_nodes_pruned_
                  << " #a-nodes-pruned=" << num_a_nodes_pruned_
                  << " #policy-evaluations=" << num_policy_evaluations_
                  << " #heuristic-evaluations=" << num_heuristic_evaluations_
                  << Utils::normal()
                  << std::endl;
        std::cout << std::endl;
#endif
        assert(problem_.applicable(s, action));
        return action;
    }
    virtual void reset_stats() const {
        problem_.clear_expansions();
        if( base_policy_ != 0 ) base_policy_->reset_stats();
        if( heuristic_ != 0 ) heuristic_->reset_stats();
        //if( algorithm_ != 0 ) algorithm_->reset_stats(*hash_);
    }
    virtual void print_other_stats(std::ostream &os, int indent) const {
        os << std::setw(indent) << ""
           << "other-stats: name=" << name()
           << " decisions=" << policy_t<T>::decisions_
           << " num-nodes-pruned=" << total_num_nodes_pruned_
           << " num-a-nodes-pruned=" << total_num_a_nodes_pruned_
           << " num-policy-evaluations=" << total_num_policy_evaluations_
           << " num-heuristic-evaluations=" << total_num_heuristic_evaluations_
           << std::endl;
        if( base_policy_ != 0 ) base_policy_->print_other_stats(os, 2 + indent);
    }
    virtual void set_parameters(const std::multimap<std::string, std::string> &parameters, Dispatcher::dispatcher_t<T> &dispatcher) {
        std::multimap<std::string, std::string>::const_iterator it = parameters.find("width");
        if( it != parameters.end() ) width_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("horizon");
        if( it != parameters.end() ) horizon_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("random-ties");
        if( it != parameters.end() ) random_ties_ = it->second == "true";
        it = parameters.find("delta");
        if( it != parameters.end() ) delta_ = strtod(it->second.c_str(), 0);
        it = parameters.find("epsilon");
        if( it != parameters.end() ) epsilon_ = strtod(it->second.c_str(), 0);
        it = parameters.find("damping");
        if( it != parameters.end() ) damping_ = strtod(it->second.c_str(), 0);
        it = parameters.find("max-num-samples");
        if( it != parameters.end() ) max_num_samples_ = strtol(it->second.c_str(), 0, 0);
        it = parameters.find("soft-pruning");
        if( it != parameters.end() ) soft_pruning_ = it->second == "true";

        it = parameters.find("policy");
        if( it != parameters.end() ) {
            delete base_policy_;
            dispatcher.create_request(problem_, it->first, it->second);
            base_policy_ = dispatcher.fetch_policy(it->second);
        }
        it = parameters.find("heuristic");
        if( it != parameters.end() ) {
            delete heuristic_;
            dispatcher.create_request(problem_, it->first, it->second);
            heuristic_ = dispatcher.fetch_heuristic(it->second);
        }

        it = parameters.find("algorithm");
        if( it != parameters.end() ) {
            delete algorithm_;
            dispatcher.create_request(problem_, it->first, it->second);
            algorithm_ = dispatcher.fetch_algorithm(it->second);
        }
        it = parameters.find("g");
        if( it != parameters.end() ) g_ = strtod(it->second.c_str(), 0);

        calculate_parameters();
        if( algorithm_ != 0 ) solve_problem();
#ifdef DEBUG
        std::cout << "debug: pac(): params:"
                  << " width=" << width_
                  << " horizon=" << horizon_
                  << " random-ties=" << random_ties_
                  << " delta=" << delta_
                  << " epsilon=" << epsilon_
                  << " damping=" << damping_
                  << " max-num-samples=" << max_num_samples_
                  << " soft-pruning=" << soft_pruning_
                  << " policy=" << (base_policy_ == 0 ? std::string("null") : base_policy_->name())
                  << " heuristic=" << (heuristic_ == 0 ? std::string("null") : heuristic_->name())
                  << " algorithm=" << (algorithm_ == 0 ? std::string("null") : algorithm_->name())
                  << " hash=" << hash_
                  << " g=" << g_
                  << std::endl;
#endif
    }

    void compute_bounds(state_node_t<T> &node) const {
        float value = 0;
        if( (algorithm_ != 0) && ((heuristic_ == 0) || (base_policy_ == 0)) ) {
            assert(hash_ != 0);
            value = hash_->value(*node.state_);
        }

        assert(node.lower_bound_ == 0);
        if( heuristic_ != 0 ) {
            ++num_heuristic_evaluations_;
            ++total_num_heuristic_evaluations_;
            node.lower_bound_ = heuristic_->value(*node.state_); // XXXX: heuristic should use (s, depth)
        } else {
            assert(algorithm_ != 0);
            node.lower_bound_ = (1 - powf(g_, 2 * node.depth_)) * value;
        }

        assert(node.upper_bound_ == 0);
        if( base_policy_ != 0 ) {
            node.upper_bound_ = evaluate(node);
        } else {
            assert(algorithm_ != 0);
            node.upper_bound_ = (1 + powf(g_, 2 * node.depth_)) * value;
        }
    }

    bool solved(const state_node_t<T> &node) const {
        return node.gap() < solved_threshold_[node.depth_]; 
    }

    // assuming the upper bound decreases by gap / 4, compute sum of gap reductions on ancestor nodes
    void compute_score(const state_node_t<T> &node) const {
        // XXXX: NEED TO ESTIMATE PRUNED BRANCHES
        if( node.parent_ != 0 ) {
            node.lower_bound_est_ = (node.lower_bound_ + (node.lower_bound_ + node.upper_bound_) / 2) / 2;
            node.upper_bound_est_ = ((node.lower_bound_ + node.upper_bound_) / 2 + node.upper_bound_) / 2;

            update_bounds_est(static_cast<const action_node_t<T>*>(node.parent_), &node);

            float score = 0;
            float damping = powf(damping_, node.depth_ - 1);
            for( const node_t<T> *n = node.parent_; n != 0; n = n->parent_ ) {
               n = n->parent_;
               assert(dynamic_cast<const state_node_t<T>*>(n) != 0);
               float gap_reduction = n->gap() - n->gap_est();
               //assert(gap_reduction >= 0);
               //score = std::max(score, gap_reduction * damping);
               //damping /= damping_;
               score += gap_reduction * damping;
               damping /= damping_;
            }
            node.score_ = score;
        }
    }

    void expand(state_node_t<T> &node, std::vector<state_node_t<T>*> &leaf_nodes) const {
        assert(node.children_.empty());
        int nactions = problem_.number_actions(*node.state_);
        std::vector<node_t<T>*> children;
        children.reserve(nactions);
        for( Problem::action_t a = 0; a < nactions; ++a ) {
            if( problem_.applicable(*node.state_, a) ) {
                action_node_t<T> *a_node = new action_node_t<T>(*node.state_, node.depth_, a, &node);
                std::vector<std::pair<T, float> > outcomes;
                problem_.next(*node.state_, a, outcomes);

                a_node->prob_ = std::vector<float>(outcomes.size(), 0);
                a_node->children_ = std::vector<node_t<T>*>(outcomes.size(), 0);

                for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
                    const T &state = outcomes[i].first;
                    float prob = outcomes[i].second;

                    state_node_t<T> *s_node = new state_node_t<T>(state, 1 + node.depth_, a_node);
                    compute_bounds(*s_node);
                    leaf_nodes.push_back(s_node);

                    a_node->children_[i] = s_node;
                    a_node->prob_[i] = prob;
                }

                update_bounds(*a_node);
                children.push_back(a_node);
            }
        }
        node.children_ = std::move(children);
        // we don't update bounds here. It is done in propagate_values()
    }

    void update_bounds(state_node_t<T> &node) const {
        assert(!node.children_.empty());
        node.upper_bound_ = node.children_[0]->upper_bound_;
        node.best_child_ = 0;
        for( int i = 1; i < int(node.children_.size()); ++i ) {
            const action_node_t<T> &a_node = *static_cast<const action_node_t<T>*>(node.children_[i]);
            if( node.upper_bound_ > a_node.upper_bound_ ) {
                node.upper_bound_ = a_node.upper_bound_;
                node.best_child_ = i;
            }
        }
        node.lower_bound_ = node.children_[node.best_child_]->lower_bound_;

        // mark nodes (subtrees) as pruned
        float eta = 0.1; // CHECK
        for( int i = 0; i < int(node.children_.size()); ++i ) {
            action_node_t<T> &a_node = *static_cast<action_node_t<T>*>(node.children_[i]);
            if( (node.best_child_ == i) || a_node.pruned_ ) continue;
            if( node.upper_bound_ < a_node.lower_bound_ ) { // hard pruning
#ifdef DEBUG2
                std::cout << "debug: pac(): HARD pruning: a-node=" << a_node << std::endl;
#endif
                mark_node_as_pruned(a_node, true);
            } else if( soft_pruning_ && (node.upper_bound_ < a_node.lower_bound_ + eta) && (node.upper_bound_ <= a_node.upper_bound_) ) { // soft pruning
#ifdef DEBUG
                std::cout << "debug: pac(): SOFT pruning: a-node=" << a_node << std::endl;
#endif
                mark_node_as_pruned(a_node, false);
            }
        }
    }
    void update_bounds(action_node_t<T> &a_node) const {
        assert(!a_node.children_.empty());
        assert(a_node.children_.size() == a_node.prob_.size());
        a_node.lower_bound_ = 0;
        a_node.upper_bound_ = 0;
        for( int i = 0; i < int(a_node.children_.size()); ++i ) {
            const node_t<T> &node = *a_node.children_[i];
            float prob = a_node.prob_[i];
            a_node.lower_bound_ += prob * node.lower_bound_;
            a_node.upper_bound_ += prob * node.upper_bound_;
        }
        a_node.lower_bound_ = problem_.cost(*a_node.state_, a_node.action_) + gamma_ * a_node.lower_bound_;
        a_node.upper_bound_ = problem_.cost(*a_node.state_, a_node.action_) + gamma_ * a_node.upper_bound_;
    }

    void update_bounds_est(const state_node_t<T> *node, const action_node_t<T> *child) const {
        assert(node != 0);
        assert(!node->children_.empty());
        node->lower_bound_est_ = node->children_[0] == child ? node->children_[0]->lower_bound_est_ : node->children_[0]->lower_bound_;
        node->upper_bound_est_ = node->children_[0] == child ? node->children_[0]->upper_bound_est_ : node->children_[0]->upper_bound_;
        for( int i = 1; i < int(node->children_.size()); ++i ) {
            const action_node_t<T> *a_node = static_cast<const action_node_t<T>*>(node->children_[i]);
            node->lower_bound_est_ = std::min(node->lower_bound_est_, a_node == child ? a_node->lower_bound_est_ : a_node->lower_bound_);
            node->upper_bound_est_ = std::min(node->upper_bound_est_, a_node == child ? a_node->upper_bound_est_ : a_node->upper_bound_);
        }

        if( node->parent_ != 0 ) {
            assert(dynamic_cast<const action_node_t<T>*>(node->parent_) != 0);
            update_bounds_est(static_cast<const action_node_t<T>*>(node->parent_), node);
        }
    }
    void update_bounds_est(const action_node_t<T> *a_node, const state_node_t<T> *child) const {
        assert(a_node != 0);
        assert(!a_node->children_.empty());
        assert(a_node->children_.size() == a_node->prob_.size());
        a_node->lower_bound_est_ = 0;
        a_node->upper_bound_est_ = 0;
        for( int i = 0; i < int(a_node->children_.size()); ++i ) {
            const node_t<T> *node = a_node->children_[i];
            float prob = a_node->prob_[i];
            a_node->lower_bound_est_ += prob * (node == child ? node->lower_bound_est_ : node->lower_bound_);
            a_node->upper_bound_est_ += prob * (node == child ? node->upper_bound_est_ : node->upper_bound_);
        }
        a_node->lower_bound_est_ = problem_.cost(*a_node->state_, a_node->action_) + gamma_ * a_node->lower_bound_est_;
        a_node->upper_bound_est_ = problem_.cost(*a_node->state_, a_node->action_) + gamma_ * a_node->upper_bound_est_;

        assert(a_node->parent_ != 0);
        assert(dynamic_cast<const state_node_t<T>*>(a_node->parent_) != 0);
        update_bounds_est(static_cast<const state_node_t<T>*>(a_node->parent_), a_node);
    }

    void mark_node_as_pruned(state_node_t<T> &node, bool hard) const {
#ifdef DEBUG2
        std::cout << "debug: pac(): marking node as pruned:"
                  << " hard=" << hard
                  << " node=" << node;
        if( !hard ) {
            std::cout << " gap=" << node.gap();
        }
        std::cout << std::endl;
#endif
        for( int i = 0; i < int(node.children_.size()); ++i ) {
            action_node_t<T> &a_node = *static_cast<action_node_t<T>*>(node.children_[i]);
            if( !a_node.pruned_ ) mark_node_as_pruned(a_node, hard);
        }
        node.pruned_ = true;
        ++num_nodes_pruned_;
        ++total_num_nodes_pruned_;
    }
    void mark_node_as_pruned(action_node_t<T> &a_node, bool hard) const {
#ifdef DEBUG2
        std::cout << "debug: pac(): marking node as pruned:"
                  << " hard=" << hard
                  << " a-node=" << a_node;
        if( !hard ) {
            std::cout << " gap=" << a_node.gap();
        }
        std::cout << std::endl;
#endif
        for( int i = 0; i < int(a_node.children_.size()); ++i ) {
            state_node_t<T> &node = *static_cast<state_node_t<T>*>(a_node.children_[i]);
            if( !node.pruned_ ) mark_node_as_pruned(node, hard);
        }
        a_node.pruned_ = true;
        ++num_a_nodes_pruned_;
        ++total_num_a_nodes_pruned_;
    }

    void propagate_values(state_node_t<T> &node) const {
        update_bounds(node);
        if( node.parent_ != 0 ) {
            assert(dynamic_cast<action_node_t<T>*>(node.parent_) != 0);
            propagate_values(*static_cast<action_node_t<T>*>(node.parent_));
        }
    }
    void propagate_values(action_node_t<T> &a_node) const {
        update_bounds(a_node);
        if( a_node.parent_ != 0 ) {
            assert(dynamic_cast<state_node_t<T>*>(a_node.parent_) != 0);
            propagate_values(*static_cast<state_node_t<T>*>(a_node.parent_));
        }
    }

    float evaluate(const T &s, unsigned num_samples, unsigned depth) const {
        ++num_policy_evaluations_;
        ++total_num_policy_evaluations_;
        return Evaluation::evaluation(*base_policy_, s, num_samples, horizon_ - depth);
    }
    float evaluate(const state_node_t<T> &node) const {
        return evaluate(*node.state_, num_samples_[node.depth_], node.depth_);
    }

    void delete_tree(state_node_t<T> *node) const {
        for( int i = 0; i < int(node->children_.size()); ++i ) {
            action_node_t<T> *a_node = static_cast<action_node_t<T>*>(node->children_[i]);
            delete_tree(a_node);
        }
        delete node;
    }
    void delete_tree(action_node_t<T> *a_node) const {
        for( int i = 0; i < int(a_node->children_.size()); ++i ) {
            state_node_t<T> *node = static_cast<state_node_t<T>*>(a_node->children_[i]);
            delete_tree(node);
        }
        delete a_node;
    }

    void build_max_heap(std::vector<state_node_t<T>*> &heap) const {
        for( int i = heap.size() / 2; i >= 1; --i )
            max_heapify(heap, i - 1);
        assert(is_max_heap(heap));
    }
    void max_heapify(std::vector<state_node_t<T>*> &heap, int i) const {
        int right = (i + 1) << 1;
        int left = right - 1;
        int largest = -1;
        if( (left < heap.size()) && (heap[left]->score_ > heap[i]->score_) )
            largest = left;
        else
            largest = i;
        if( (right < heap.size()) && (heap[right]->score_ > heap[largest]->score_) )
            largest = right;
        if( largest != i ) {
            state_node_t<T> *node = heap[i];
            heap[i] = heap[largest];
            heap[largest] = node;
            max_heapify(heap, largest);
        }
    }
    bool is_max_heap(const std::vector<state_node_t<T>*> &heap) const {
        for( int i = 0; i < int(heap.size()); ++i ) {
            int right = (i + 1) << 1;
            int left = right - 1;
            if( (left < heap.size()) && (heap[i]->score_ < heap[left]->score_) ) return false;
            if( (right < heap.size()) && (heap[i]->score_ < heap[right]->score_) ) return false;
        }
        return true;
    }

    void solve_problem() {
        if( algorithm_ == 0 ) {
            std::cout << Utils::error() << "algorithm must be specified for solving problem!" << std::endl;
            exit(1);
        }
#ifdef DEBUG
        std::cout << "debug: pac-tree(): solving problem with algorithm=" << algorithm_->name() << std::endl;
#endif
        delete hash_;
        hash_ = new Problem::hash_t<T>(problem_);
        algorithm_->solve(problem_.init(), *hash_);
    }
};

}; // namespace PAC

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

