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

#ifndef PAC2_H
#define PAC2_H

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

namespace PAC2 {

////////////////////////////////////////////////
//
// Nodes
//

template<typename T> struct node_t {
    // bounding and pruning
    float lower_bound_;
    float upper_bound_;
    bool pruned_;

    // used for computing scores
    mutable float lower_bound_guessed_;
    mutable float upper_bound_guessed_;

    node_t()
      : lower_bound_(0),
        upper_bound_(0),
        pruned_(false),
        lower_bound_guessed_(0),
        upper_bound_guessed_(0) {
    }
    virtual ~node_t() { }

    float gap() const { return upper_bound_ - lower_bound_; }
    float gap_guessed() const { return upper_bound_guessed_ - lower_bound_guessed_; }
    virtual void print(std::ostream &os) const = 0;
};

template<typename T> struct state_node_t;
template<typename T> struct action_node_t : public node_t<T> {
    Problem::action_t action_;
    float action_cost_;

    state_node_t<T> *parent_;
    std::vector<std::pair<float, state_node_t<T>*> > children_;

    action_node_t(Problem::action_t action, float action_cost, state_node_t<T> *parent)
      : action_(action),
        action_cost_(action_cost),
        parent_(parent) {
    }
    //action_node_t(Problem::action_t action) : action_(action) { }
    virtual ~action_node_t() { }

    virtual void print(std::ostream &os) const {
        os << "[s=" << parent_->state_
           << ",d=" << parent_->depth_
           << ",a=" << action_
           << "]";
    }
};

template<typename T> struct state_node_t : public node_t<T> {
    const T state_;
    bool is_goal_;
    bool is_dead_end_;
    int depth_;

    std::vector<std::pair<int, action_node_t<T>*> > parents_;
    std::vector<action_node_t<T>*> children_;

    mutable int best_child_;
    mutable float score_;

    state_node_t(const T &state, int depth = 0)
      : state_(state),
        is_goal_(false),
        is_dead_end_(false),
        depth_(depth),
        best_child_(-1),
        score_(0) {
    }
    virtual ~state_node_t() { }

    Problem::action_t select_action(bool random_ties, bool use_upper_bound = true) const {
        assert(!children_.empty());
        float best_value = use_upper_bound ? children_[0]->upper_bound_ : children_[0]->lower_bound_;
        std::vector<Problem::action_t> best_actions;
        best_actions.reserve(random_ties ? children_.size() : 1);
        best_actions.push_back(children_[0]->action_);

        for( int i = 1; i < int(children_.size()); ++i ) {
            const action_node_t<T> *a_node = children_[i];
            if( (use_upper_bound && (a_node->upper_bound_ <= best_value)) || (!use_upper_bound && (a_node->lower_bound_ <= best_value)) ) {
                if( (use_upper_bound && (a_node->upper_bound_ < best_value)) || (!use_upper_bound && (a_node->lower_bound_ < best_value)) )
                    best_actions.clear();
                best_value = use_upper_bound ? a_node->upper_bound_ : a_node->lower_bound_;
                best_actions.push_back(a_node->action_);
            }
        }

        assert(!best_actions.empty());
        return best_actions[Random::random(best_actions.size())];
    }

    virtual void print(std::ostream &os) const {
        os << "[s=" << state_
           << ",d=" << depth_
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
    float score_damping_;
    unsigned max_num_samples_;
    bool soft_pruning_;

    const Heuristic::heuristic_t<T> *heuristic_;

    const Algorithm::algorithm_t<T> *algorithm_; // used for setting LBs and UBs
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
    mutable int num_nodes_pruned_soft_;
    mutable int num_a_nodes_pruned_soft_;
    mutable int num_policy_evaluations_;
    mutable int num_heuristic_evaluations_;

    // overall stats
    mutable int total_num_nodes_pruned_;
    mutable int total_num_a_nodes_pruned_;
    mutable int total_num_nodes_pruned_soft_;
    mutable int total_num_a_nodes_pruned_soft_;
    mutable int total_num_policy_evaluations_;
    mutable int total_num_heuristic_evaluations_;

    pac_tree_t(const Problem::problem_t<T> &problem,
               const policy_t<T> *base_policy,
               unsigned width,
               unsigned horizon,
               bool random_ties,
               float delta,
               float epsilon,
               float score_damping,
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
        score_damping_(score_damping),
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
        score_damping_(.8),
        max_num_samples_(10),
        soft_pruning_(true),
        heuristic_(0),
        algorithm_(0),
        hash_(0),
        g_(.9) {
        gamma_ = problem_.discount();
        total_num_nodes_pruned_ = 0;
        total_num_a_nodes_pruned_ = 0;
        total_num_nodes_pruned_soft_ = 0;
        total_num_a_nodes_pruned_soft_ = 0;
        total_num_policy_evaluations_ = 0;
        total_num_heuristic_evaluations_ = 0;
        std::cout << Utils::red() << "warning: pac2-tree() is research in progress. Don't use it without permission" << Utils::normal() << std::endl;
    }
    virtual ~pac_tree_t() { }
    virtual policy_t<T>* clone() const {
        return new pac_tree_t(problem_, base_policy_, width_, horizon_, random_ties_, delta_, epsilon_, score_damping_, max_num_samples_, soft_pruning_, heuristic_, algorithm_, hash_, g_);
    }
    virtual std::string name() const {
        return std::string("pac-tree(") +
          std::string("width=") + std::to_string(width_) +
          std::string(",horizon=") + std::to_string(horizon_) +
          std::string(",random-ties=") + (random_ties_ ? "true" : "false") +
          std::string(",delta=") + std::to_string(delta_) +
          std::string(",epsilon=") + std::to_string(epsilon_) +
          std::string(",score-damping=") + std::to_string(score_damping_) +
          std::string(",max-num-samples=") + std::to_string(max_num_samples_) +
          std::string(",soft-pruning=") + (soft_pruning_ ? "true" : "false") +
          std::string(",policy=") + (base_policy_ == 0 ? std::string("null") : base_policy_->name()) +
          std::string(",heuristic=") + (heuristic_ == 0 ? std::string("null") : heuristic_->name()) +
          std::string(",algorithm=") + (algorithm_ == 0 ? std::string("null") : algorithm_->name()) +
          std::string(",g=") + std::to_string(g_) +
          std::string(")");
    }

    void calculate_parameters() {
        total_num_nodes_pruned_ = 0;
        total_num_a_nodes_pruned_ = 0;
        total_num_nodes_pruned_soft_ = 0;
        total_num_a_nodes_pruned_soft_ = 0;
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
        num_nodes_pruned_soft_ = 0;
        num_a_nodes_pruned_soft_ = 0;
        num_policy_evaluations_ = 0;
        num_heuristic_evaluations_ = 0;
    }

    virtual Problem::action_t operator()(const T &s) const {
        std::priority_queue<state_node_t<T>*, std::vector<state_node_t<T>*>, state_node_comparator_t<T> > heap, heap_aux;

        ++policy_t<T>::decisions_;

        reset_stats_for_single_action_selection();
        state_node_t<T> *root = new state_node_t<T>(s, 0);
        compute_bounds(*root);

        bool pac_tree = true;

        Problem::action_t action = 0;
        if( solved(*root) ) {
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
            if( base_policy_ != 0 ) {
                action = (*base_policy_)(s);
            } else {
                assert(algorithm_ != 0);
                assert(hash_ != 0);
                std::pair<Problem::action_t, float> p = hash_->best_q_value(s);
                action = p.first;
            }
#ifdef DEBUG
            std::cout << "debug: pac(): selection:"
                      << " action=" << action
                      << " method=base-policy"
                      << std::endl;
#endif
        } else {
            compute_score(*root, pac_tree);
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
                assert(node.children_.empty()); // node must be a leaf node

                // expand leaf node
                expand(node, leaf_nodes_, pac_tree);
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
                        if( !is_solved && !node->pruned_ && is_leaf ) {
                            compute_score(*node, pac_tree);
                        } else {
                            if( is_leaf ) {
                                if( is_solved ) ++nsolved;
                                if( !is_solved && node->pruned_ ) ++npruned;
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
                        assert(is_leaf && !is_solved && !node->pruned_);
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
                  << " #nodes-pruned-soft=" << num_nodes_pruned_soft_
                  << " #a-nodes-pruned-soft=" << num_a_nodes_pruned_soft_
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
        policy_t<T>::setup_time_ = 0;
        policy_t<T>::base_policy_time_ = 0;
        policy_t<T>::heuristic_time_ = 0;
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
           << " num-nodes-pruned-soft=" << total_num_nodes_pruned_soft_
           << " num-a-nodes-pruned-soft=" << total_num_a_nodes_pruned_soft_
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
        it = parameters.find("score-damping");
        if( it != parameters.end() ) score_damping_ = strtod(it->second.c_str(), 0);
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

        // setup time for solve_problem() is accounted inside function
        policy_t<T>::setup_time_ += base_policy_ == 0 ? 0 : base_policy_->setup_time();
        policy_t<T>::setup_time_ += heuristic_ == 0 ? 0 : heuristic_->setup_time();

#ifdef DEBUG
        std::cout << "debug: pac(): params:"
                  << " width=" << width_
                  << " horizon=" << horizon_
                  << " random-ties=" << random_ties_
                  << " delta=" << delta_
                  << " epsilon=" << epsilon_
                  << " score-damping=" << score_damping_
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
    virtual typename policy_t<T>::usage_t uses_base_policy() const { return policy_t<T>::usage_t::Optional; }
    virtual typename policy_t<T>::usage_t uses_heuristic() const { return policy_t<T>::usage_t::Optional; }
    virtual typename policy_t<T>::usage_t uses_algorithm() const { return policy_t<T>::usage_t::Optional; }

    void compute_bounds(state_node_t<T> &node) const {
        float value = 0, gamma = 0;
        float lower_bound = 0, upper_bound = 0;
        if( (algorithm_ != 0) && ((heuristic_ == 0) || (base_policy_ == 0)) ) {
            assert(hash_ != 0);
            value = hash_->value(node.state_);
            lower_bound = (1 - powf(g_, node.depth_)) * value;
            upper_bound = (1 + powf(g_, node.depth_)) * value;
        } else if( (algorithm_ == 0) && ((heuristic_ == 0) || (base_policy_ == 0)) ) {
            gamma = (1 - powf(problem_.discount(), horizon_ - node.depth_ + 1)) / (1 - problem_.discount());
        }

        // lower bound
        assert(node.lower_bound_ == 0);
        if( heuristic_ != 0 ) {
            ++num_heuristic_evaluations_;
            ++total_num_heuristic_evaluations_;
            node.lower_bound_ = heuristic_->value(node.state_); // CHECK: heuristic should use (s, depth)
            policy_t<T>::heuristic_time_ = heuristic_->eval_time();
        } else if( algorithm_ != 0 ) {
            //node.lower_bound_ = (1 - powf(g_, 2 * node.depth_)) * value; // CHECK: old lower bound
            node.lower_bound_ = Random::uniform(lower_bound, value);
        } else {
            node.lower_bound_ = gamma * problem_.min_absolute_cost();
        }

        // upper bound
        assert(node.upper_bound_ == 0);
        if( base_policy_ != 0 ) {
            float start_time = Utils::read_time_in_seconds();
            node.upper_bound_ = evaluate(node);
            policy_t<T>::base_policy_time_ += Utils::read_time_in_seconds() - start_time;
        } else if( algorithm_ != 0 ) {
            //node.upper_bound_ = (1 + powf(g_, 2 * node.depth_)) * value; // CHECK: old upper bound
            node.upper_bound_ = Random::uniform(value, upper_bound);
        } else {
            node.upper_bound_ = gamma * problem_.max_absolute_cost();
        }
    }

    bool solved(const state_node_t<T> &node) const {
        return node.is_goal_ || node.is_dead_end_ || node.gap() < solved_threshold_[node.depth_]; 
    }

    // assuming the upper bound decreases by gap / 4, compute sum of gap reductions on ancestor nodes
    void compute_score(const state_node_t<T> &node, bool is_tree) const {
        // CHECK: NEED TO ESTIMATE PRUNED BRANCHES
        if( !node.parents_.empty() ) {
            // assume that we can improve the lower and upper bounds on node when expanded
            node.lower_bound_guessed_ = (node.lower_bound_ + (node.lower_bound_ + node.upper_bound_) / 2) / 2;
            node.upper_bound_guessed_ = ((node.lower_bound_ + node.upper_bound_) / 2 + node.upper_bound_) / 2;

            // propagate this assumption on ancestors
            std::set<const node_t<T>*> affected;
            if( !is_tree ) affected.insert(&node);
            assert(!is_tree || (node.parents_.size() == 1));
            for( int i = 0; i < int(node.parents_.size()); ++i ) {
                if( is_tree )
                    propagate_guessed_bounds_upward(node.parents_[i].second, &node);
                else
                    propagate_guessed_bounds_upward(node.parents_[i].second, affected);
            }

            // summarize the impact of the assumption in the score
            float damping = powf(score_damping_, node.depth_ - 1);
            const state_node_t<T> *n = &node;
            const action_node_t<T> *a_n = 0;
            float score = 0;
            if( is_tree ) {
                assert(n->parents_.size() == 1);
                a_n = n->parents_[0].second;
                while( a_n != 0 ) {
                    n = a_n->parent_;
                    float gap_reduction = n->gap() - n->gap_guessed();
                    //assert(gap_reduction >= 0);
                    //score = std::max(score, gap_reduction * damping);
                    //damping /= score_damping_;
                    score += gap_reduction * damping;
                    damping /= score_damping_;
                    assert(n->parents_.size() <= 1);
                    a_n = n->parents_.empty() ? 0 : n->parents_[0].second;
                }
            } else {
                assert(0); // XXX: need to do this for non-trees
            }
            node.score_ = score;
        }
    }

    std::pair<state_node_t<T>*, bool> fetch_node_in_hash(const T &state, int depth, bool debug) const {
        typename hash_t<T>::iterator it = table_.find(std::make_pair(&state, depth));
        if( it == table_.end() ) {
            if( debug ) std::cout << "fetch_node: node was NOT-FOUND" << std::endl;
            ++num_nodes_;
            state_node_t<T> *node = new state_node_t<T>(state, depth);
            table_.insert(std::make_pair(std::make_pair(&node->state_, depth), node));
            if( problem_.terminal(state) ) {
                if( debug ) std::cout << "fetch_node: node is TERMINAL" << std::endl;
                node->value_ = 0;
                node->is_goal_ = true;
            } else if( problem_.dead_end(state) ) {
                if( debug ) std::cout << "fetch_node: node is DEAD-END" << std::endl;
                node->value_ = problem_.dead_end_value();
                node->is_dead_end_ = true;
            } else {
                if( debug ) std::cout << "fetch_node: node is REGULAR" << std::endl;
                node->value_ = evaluate(state, depth);
                node->nsamples_ = leaf_nsamples_;
            }
            return std::make_pair(node, false);
        } else {
            if( debug ) std::cout << "fetch_node: node was FOUND!" << std::endl;

            // if marked as pruned, unmark it and descendants
            if( it->second->pruned_ )
                unmark_node_as_pruned(it->second);
#if 0
            bool re_evaluated = false;
            if( it->second->is_leaf() && !it->second->is_dead_end_ && !it->second->is_goal_ && (heuristic_ == 0) ) {
                // resample: throw other rollouts to get better estimation. Only done when heuristic_ == 0
                float old_val = it->second->value_;
                float new_val = old_val * it->second->nsamples_ + evaluate(state, depth);
                it->second->nsamples_ += leaf_nsamples_;
                it->second->value_ = new_val / it->second->nsamples_;
                re_evaluated = true;
            }
#endif
            return std::make_pair(it->second, re_evaluated);
        }
    }
    std::pair<state_node_t<T>*, bool> fetch_node(const T &state, int depth, bool build_tree, bool debug) const {
        if( build_tree ) {
            state_node_t<T> *node = new state_node_t<T>(state, depth);
            return std::make_pair(node, false);
        } else {
            return fetch_node_in_hash(state, depth, debug);
        }
    }

    void expand(state_node_t<T> &node, std::vector<state_node_t<T>*> &leaf_nodes, bool build_tree) const {
        assert(node.children_.empty());
        int nactions = problem_.number_actions(node.state_);
        node.children_.reserve(nactions);
        for( Problem::action_t a = 0; a < nactions; ++a ) {
            if( problem_.applicable(node.state_, a) ) {
                float cost = problem_.cost(node.state_, a);
                action_node_t<T> *a_node = new action_node_t<T>(a, cost, &node);
                expand(*a_node, leaf_nodes, build_tree);
                update_bounds(*a_node);
                node.children_.push_back(a_node);
            }
        }
        // we don't update bounds here. It is done in propagate_values()
    }
    void expand(action_node_t<T> &a_node, std::vector<state_node_t<T>*> &leaf_nodes, bool build_tree) const {
        std::vector<std::pair<T, float> > outcomes;
        const state_node_t<T> &parent = *a_node.parent_;
        problem_.next(parent.state_, a_node.action_, outcomes);
        assert(a_node.children_.empty());
        a_node.children_.reserve(outcomes.size());
        for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
            const T &state = outcomes[i].first;
            float prob = outcomes[i].second;

            std::pair<state_node_t<T>*, bool> p = fetch_node(state, 1 + parent.depth_, build_tree, true);
            compute_bounds(*p.first);
            leaf_nodes.push_back(p.first);

            p.first->parents_.push_back(std::make_pair(i, &a_node));
            a_node.children_.push_back(std::make_pair(prob, p.first));
            assert(!build_tree || (p.first->parents_.size() == 1));
        }
    }

    void update_bounds(state_node_t<T> &node) const {
        assert(!node.children_.empty());
        node.upper_bound_ = node.children_[0]->upper_bound_;
        node.best_child_ = 0;
        for( int i = 1; i < int(node.children_.size()); ++i ) {
            const action_node_t<T> &a_node = *node.children_[i];
            if( node.upper_bound_ > a_node.upper_bound_ ) {
                node.upper_bound_ = a_node.upper_bound_;
                node.best_child_ = i;
            }
        }
        node.lower_bound_ = node.children_[node.best_child_]->lower_bound_;

        // mark nodes (subtrees) as pruned
        float eta = 0.1; // CHECK: eta is the gap parameter
        for( int i = 0; i < int(node.children_.size()); ++i ) {
            action_node_t<T> &a_node = *node.children_[i];
            if( (node.best_child_ == i) || a_node.pruned_ ) continue;
            if( node.upper_bound_ < a_node.lower_bound_ ) { // hard pruning
#ifdef DEBUG2
                std::cout << "debug: pac(): try to HARD prune: a-node=" << a_node << std::endl;
#endif
                try_to_mark_node_as_pruned(a_node, true);
            } else if( soft_pruning_ && (node.upper_bound_ < a_node.lower_bound_ + eta) && (node.upper_bound_ <= a_node.upper_bound_) ) { // soft pruning
#ifdef DEBUG
                std::cout << "debug: pac(): try to SOFT prune: a-node=" << a_node << std::endl;
#endif
                try_to_mark_node_as_pruned(a_node, false);
            }
        }
    }
    void update_bounds(action_node_t<T> &a_node) const {
        assert(!a_node.children_.empty());
        a_node.lower_bound_ = 0;
        a_node.upper_bound_ = 0;
        for( int i = 0; i < int(a_node.children_.size()); ++i ) {
            float prob = a_node.children_[i].first;
            const state_node_t<T> &node = *a_node.children_[i].second;
            a_node.lower_bound_ += prob * node.lower_bound_;
            a_node.upper_bound_ += prob * node.upper_bound_;
        }
        a_node.lower_bound_ = a_node.action_cost_ + gamma_ * a_node.lower_bound_;
        a_node.upper_bound_ = a_node.action_cost_ + gamma_ * a_node.upper_bound_;
    }

    // propagate guessed bounds for trees
    void propagate_guessed_bounds_upward(const state_node_t<T> *node, const action_node_t<T> *child) const {
        assert(node != 0);
        assert(!node->children_.empty());

        // compute guessed bounds
#if 0
        int best_child = 0;
        float best_upper = node->children_[0] == child ? child->upper_bound_guessed_ : node->children_[0]->upper_bound_;
        //float best_gap = node->children_[0] == child ? child->upper_bound_guessed_ : node->children_[0]->upper_bound_;
        //best_gap -= node->children_[0] == child ? child->lower_bound_guessed_ : node->children_[0]->lower_bound_;
        for( int i = 1; i < int(node->children_.size()); ++i ) {
            const action_node_t<T> *a_node = node->children_[i];
            if( a_node == child ) {
                if( child->upper_bound_guessed_ < best_upper ) {
                    best_child = i;
                    best_upper = child->upper_bound_guessed_;
                }
            } else {
                if( child->upper_bound_ < best_upper ) {
                    best_child = i;
                    best_upper = child->upper_bound_;
                }
            }
        }
        const action_node_t<T> *best_a_node = node->children_[best_child];
        node->lower_bound_guessed_ = best_a_node == child ? child->lower_bound_guessed_ : best_a_node->lower_bound_;
        node->upper_bound_guessed_ = best_upper;
#else
        node->lower_bound_guessed_ = node->children_[0] == child ? child->lower_bound_guessed_ : node->children_[0]->lower_bound_;
        node->upper_bound_guessed_ = node->children_[0] == child ? child->upper_bound_guessed_ : node->children_[0]->upper_bound_;
        for( int i = 1; i < int(node->children_.size()); ++i ) {
            const action_node_t<T> *a_node = node->children_[i];
            node->lower_bound_guessed_ = std::min(node->lower_bound_guessed_, a_node == child ? child->lower_bound_guessed_ : a_node->lower_bound_);
            node->upper_bound_guessed_ = std::min(node->upper_bound_guessed_, a_node == child ? child->upper_bound_guessed_ : a_node->upper_bound_);
        }
#endif

        assert(node->parents_.size() <= 1);
        if( !node->parents_.empty() ) propagate_guessed_bounds_upward(node->parents_[0].second, node);
    }
    void propagate_guessed_bounds_upward(const action_node_t<T> *a_node, const state_node_t<T> *child) const {
        assert(a_node != 0);
        assert(!a_node->children_.empty());

        // compute guessed bounds
        a_node->lower_bound_guessed_ = 0;
        a_node->upper_bound_guessed_ = 0;
        for( int i = 0; i < int(a_node->children_.size()); ++i ) {
            float prob = a_node->children_[i].first;
            const node_t<T> *node = a_node->children_[i].second;
            a_node->lower_bound_guessed_ += prob * (node == child ? node->lower_bound_guessed_ : node->lower_bound_);
            a_node->upper_bound_guessed_ += prob * (node == child ? node->upper_bound_guessed_ : node->upper_bound_);
        }
        a_node->lower_bound_guessed_ = a_node->action_cost_ + gamma_ * a_node->lower_bound_guessed_;
        a_node->upper_bound_guessed_ = a_node->action_cost_ + gamma_ * a_node->upper_bound_guessed_;

        // propagate upwards
        assert(a_node->parent_ != 0);
        propagate_guessed_bounds_upward(a_node->parent_, a_node);
    }

    // propagate guessed bounds for non-trees
    void propagate_guessed_bounds_upward(const state_node_t<T> *node, std::set<const node_t<T>*> &affected) const {
        assert(node != 0);
        assert(!node->children_.empty());

        // compute guessed bounds
#if 0
        int best_child = 0;
        float best_upper = node->children_[0] == child ? child->upper_bound_guessed_ : node->children_[0]->upper_bound_;
        //float best_gap = node->children_[0] == child ? child->upper_bound_guessed_ : node->children_[0]->upper_bound_;
        //best_gap -= node->children_[0] == child ? child->lower_bound_guessed_ : node->children_[0]->lower_bound_;
        for( int i = 1; i < int(node->children_.size()); ++i ) {
            const action_node_t<T> *a_node = node->children_[i];
            if( a_node == child ) {
                if( child->upper_bound_guessed_ < best_upper ) {
                    best_child = i;
                    best_upper = child->upper_bound_guessed_;
                }
            } else {
                if( child->upper_bound_ < best_upper ) {
                    best_child = i;
                    best_upper = child->upper_bound_;
                }
            }
        }
        const action_node_t<T> *best_a_node = node->children_[best_child];
        node->lower_bound_guessed_ = best_a_node == child ? child->lower_bound_guessed_ : best_a_node->lower_bound_;
        node->upper_bound_guessed_ = best_upper;
#else
        if( affected.find(node->children_[0]) != affected.end() ) {
            node->lower_bound_guessed_ = node->children_[0]->lower_bound_guessed_;
            node->upper_bound_guessed_ = node->children_[0]->upper_bound_guessed_;
        } else {
            node->lower_bound_guessed_ = node->children_[0]->lower_bound_;
            node->upper_bound_guessed_ = node->children_[0]->upper_bound_;
        }
        for( int i = 1; i < int(node->children_.size()); ++i ) {
            const action_node_t<T> *a_node = node->children_[i];
            if( affected.find(a_node) != affected.end() ) {
                node->lower_bound_guessed_ = a_node->lower_bound_guessed_;
                node->upper_bound_guessed_ = a_node->upper_bound_guessed_;
            } else {
                node->lower_bound_guessed_ = a_node->lower_bound_;
                node->upper_bound_guessed_ = a_node->upper_bound_;
            }
        }
#endif

        affected.insert(node);
        for( int i = 0; i < int(node->parents_.size()); ++i )
            propagate_guessed_bounds_upward(node->parents_[i].second, affected);
    }
    void propagate_guessed_bounds_upward(const action_node_t<T> *a_node, std::set<const node_t<T>*> &affected) const {
        assert(a_node != 0);
        assert(!a_node->children_.empty());

        // compute guessed bounds
        a_node->lower_bound_guessed_ = 0;
        a_node->upper_bound_guessed_ = 0;
        for( int i = 0; i < int(a_node->children_.size()); ++i ) {
            float prob = a_node->children_[i].first;
            const node_t<T> *node = a_node->children_[i].second;
            if( affected.find(node) != affected.end() ) {
                a_node->lower_bound_guessed_ += prob * node->lower_bound_guessed_;
                a_node->upper_bound_guessed_ += prob * node->upper_bound_guessed_;
            } else {
                a_node->lower_bound_guessed_ += prob * node->lower_bound_;
                a_node->upper_bound_guessed_ += prob * node->upper_bound_;
            }
        }
        a_node->lower_bound_guessed_ = a_node->action_cost_ + gamma_ * a_node->lower_bound_guessed_;
        a_node->upper_bound_guessed_ = a_node->action_cost_ + gamma_ * a_node->upper_bound_guessed_;

        // propagate upwards
        affected.insert(a_node);
        propagate_guessed_bounds_upward(a_node->parent_, affected);
    }

    void try_to_mark_node_as_pruned(state_node_t<T> &node, bool hard) const {
        // state nodes may have multiple parent nodes, mark as pruned only if all parents are pruned
        bool all_parents_pruned = true;
        for( int i = 0; all_parents_pruned && (i < int(node.parents_.size())); ++i ) {
            action_node_t<T> &parent = *node.parents_[i].second;
            all_parents_pruned = parent.pruned_;
        }

        if( !all_parents_pruned ) {
#if 1//def DEBUG2
            std::cout << "debug: pac(): failing to mark state-node as pruned:"
                      << " hard=" << hard
                      << " node=" << node;
            if( !hard ) {
                std::cout << " gap=" << node.gap();
            }
            std::cout << std::endl;
#endif
            return;
        } 
 
#ifdef DEBUG2
        std::cout << "debug: pac(): marking node as pruned:"
                  << " hard=" << hard
                  << " node=" << node;
        if( !hard ) {
            std::cout << " gap=" << node.gap();
        }
        std::cout << std::endl;
#endif

        node.pruned_ = true;
        ++num_nodes_pruned_;
        ++total_num_nodes_pruned_;
        if( !hard ) {
            ++num_nodes_pruned_soft_;
            ++total_num_nodes_pruned_soft_;
        }

        // try to mark children nodes
        for( int i = 0; i < int(node.children_.size()); ++i ) {
            action_node_t<T> &a_node = *node.children_[i];
            if( !a_node.pruned_ ) try_to_mark_node_as_pruned(a_node, hard);
        }
    }
    void try_to_mark_node_as_pruned(action_node_t<T> &a_node, bool hard) const {
        // action nodes have just one parent, mark this as pruned
#ifdef DEBUG2
        std::cout << "debug: pac(): marking node as pruned:"
                  << " hard=" << hard
                  << " a-node=" << a_node;
        if( !hard ) {
            std::cout << " gap=" << a_node.gap();
        }
        std::cout << std::endl;
#endif

        a_node.pruned_ = true;
        ++num_a_nodes_pruned_;
        ++total_num_a_nodes_pruned_;
        if( !hard ) {
            ++num_a_nodes_pruned_soft_;
            ++total_num_a_nodes_pruned_soft_;
        }

        // try to mark children nodes
        for( int i = 0; i < int(a_node.children_.size()); ++i ) {
            state_node_t<T> &node = *a_node.children_[i].second;
            if( !node.pruned_ ) try_to_mark_node_as_pruned(node, hard);
        }
    }

    void unmark_node_as_pruned(state_node_t<T> &node) const {
        assert(0); // XXX
    }

    void propagate_values(state_node_t<T> &node) const {
        update_bounds(node);
        for( int i = 0; i < int(node.parents_.size()); ++i )
            propagate_values(*node.parents_[i].second);
    }
    void propagate_values(action_node_t<T> &a_node) const {
        update_bounds(a_node);
        if( a_node.parent_ != 0 )
            propagate_values(*a_node.parent_);
    }

    float evaluate(const T &s, unsigned num_samples, unsigned depth) const {
        ++num_policy_evaluations_;
        ++total_num_policy_evaluations_;
        return Evaluation::evaluation(*base_policy_, s, num_samples, horizon_ - depth);
    }
    float evaluate(const state_node_t<T> &node) const {
        //return evaluate(*node.state_, num_samples_[node.depth_], node.depth_);
        return evaluate(node.state_, 1, node.depth_); // CHECK: 3 is number of samples at node, it should be parameter
    }

    void delete_tree(state_node_t<T> *node) const {
        assert(node->parents_.size() <= 1);
        for( int i = 0; i < int(node->children_.size()); ++i )
            delete_tree(node->children_[i]);
        delete node;
    }
    void delete_tree(action_node_t<T> *a_node) const {
        for( int i = 0; i < int(a_node->children_.size()); ++i )
            delete_tree(a_node->children_[i].second);
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
        float start_time = Utils::read_time_in_seconds();
        delete hash_;
        hash_ = new Problem::hash_t<T>(problem_);
        algorithm_->solve(problem_.init(), *hash_);
        policy_t<T>::setup_time_ = Utils::read_time_in_seconds() - start_time;
    }
};

}; // namespace PAC2

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

