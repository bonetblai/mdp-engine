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

    mutable float score_;

    state_node_t(const T &state, unsigned char depth, node_t<T> *parent)
      : node_t<T>(state, depth, parent), score_(0) {
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
  using policy_t<T>::problem;

  protected:
    unsigned width_;
    unsigned horizon_;
    float parameter_;
    bool random_ties_;

    // parameters
    mutable float delta_;
    mutable float epsilon_;
    mutable float damping_;
    mutable unsigned max_num_samples_;
    mutable const Heuristic::heuristic_t<T> *heuristic_;

    // computed parameters
    mutable float gamma_;
    mutable std::vector<float> solved_threshold_;
    mutable std::vector<float> num_samples_;

    // leaf nodes
    mutable std::vector<state_node_t<T>*> leaf_nodes_;

    // stats
    mutable int num_nodes_pruned_;
    mutable int num_a_nodes_pruned_;
    mutable int num_evaluations_;

  public:
    pac_tree_t(const policy_t<T> &base_policy,
               unsigned width,
               unsigned horizon,
               float parameter,
               bool random_ties)
      : improvement_t<T>(base_policy),
        width_(width),
        horizon_(horizon),
        parameter_(parameter),
        random_ties_(random_ties) {
        std::stringstream name_stream;
        name_stream << "pac-tree("
                    << "width=" << width_
                    << ",horizon=" << horizon_
                    << ",par=" << parameter_
                    << ",random-ties=" << (random_ties_ ? "true" : "false")
                    << ")";
        policy_t<T>::set_name(name_stream.str());
        num_nodes_pruned_ = 0;
        num_a_nodes_pruned_ = 0;
        num_evaluations_ = 0;
    }
    virtual ~pac_tree_t() { }

    void set_parameters(float epsilon,
                        float delta,
                        unsigned max_num_samples,
                        float damping,
                        const Heuristic::heuristic_t<T> *heuristic) const {
        epsilon_ = epsilon;
        delta_ = delta;
        max_num_samples_ = max_num_samples;
        damping_ = damping;
        heuristic_ = heuristic;

        gamma_ = problem().discount();

std::cout << "PAC: epsilon=" << epsilon_ << ", delta=" << delta_ << ", gamma=" << gamma_ << ", max-samples=" << max_num_samples_ << ", damping=" << damping_ << std::endl;

        // threshold = epsilon / 2 * gamma^d
std::cout << "PAC: thresholds:";
        solved_threshold_ = std::vector<float>(1 + horizon_, 0);
        for( int d = 0; d <= horizon_; ++d ) {
            float threshold = epsilon_ / (2 * powf(gamma_, d));
            solved_threshold_[d] = threshold;
std::cout << " " << std::setprecision(4) << threshold;
        }
std::cout << std::endl;

        // l(s,d) = 4 gamma^(2d) C_max (d ln b + d ln 2 - ln delta) / (1 - gamma) epsilon^2
std::cout << "PAC: num-samples:";
        num_samples_ = std::vector<float>(1 + horizon_, 0);
        for( int d = 0; d <= horizon_; ++d ) {
            float lsd = 4 * powf(gamma_ * gamma_, d) * problem().max_absolute_cost();
std::cout << " [" << lsd;
            lsd *= d * log(2 * problem().max_combined_branching()) - log(delta_);
std::cout << " " << lsd;
            lsd /= (1 - gamma_) * epsilon_ * epsilon_;
std::cout << " " << lsd << "]";
            num_samples_[d] = std::min<float>(lsd, max_num_samples_);
        }
std::cout << std::endl;
    }

    virtual Problem::action_t operator()(const T &s) const {
        std::priority_queue<state_node_t<T>*, std::vector<state_node_t<T>*>, state_node_comparator_t<T> > heap, heap_aux;

        ++policy_t<T>::decisions_;

        state_node_t<T> *root = new state_node_t<T>(s, 0, 0);
        compute_bounds(*root);

        Problem::action_t action = 0;
        if( solved(*root) ) {
            std::cout << "PAC: root=" << *root << ", lb=" << root->lower_bound_ << ", ub=" << root->upper_bound_ << ", gap=" << root->gap() << ", threshold=" << solved_threshold_[root->depth_] << std::endl;
            delete root;
            action = improvement_t<T>::base_policy_(s);
            std::cout << "PAC: selection: action=" << action << ", method=base-policy" << std::endl;
        } else {
            compute_score(*root);
            heap.push(root);
            int last_heap_size = heap.size();

            leaf_nodes_.clear();
            for( int i = 0; !heap.empty() && (i < width_); ++i ) {
                state_node_t<T> &node = *heap.top();
                heap.pop();
                //std::cout << "PAC: pop=" << node << ", gap=" << node.gap() << ", score=" << node.score_ << ", gap-at-root=" << root->gap() << std::endl;

                // node must be active leaf
                if( node.pruned_ ) continue; // node can become pruned after it is inserted in heap
                assert(!solved(node));
                assert(node.children_.empty());

                // expand leaf node
                expand(node, leaf_nodes_);
                propagate_values(node);

                if( heap.empty() ) {
                    //std::cout << "PAC: #leaves=" << leaf_nodes_.size() << std::endl;
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

                    std::cout << "PAC: #selected=" << heap.size()
                              << ", #remaining=" << leaf_nodes_.size()
                              << ", #solved=" << nsolved
                              << ", #pruned=" << npruned
                              << ", #non-leaf=" << nonleaf
                              << std::endl;
                    assert(n_leaf_nodes == heap.size() + leaf_nodes_.size() + nsolved + npruned + nonleaf);
                }
            }
            //std::cout << "PAC: main loop ended: heap-sz=" << heap.size() << ", #leaves=" << leaf_nodes_.size() << std::endl;

            std::cout << "PAC: root=" << *root << ", lb=" << root->lower_bound_ << ", ub=" << root->upper_bound_ << ", gap=" << root->gap() << ", threshold=" << solved_threshold_[root->depth_] << std::endl;
            action = root->select_action(random_ties_);
            delete_tree(root);
            std::cout << "PAC: selection: action=" << action << ", method=tree" << std::endl;
        }
        assert(problem().applicable(s, action));
        return action;
    }
    virtual const policy_t<T>* clone() const {
        return new pac_tree_t(improvement_t<T>::base_policy_, width_, horizon_, parameter_, random_ties_);
    }
    virtual void print_stats(std::ostream &os) const {
        os << "stats: policy=" << policy_t<T>::name() << std::endl;
        os << "stats: decisions=" << policy_t<T>::decisions_ << std::endl;
        os << "stats: num-nodes-pruned=" << num_nodes_pruned_ << std::endl;
        os << "stats: num-a-nodes-pruned=" << num_a_nodes_pruned_ << std::endl;
        os << "stats: num-evaluations=" << num_evaluations_ << std::endl;
        improvement_t<T>::base_policy_.print_stats(os);
    }

    void compute_bounds(state_node_t<T> &node) const {
        assert(node.lower_bound_ == 0);
        assert(node.upper_bound_ == 0);
        node.lower_bound_ = heuristic_->value(*node.state_); // XXXX: heuristic should use (s, depth)
        node.upper_bound_ = evaluate(node);
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
            //std::cout << "XXX.0: node=" << node << ", lb=" << node.lower_bound_est_ << ", ub=" << node.upper_bound_est_ << std::endl;

            update_bounds_est(static_cast<const action_node_t<T>*>(node.parent_), &node);

            float score = 0;
            float damping = powf(damping_, node.depth_ - 1);
            for( const node_t<T> *n = node.parent_; n != 0; n = n->parent_ ) {
               n = n->parent_;
               assert(dynamic_cast<const state_node_t<T>*>(n) != 0);
               //std::cout << "score=" << score << ", damping=" << damping << ", depth=" << int(n->depth_) << ", gap=" << n->gap_est() << ", dgap=" << n->gap_est() * damping << std::endl;
               float gap_reduction = n->gap() - n->gap_est();
               //assert(gap_reduction >= 0);
               //score = std::max(score, gap_reduction * damping);
               //damping /= damping_;
               score += gap_reduction * damping;
               damping /= damping_;
            }
            node.score_ = score;
        }
        //std::cout << "XXX.1: node=" << node << ", score=" << node.score_ << std::endl;
    }

    void expand(state_node_t<T> &node, std::vector<state_node_t<T>*> &leaf_nodes) const {
        assert(node.children_.empty());
        int nactions = problem().number_actions(*node.state_);
        std::vector<node_t<T>*> children;
        children.reserve(nactions);
        for( Problem::action_t a = 0; a < nactions; ++a ) {
            if( problem().applicable(*node.state_, a) ) {
                action_node_t<T> *a_node = new action_node_t<T>(*node.state_, node.depth_, a, &node);
                std::vector<std::pair<T, float> > outcomes;
                problem().next(*node.state_, a, outcomes);

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
        node.lower_bound_ = node.children_[0]->lower_bound_;
        node.upper_bound_ = node.children_[0]->upper_bound_;
        for( int i = 1; i < int(node.children_.size()); ++i ) {
            const action_node_t<T> &a_node = *static_cast<const action_node_t<T>*>(node.children_[i]);
            node.lower_bound_ = std::min(node.lower_bound_, a_node.lower_bound_);
            node.upper_bound_ = std::min(node.upper_bound_, a_node.upper_bound_);
        }

        // mark nodes (subtrees) as pruned
        for( int i = 0; i < int(node.children_.size()); ++i ) {
            action_node_t<T> &a_node = *static_cast<action_node_t<T>*>(node.children_[i]);
            if( (node.upper_bound_ < a_node.lower_bound_) && !a_node.pruned_ ) {
                //std::cout << "Pruning: a-node=" << a_node << std::endl;
                mark_node_as_pruned(a_node);
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
        a_node.lower_bound_ = problem().cost(*a_node.state_, a_node.action_) + gamma_ * a_node.lower_bound_;
        a_node.upper_bound_ = problem().cost(*a_node.state_, a_node.action_) + gamma_ * a_node.upper_bound_;
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
        a_node->lower_bound_est_ = problem().cost(*a_node->state_, a_node->action_) + gamma_ * a_node->lower_bound_est_;
        a_node->upper_bound_est_ = problem().cost(*a_node->state_, a_node->action_) + gamma_ * a_node->upper_bound_est_;

        assert(a_node->parent_ != 0);
        assert(dynamic_cast<const state_node_t<T>*>(a_node->parent_) != 0);
        update_bounds_est(static_cast<const state_node_t<T>*>(a_node->parent_), a_node);
    }

    void mark_node_as_pruned(state_node_t<T> &node) const {
        //std::cout << "  Marking node as pruned: node=" << node << std::endl;
        for( int i = 0; i < int(node.children_.size()); ++i ) {
            action_node_t<T> &a_node = *static_cast<action_node_t<T>*>(node.children_[i]);
            if( !a_node.pruned_ ) mark_node_as_pruned(a_node);
        }
        node.pruned_ = true;
        ++num_nodes_pruned_;
    }
    void mark_node_as_pruned(action_node_t<T> &a_node) const {
        //std::cout << "  Marking node as pruned: a-node=" << a_node << std::endl;
        for( int i = 0; i < int(a_node.children_.size()); ++i ) {
            state_node_t<T> &node = *static_cast<state_node_t<T>*>(a_node.children_[i]);
            if( !node.pruned_ ) mark_node_as_pruned(node);
        }
        a_node.pruned_ = true;
        ++num_a_nodes_pruned_;
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
        ++num_evaluations_;
        return Evaluation::evaluation(improvement_t<T>::base_policy_, s, num_samples, horizon_ - depth);
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
};

}; // namespace PAC

template<typename T>
inline const policy_t<T>* make_pac_tree(const policy_t<T> &base_policy,
                                        unsigned width,
                                        unsigned horizon,
                                        float parameter,
                                        bool random_ties) {
    return new PAC::pac_tree_t<T>(base_policy, width, horizon, parameter, random_ties);
}

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

