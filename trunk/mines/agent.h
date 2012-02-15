#ifndef AGENT_H
#define AGENT_H

#include "mines_belief.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <stdlib.h>

class state_t {
  protected:
    int ncells_;
    int nmines_;
    int nflags_;
    float prior_;
    std::vector<bool> flag_;
    std::vector<bool> uncovered_;
    mines_belief_t belief_;

  public:
    state_t(int rows, int cols, int nmines)
      : ncells_(rows * cols), nmines_(nmines), nflags_(0) {
        flag_ = std::vector<bool>(ncells_, false);
        uncovered_ = std::vector<bool>(ncells_, false);
        prior_ = (float)nmines_ / (float)ncells_;
    }
    explicit state_t(const state_t &state)
      : ncells_(state.ncells_), nmines_(state.nmines_),
        nflags_(state.nflags_), prior_(state.prior_),
        flag_(state.flag_), uncovered_(state.uncovered_),
        belief_(state.belief_) {
    }
    ~state_t() { }

    int rows() const { return belief_t::rows(); }
    int cols() const { return belief_t::cols(); }
    int ncells() const { return ncells_; }
    int nflags() const { return nflags_; }

    bool inconsistent() const { return belief_.inconsistent(); }

    void set_as_unknown() {
        belief_.set_as_unknown();
    }

    bool applicable(int cell) const {
        return !uncovered_[cell] && !flag_[cell];
    }

    void apply(bool flag, int cell) {
        if( flag ) {
            flag_[cell] = true;
            ++nflags_;
        } else {
            uncovered_[cell] = true;
        }
    }
    void apply(int action) {
        apply(action < ncells_, action < ncells_ ? action : action - ncells_);
    }

    void update(bool flag, int cell, int obs) {
        assert(!flag || (obs == -1));
        if( !flag ) {
            assert((0 <= obs) && (obs < 9));
            belief_.mine_filter(cell, obs);
            belief_.mine_ac3(cell);
        }
    }

    void apply_action_and_update(int action, int obs) {
        apply(action);
        update(action < ncells_, action < ncells_ ? action : action - ncells_, obs);
    }

    void print(std::ostream &os) const {
        os << "#flags=" << nflags_ << std::endl;
        os << belief_;
    }

    bool mine_at(int cell) const {
        return belief_.mine_at(cell);
    }
    bool no_mine_at(int cell) const {
        return belief_.no_mine_at(cell);
    }
    int max_num_mines(int cell) const {
        return belief_.max_num_mines(cell);
    }
    float mine_probability(int cell) const {
        return belief_.mine_probability(cell, prior_);
    }
    bool virgin(int cell) const {
        return belief_.virgin(cell);
    }

};

struct base_policy_t {

    base_policy_t() { }
    ~base_policy_t() { }

    int operator()(const state_t &state) const {

        // 1st: flag a cell in which we know there is mine
        for( int cell = 0; cell < state.ncells(); ++cell ) {
            if( state.applicable(cell) ) {
                bool mine_at = state.mine_at(cell);
                if( mine_at ) return cell;
            }
        }

        // 2nd: uncover cell in which we known there is no mine
        for( int cell = 0; cell < state.ncells(); ++cell ) {
            if( state.applicable(cell) ) {
                bool no_mine_at = state.no_mine_at(cell);
                if( no_mine_at ) return state.ncells() + cell;
            }
        }

        std::vector<int> best;
        best.reserve(state.ncells());

#if 0
        // look for virgin cell
        for( int cell = 0; cell < state.ncells(); ++cell ) {
            if( state.applicable(cell) && state.virgin(cell) ) {
                best.push_back(cell);
            }
        }
        if( !best.empty() ) {
            std::cout << "selecting (non-sure) virgin cell" << std::endl;
            return state.ncells() + best[lrand48() % best.size()];
        }
#endif

#if 1
        // 3rd: uncover cell with less probability of mine (random)
        float best_probability = 1.0;
        for( int cell = 0; cell < state.ncells(); ++cell ) {
            if( state.applicable(cell) ) {
                float probability = state.mine_probability(cell);
                if( probability <= best_probability ) {
                    if( probability < best_probability ) {
                        best_probability = probability;
                        best.clear();
                    }
                    best.push_back(cell);
                }
            }
        }
        if( !best.empty() ) {
            //std::cout << "guessing..." << std::endl;
            return state.ncells() + best[lrand48() % best.size()];
        }
#endif

        // no available action, return -1
        return -1;
    }

};

#endif

