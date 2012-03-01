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
    int ncovered_;
    std::vector<bool> flag_;
    std::vector<bool> uncovered_;
    mines_belief_t belief_;

  public:
    state_t(int rows, int cols, int nmines)
      : ncells_(rows * cols), nmines_(nmines), nflags_(0), ncovered_(ncells_) {
        flag_ = std::vector<bool>(ncells_, false);
        uncovered_ = std::vector<bool>(ncells_, false);
    }
    explicit state_t(const state_t &state)
      : ncells_(state.ncells_), nmines_(state.nmines_),
        nflags_(state.nflags_), ncovered_(state.ncovered_),
        flag_(state.flag_), uncovered_(state.uncovered_),
        belief_(state.belief_) {
    }
    ~state_t() { }

    int ncells() const { return ncells_; }
    int nflags() const { return nflags_; }
    int ncovered() const { return ncovered_; }

    int remove_cell_with_mine() const {
        if( belief_.cells_with_mine().empty() ) {
            return -1;
        } else {
            int cell = belief_.cells_with_mine().back();
            belief_.cells_with_mine().pop_back();
            return cell;
        }
    }

    int remove_cell_without_mine() const {
        if( belief_.cells_without_mine().empty() ) {
            return -1;
        } else {
            int cell = belief_.cells_without_mine().back();
            belief_.cells_without_mine().pop_back();
            return cell;
        }
    }

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
            //uncovered_[cell] = true;
            //--ncovered_;
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
        uncovered_[cell] = true;
        --ncovered_;
    }

    void apply_action_and_update(int action, int obs) {
        apply(action);
        update(action < ncells_, action < ncells_ ? action : action - ncells_, obs);
    }

    void print(std::ostream &os) const {
        os << "#flags=" << nflags_ << std::endl;
        os << belief_;
    }

    void mark_cell(int cell) {
        belief_.mark_cell(cell);
    }
    bool marked_cell(int cell) const {
        return belief_.marked_cell(cell);
    }

    bool mine_at(int cell) const {
        return belief_.mine_at(cell);
    }
    bool no_mine_at(int cell) const {
        return belief_.no_mine_at(cell);
    }
    std::pair<int, int> num_surrounding_mines(int cell) const {
        return belief_.num_surrounding_mines(cell);
    }
    float mine_probability(int cell) const {
        float prior = (float)(nmines_ - nflags_) / (float)ncovered_;
        //std::cout << "nmines=" << nmines_ - nflags_ << std::endl;
        //std::cout << "ncovered=" << ncells_ - ncovered_ << std::endl;
        //std::cout << "prior=" << prior << std::endl;
        return belief_.mine_probability(cell, prior);
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
        int cell_with_mine = state.remove_cell_with_mine();
        while( (cell_with_mine != -1) && !state.applicable(cell_with_mine) ) {
            cell_with_mine = state.remove_cell_with_mine();
        }
        if( cell_with_mine != -1 ) {
            assert(state.applicable(cell_with_mine));
            assert(state.mine_at(cell_with_mine));
            return cell_with_mine;
        }
        //for( int cell = 0; cell < state.ncells(); ++cell ) {
        //    if( state.applicable(cell) )
        //        assert(state.marked_cell(cell) || !state.mine_at(cell));
        //}

        // 2nd: uncover cell in which we known there is no mine
        int cell_without_mine = state.remove_cell_without_mine();
        while( (cell_without_mine != -1) && !state.applicable(cell_without_mine) ) {
            cell_without_mine = state.remove_cell_without_mine();
        }
        if( cell_without_mine != -1 ) {
            assert(state.applicable(cell_without_mine));
            assert(state.no_mine_at(cell_without_mine));
            return state.ncells() + cell_without_mine;
        }
        //for( int cell = 0; cell < state.ncells(); ++cell ) {
        //    if( state.applicable(cell) )
        //        assert(state.marked_cell(cell) || !state.no_mine_at(cell));
        //}

        // 3rd: we must guess a cell to open
        std::cout << "guessing..." << std::endl;

        std::vector<int> best;
        best.reserve(state.ncells());

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
            return state.ncells() + best[lrand48() % best.size()];
        }

        // 4th: no available action, return -1
        return -1;
    }

};

#endif

