#ifndef MINES_BELIEF_H
#define MINES_BELIEF_H

#include "belief.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <stdlib.h>

class mines_belief_t : public belief_t {
  protected:
    std::vector<bin_t*> bins_;

  public:
    mines_belief_t() : belief_t() {
        bins_.reserve(rows_ * cols_);
        for( int r = 0; r < rows_; ++r ) {
            for( int c = 0; c < cols_; ++c ) {
                bins_.push_back(new bin_t(r, c, types_[r * cols_ + c]));
            }
        }
    }
    explicit mines_belief_t(const mines_belief_t &bel) : belief_t(bel) {
        bins_.reserve(rows_ * cols_);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            bins_.push_back(new bin_t(*bel.bins_[p]));
        }
    }
    virtual ~mines_belief_t() {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            delete bins_[p];
        }
    }

    static void initialize(int rows, int cols) {
        bin_t::initialize();
        belief_t::initialize(rows, cols, belief_t::octile_neighbourhood);
    }

    virtual bool inconsistent() const {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            if( bins_[p]->empty() ) return true;
        }
        return false;
    }

    virtual void clear() {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            bins_[p]->clear();
        }
    }

    virtual void set_as_unknown() {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            bins_[p]->set_as_unknown(belief_t::octile_neighbourhood);
        }
    }

    void mine_filter(int cell, int nobjs) {
        filter(bins_, cell, nobjs);
        mine_ac3(cell);
    }

    virtual const mines_belief_t& operator=(const belief_t &bel) {
        static_cast<belief_t&>(*this) = bel;
        const mines_belief_t &mbel = static_cast<const mines_belief_t&>(bel);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            *bins_[p] = *mbel.bins_[p];
        }
        return *this;
    }

    virtual bool operator==(const belief_t &bel) const {
        if( static_cast<const belief_t&>(*this) != bel ) return false;
        const mines_belief_t &mbel = static_cast<const mines_belief_t&>(bel);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            if( *bins_[p] != *mbel.bins_[p] ) return false;
        }
        return true;
    }
    virtual bool operator!=(const belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        for( int r = 0; r < rows_; ++r ) {
            for( int c = 0; c < cols_; ++c ) {
                int p = r * cols_ + c;
                os << "bin(" << c << "," << r << ")=" << *bins_[p] << std::endl;
            }
        }
    }

    void mine_ac3(int seed_bin, bool propagate = true) {
        ac3(bins_, seed_bin, propagate);
    }

    // Knowledge-query methods
    bool mine_at(int cell) const {
        return belief_t::hazard_at(bins_, cell);
    }
    bool no_mine_at(int cell) const {
        return belief_t::no_hazard_at(bins_, cell);
    }
    std::pair<int, int> num_surrounding_mines(int cell) const {
        return num_surrounding_objs(bins_, cell);
    }
    float mine_probability(int cell, float prior) const {
        return obj_probability(bins_, cell, prior);
    }
    bool virgin(int cell) const {
        return belief_t::virgin(bins_, cell);
    }

};

inline std::ostream& operator<<(std::ostream &os, const mines_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

