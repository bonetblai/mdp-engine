#ifndef WUMPUS_BELIEF_H
#define WUMPUS_BELIEF_H

#include "belief.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <stdlib.h>

class wumpus_belief_t : public belief_t {
  protected:
    std::vector<bin_t*> pit_bins_;
    std::vector<bin_t*> wumpus_bins_;

  public:
    wumpus_belief_t() : belief_t() {
        pit_bins_.reserve(rows_ * cols_);
        wumpus_bins_.reserve(rows_ * cols_);
        for( int r = 0; r < rows_; ++r ) {
            for( int c = 0; c < cols_; ++c ) {
                pit_bins_.push_back(new bin_t(r, c, types_[r * cols_ + c]));
                wumpus_bins_.push_back(new bin_t(r, c, types_[r * cols_ + c]));
            }
        }
    }
    explicit wumpus_belief_t(const wumpus_belief_t &bel) : belief_t(bel) {
//std::cout << "wbelief_t::copy const." << std::endl;
        pit_bins_.reserve(rows_ * cols_);
        wumpus_bins_.reserve(rows_ * cols_);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            pit_bins_.push_back(new bin_t(*bel.pit_bins_[p]));
            wumpus_bins_.push_back(new bin_t(*bel.wumpus_bins_[p]));
        }
    }
    virtual ~wumpus_belief_t() {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            delete pit_bins_[p];
            delete wumpus_bins_[p];
        }
    }

    static void initialize(int rows, int cols) {
        bin_t::initialize();
        belief_t::initialize(rows, cols, belief_t::manhattan_neighbourhood);
    }

    virtual bool inconsistent() const {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            if( pit_bins_[p]->empty() ) return true;
            if( wumpus_bins_[p]->empty() ) return true;
        }
        return false;
    }

    virtual void clear() {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            pit_bins_[p]->clear();
            wumpus_bins_[p]->clear();
        }
    }

    virtual void set_as_unknown() {
        for( int p = 0; p < rows_ * cols_; ++p ) {
            pit_bins_[p]->set_as_unknown(belief_t::manhattan_neighbourhood);
            wumpus_bins_[p]->set_as_unknown(belief_t::manhattan_neighbourhood);
        }
    }

    void pit_filter(int cell, int nobjs = 1, bool at_least = true) {
        filter(pit_bins_, cell, nobjs, at_least);
        pit_ac3(cell);
    }
    void wumpus_filter(int cell, int nobjs = 1, bool at_least = true) {
        filter(wumpus_bins_, cell, nobjs, at_least);
        wumpus_ac3(cell);
    }

    virtual const wumpus_belief_t& operator=(const wumpus_belief_t &bel) {
//std::cout << "wbelief_t::operator=" << std::endl;
        belief_t::operator=(bel);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            *pit_bins_[p] = *bel.pit_bins_[p];
            *wumpus_bins_[p] = *bel.wumpus_bins_[p];
        }
        return *this;
    }

    virtual bool operator==(const wumpus_belief_t &bel) const {
        if( !(belief_t::operator==(bel)) ) return false;
        const wumpus_belief_t &wbel = static_cast<const wumpus_belief_t&>(bel);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            if( *pit_bins_[p] != *wbel.pit_bins_[p] ) return false;
            if( *wumpus_bins_[p] != *wbel.wumpus_bins_[p] ) return false;
        }
        return true;
    }
    virtual bool operator!=(const wumpus_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        for( int r = 0; r < rows_; ++r ) {
            for( int c = 0; c < cols_; ++c ) {
                int p = r * cols_ + c;
                os << "pbin(" << c << "," << r << ")=" << *pit_bins_[p]
                   << std::endl
                   << "wbin(" << c << "," << r << ")=" << *wumpus_bins_[p]
                   << std::endl;
            }
        }
    }

    void pit_ac3(int seed_bin, bool propagate = true) {
        ac3(pit_bins_, seed_bin, propagate);
    }
    void wumpus_ac3(int seed_bin, bool propagate = true) {
        ac3(wumpus_bins_, seed_bin, propagate);
    }

    // Knowledge-query methods
    bool hazard_at(int cell) const {
        return belief_t::hazard_at(pit_bins_, cell) ||
               belief_t::hazard_at(wumpus_bins_, cell);
    }
    bool no_hazard_at(int cell) const {
        return belief_t::no_hazard_at(pit_bins_, cell) &&
               belief_t::no_hazard_at(wumpus_bins_, cell);
    }
    int max_num_pits(int cell) const {
        return max_num_objs(pit_bins_, cell);
    }
    float pit_probability(int cell, float pit_prior) const {
        return obj_probability(pit_bins_, cell, pit_prior);
    }
    int max_num_wumpus(int cell) const {
        return max_num_objs(wumpus_bins_, cell);
    }
    float wumpus_probability(int cell, float wumpus_prior) const {
        return obj_probability(wumpus_bins_, cell, wumpus_prior);
    }

};

inline std::ostream& operator<<(std::ostream &os, const wumpus_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

