#ifndef WUMPUS_BELIEF_H
#define WUMPUS_BELIEF_H

#include "belief.h"
#include <cassert>
#include <iostream>
#include <list>
#include <vector>
#include <stdlib.h>

class wumpus_belief_t : public belief_t {
    static std::list<wumpus_belief_t*> beliefs_;

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
        pit_bins_.reserve(rows_ * cols_);
        wumpus_bins_.reserve(rows_ * cols_);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            pit_bins_.push_back(new bin_t(*bel.pit_bins_[p]));
            wumpus_bins_.push_back(new bin_t(*bel.wumpus_bins_[p]));
        }
    }
    wumpus_belief_t(wumpus_belief_t &&bel) : belief_t(std::move(bel)) {
        pit_bins_.reserve(rows_ * cols_);
        wumpus_bins_.reserve(rows_ * cols_);
        for( int p = 0; p < rows_ * cols_; ++p ) {
            pit_bins_.push_back(bel.pit_bins_[p]);
            wumpus_bins_.push_back(bel.wumpus_bins_[p]);
        }
        bel.pit_bins_.clear();
        bel.wumpus_bins_.clear();
        //std::cout << "wumpus_belief_t::move const. for " << &bel << std::endl;
    }
    virtual ~wumpus_belief_t() {
        for( int i = 0, isz = pit_bins_.size(); i < isz; ++i )
            delete pit_bins_[i];
        for( int i = 0, isz = wumpus_bins_.size(); i < isz; ++i )
            delete wumpus_bins_[i];
    }

    static wumpus_belief_t* allocate() {
        if( beliefs_.empty() ) {
            return new wumpus_belief_t;
        } else {
            wumpus_belief_t *belief = beliefs_.front();
            beliefs_.pop_front();
            assert(belief != 0);
            belief->clear();
            return belief;
        }
    }
    static void deallocate(wumpus_belief_t *belief) {
        if( belief != 0 ) {
            beliefs_.push_front(belief);
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

    void pit_filter(int cell, int nobjs, bool at_least) {
        filter(pit_bins_, cell, nobjs, at_least);
        pit_ac3(cell);
    }
    void wumpus_filter(int cell, int nobjs, bool at_least) {
        filter(wumpus_bins_, cell, nobjs, at_least);
        wumpus_ac3(cell);
    }

    virtual const wumpus_belief_t& operator=(const wumpus_belief_t &bel) {
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

    virtual void mark_cell(std::vector<bin_t*> &bins, int cell, bool hazard) {
    }
    void pit_ac3(int seed_bin, bool propagate = true) {
        ac3(pit_bins_, seed_bin, propagate);
    }
    void wumpus_ac3(int seed_bin, bool propagate = true) {
        ac3(wumpus_bins_, seed_bin, propagate);
    }

    // Knowledge-query methods
    bool pit_at(int cell) const {
        return belief_t::hazard_at(pit_bins_, cell);
    }
    bool wumpus_at(int cell) const {
        return belief_t::hazard_at(wumpus_bins_, cell);
    }
    bool hazard_at(int cell) const {
        return pit_at(cell) || wumpus_at(cell);
    }

    bool no_pit_at(int cell) const {
        return belief_t::no_hazard_at(pit_bins_, cell);
    }
    bool no_wumpus_at(int cell) const {
        return belief_t::no_hazard_at(wumpus_bins_, cell);
    }
    bool no_hazard_at(int cell) const {
        return no_pit_at(cell) && no_wumpus_at(cell);
    }

    std::pair<int, int> num_surrounding_pits(int cell) const {
        return num_surrounding_objs(pit_bins_, cell);
    }
    float pit_probability(int cell, float pit_prior) const {
        return obj_probability(pit_bins_, cell, pit_prior);
    }

    std::pair<int, int> num_surrounding_wumpus(int cell) const {
        return num_surrounding_objs(wumpus_bins_, cell);
    }
    float wumpus_probability(int cell, float wumpus_prior) const {
        return obj_probability(wumpus_bins_, cell, wumpus_prior);
    }

};

std::list<wumpus_belief_t*> wumpus_belief_t::beliefs_;

inline std::ostream& operator<<(std::ostream &os, const wumpus_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

