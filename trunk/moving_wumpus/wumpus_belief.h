#ifndef WUMPUS_BELIEF_H
#define WUMPUS_BELIEF_H

#include "grid_belief.h"
#include "grid_var_bin.h"
#include "defs.h"

#include <cassert>
#include <iostream>
#include <list>
#include <vector>

class wumpus_belief_t : public grid_belief_t {
    static int nrows_;
    static int ncols_;
    static int npits_;
    static int nwumpus_;
    static std::list<wumpus_belief_t*> beliefs_;

  protected:
    int pos_;
    int heading_;
    int narrows_;
    bool have_gold_;
    bool dead_;

    grid_var_bin_t gold_;
    grid_arc_consistency_t pits_;
    grid_arc_consistency_t wumpus_;

  public:
    wumpus_belief_t(int pos = 0, int heading = 0)
      : grid_belief_t(), pos_(pos), heading_(heading), narrows_(0),
        have_gold_(false), dead_(false), gold_(1) {
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                pits_.set_domain(cell, new cell_bin_t(r, c, types_[cell]));
                wumpus_.set_domain(cell, new cell_bin_t(r, c, types_[cell]));
            }
        }
    }
    explicit wumpus_belief_t(const wumpus_belief_t &bel)
      : grid_belief_t(bel), pos_(bel.pos_), heading_(bel.heading_), narrows_(bel.narrows_),
        have_gold_(bel.have_gold_), dead_(bel.dead_), gold_(bel.gold_) {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.set_domain(cell, new cell_bin_t(*bel.pits_.domain(cell)));
            wumpus_.set_domain(cell, new cell_bin_t(*bel.wumpus_.domain(cell)));
        }
    }
    wumpus_belief_t(wumpus_belief_t &&bel)
      : grid_belief_t(bel), pos_(bel.pos_), heading_(bel.heading_), narrows_(bel.narrows_),
        have_gold_(bel.have_gold_), dead_(bel.dead_), gold_(std::move(bel.gold_)) {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.set_domain(cell, bel.pits_.domain(cell));
            bel.pits_.set_domain(cell, 0);
            wumpus_.set_domain(cell, bel.wumpus_.domain(cell));
            bel.wumpus_.set_domain(cell, 0);
        }
    }
    virtual ~wumpus_belief_t() { }

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

    static void initialize(int nrows, int ncols, int npits, int nwumpus) {
        nrows_ = nrows;
        ncols_ = ncols;
        npits_ = npits;
        nwumpus_ = nwumpus;
        cell_bin_t::initialize();
        grid_var_bin_t::initialize(nrows_, ncols_);
        grid_belief_t::initialize(nrows_, ncols_, grid_belief_t::manhattan_neighbourhood);
    }

    size_t hash() const { return 0; }

    int position() const { return pos_; }
    int heading() const { return heading_; }
    bool have_gold() const { return have_gold_; }
    bool dead() const { return dead_; }
    void set_position(int pos) { pos_ = pos; }
    void set_heading(int heading) { heading_ = heading; }

    bool consistent() const {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            if( pits_.domain(cell)->empty() ) return false;
            if( wumpus_.domain(cell)->empty() ) return false;
        }
        return have_gold() || gold_.consistent();
    }

    bool in_gold_cell() const { return gold_.known() && (*gold_.begin() == pos_); }
    bool possible_gold_at(int cell) const { return gold_.contains(cell); }

    void clear() {
        pos_ = 0;
        heading_ = 0;
        have_gold_ = false;
        dead_ = false;
        gold_.clear();
        for( int cell = 0; cell < rows_ * cols_; ++cell ) {
            pits_.domain(cell)->clear();
            wumpus_.domain(cell)->clear();
        }
    }

    void set_as_unknown() {
        gold_.set_as_unknown();
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.domain(cell)->set_as_unknown(grid_belief_t::manhattan_neighbourhood);
            wumpus_.domain(cell)->set_as_unknown(grid_belief_t::manhattan_neighbourhood);
        }
    }

    const wumpus_belief_t& operator=(const wumpus_belief_t &bel) {
        pos_ = bel.pos_;
        heading_ = bel.heading_;
        have_gold_ = bel.have_gold_;
        dead_ = bel.dead_;
        gold_ = bel.gold_;
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            assert(pits_.domain(cell) != 0);
            assert(wumpus_.domain(cell) != 0);
            *pits_.domain(cell) = *bel.pits_.domain(cell);
            *wumpus_.domain(cell) = *bel.wumpus_.domain(cell);
        }
        return *this;
    }

    virtual bool operator==(const wumpus_belief_t &bel) const {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            if( *pits_.domain(cell) != *bel.pits_.domain(cell) ) return false;
            if( *wumpus_.domain(cell) != *bel.wumpus_.domain(cell) ) return false;
        }
        return (pos_ == bel.pos_) && (heading_ == bel.heading_) &&
               (have_gold_ == bel.have_gold_) && (dead_ == bel.dead_) &&
               (gold_ == bel.gold_);
    }
    virtual bool operator!=(const wumpus_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        os << "pos=(" << (pos_ % ncols_)
           << "," << (pos_ / ncols_)
           << "," << heading_name(heading_) << ")"
           << ", have-gold=" << (have_gold_ ? "true" : "false")
           << ", dead=" << (dead_ ? "true" : "false")
           << std::endl;
#if 0
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                os << "pbin(" << c << "," << r << ")=" << *pits_.domain(cell) << std::endl;
                os << "wbin(" << c << "," << r << ")=" << *wumpus_.domain(cell) << std::endl;
            }
        }
#endif
    }

    int target_cell(int action) const {
        return ::target_cell(pos_, heading_, action, nrows_, ncols_, false);
    }
    int target_heading(int action) const {
        return ::target_heading(heading_, action);
    }

    bool applicable(int action) const {
        return applicable_for_position(action) &&
               applicable_for_gold(action) &&
               applicable_for_pits(action) &&
               applicable_for_wumpus(action);
    }

    bool possible_obs(int action, int obs) const {
        if( pos_ == OutsideCave ) {
            return obs == 0;
        } else {
            if( obs < Fell ) {
                return possible_obs_for_gold(action, obs) &&
                       possible_obs_for_pits(action, obs) &&
                       possible_obs_for_wumpus(action, obs);
            } else if ( obs == Fell ) {
                return !no_pit_at(pos_);
            } else if( obs == Eaten ) {
                return !no_wumpus_at(pos_);
            } else {
                return false;
            }
        }
    }

    void progress(int action) {
        assert(applicable(action));
        progress_position(action);
        progress_gold(action);
        progress_pits(action);
        progress_wumpus(action);
    }

    void filter(int action, int obs) {
        if( pos_ != OutsideCave ) {
            if( obs < Fell ) {
                if( pos_ != OutsideCave ) {
                    filter_gold(action, obs);
                    filter_pits(action, obs);
                    filter_wumpus(action, obs);
                }
            } else if( obs == Fell ) {
                dead_ = true;
                pit_filter(pos_, 9, false);
            } else if( obs == Eaten ) {
                dead_ = true;
                wumpus_filter(pos_, 9, false);
            } else {
                assert(0);
            }
        }
        assert(consistent());
    }

    void progress_and_filter(int action, int obs) {
        progress(action);
        filter(action, obs);
    }

    // Knowledge-query methods
    bool pit_at(int cell) const { return pits_.domain(cell)->obj_at(); }
    bool wumpus_at(int cell) const { return wumpus_.domain(cell)->obj_at(); }
    bool hazard_at(int cell) const { return pit_at(cell) || wumpus_at(cell); }
    bool no_pit_at(int cell) const { return pits_.domain(cell)->no_obj_at(); }
    bool no_wumpus_at(int cell) const { return wumpus_.domain(cell)->no_obj_at(); }
    bool no_hazard_at(int cell) const { return no_pit_at(cell) && no_wumpus_at(cell); }

  private:
    bool applicable_for_position(int action) const {
        if( dead_ || (pos_ == OutsideCave) || (action == ActionNoop) ) {
            return false;
        } else if( action == ActionShoot ) {
            return narrows_ > 0;
        } else if( action == ActionExit ) {
            return pos_ == 0;
        } else if( action == ActionMoveForward ) {
            return target_cell(action) != pos_;
        } else {
            return true;
        }
    }
    void progress_position(int action) {
        if( action == ActionExit ) {
            pos_ = OutsideCave;
        } else if( action == ActionMoveForward ) {
            pos_ = target_cell(action);
        } else if( (action == ActionTurnLeft) || (action == ActionTurnRight) ) {
            heading_ = target_heading(action);
        }
    }

    bool applicable_for_gold(int action) const {
        return (action != ActionGrab) || in_gold_cell();
    }
    void progress_gold(int action) {
        if( action == ActionGrab ) {
            assert(!have_gold());
            assert(in_gold_cell());
            gold_.clear();
            have_gold_ = true;
        }
    }
    bool possible_obs_for_gold(int action, int obs) const {
        if( obs & Glitter ) {
            return gold_.possible_obj_at(pos_);
        } else {
            return gold_.empty() || gold_.possible_no_obj_at(pos_);
        }
    }
    void filter_gold(int action, int obs) {
        if( obs & Glitter ) {
            gold_.clear();
            gold_.insert(pos_);
        } else {
            gold_.filter_no_obj_at(pos_);
        }
    }

    bool applicable_for_pits(int action) const {
        return true;
    }
    void progress_pits(int action) {
        // nothing to do
    }
    bool possible_obs_for_pits(int action, int obs) const {
        std::pair<int, int> min_max_pits = pits_.domain(pos_)->num_surrounding_objs();    
        if( obs & Breeze ) {
            return min_max_pits.second > 0;
        } else {
            return min_max_pits.first == 0;
        }
    }
    void filter_pits(int action, int obs) {
        if( obs & Breeze ) {
            pit_filter(pos_, 1, true);
        } else {
            pit_filter(pos_, 0, false);
        }
    }

    bool applicable_for_wumpus(int action) const {
        return true;
    }
    void progress_wumpus(int action) {
        // nothing to do
    }
    bool possible_obs_for_wumpus(int action, int obs) const {
        std::pair<int, int> min_max_wumpus = wumpus_.domain(pos_)->num_surrounding_objs();    
        if( obs & Stench ) {
            return min_max_wumpus.second > 0;
        } else {
            return min_max_wumpus.first == 0;
        }
    }
    void filter_wumpus(int action, int obs) {
        if( obs & Stench ) {
            wumpus_filter(pos_, 1, true);
        } else {
            wumpus_filter(pos_, 0, false);
        }
    }

    void pit_filter(int cell, int nobjs, bool at_least) {
        std::vector<bool> revised_cells;
        pits_.domain(cell)->filter(nobjs, at_least);
        pits_.ac3(cell, revised_cells);
    }

    void wumpus_filter(int cell, int nobjs, bool at_least) {
        std::vector<bool> revised_cells;
        wumpus_.domain(cell)->filter(nobjs, at_least);
        wumpus_.ac3(cell, revised_cells);
    }
};

int wumpus_belief_t::nrows_ = 0;
int wumpus_belief_t::ncols_ = 0;
int wumpus_belief_t::npits_ = 0;
int wumpus_belief_t::nwumpus_ = 0;
std::list<wumpus_belief_t*> wumpus_belief_t::beliefs_;

inline std::ostream& operator<<(std::ostream &os, const wumpus_belief_t &bel) {
    bel.print(os);
    return os;
}



class m_wumpus_belief_t : public grid_belief_t {
    static int nrows_;
    static int ncols_;
    static int npits_;
    static int nwumpus_;
    static std::list<m_wumpus_belief_t*> beliefs_;

  protected:
    int pos_;
    int heading_;
    int narrows_;
    bool have_gold_;
    bool dead_;

    grid_var_bin_t gold_;
    grid_arc_consistency_t pits_;
    grid_var_bin_t wumpus_;

  public:
    m_wumpus_belief_t(int pos = 0, int heading = 0)
      : grid_belief_t(), pos_(pos), heading_(heading), narrows_(0),
        have_gold_(false), dead_(false),
        gold_(1), wumpus_(nwumpus_) {
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                pits_.set_domain(cell, new cell_bin_t(r, c, types_[cell]));
            }
        }
    }
    explicit m_wumpus_belief_t(const m_wumpus_belief_t &bel)
      : grid_belief_t(bel), pos_(bel.pos_), heading_(bel.heading_), narrows_(bel.narrows_),
        have_gold_(bel.have_gold_), dead_(bel.dead_),
        gold_(bel.gold_), wumpus_(bel.wumpus_) {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.set_domain(cell, new cell_bin_t(*bel.pits_.domain(cell)));
        }
    }
    m_wumpus_belief_t(m_wumpus_belief_t &&bel)
      : grid_belief_t(bel), pos_(bel.pos_), heading_(bel.heading_), narrows_(bel.narrows_),
        have_gold_(bel.have_gold_), dead_(bel.dead_),
        gold_(std::move(bel.gold_)), wumpus_(std::move(bel.wumpus_)) {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.set_domain(cell, bel.pits_.domain(cell));
            bel.pits_.set_domain(cell, 0);
        }
    }
    virtual ~m_wumpus_belief_t() { }

    static m_wumpus_belief_t* allocate() {
        if( beliefs_.empty() ) {
            return new m_wumpus_belief_t;
        } else {
            m_wumpus_belief_t *belief = beliefs_.front();
            beliefs_.pop_front();
            assert(belief != 0);
            belief->clear();
            return belief;
        }
    }
    static void deallocate(m_wumpus_belief_t *belief) {
        if( belief != 0 ) {
            beliefs_.push_front(belief);
        }
    }

    static void initialize(int nrows, int ncols, int npits, int nwumpus) {
        nrows_ = nrows;
        ncols_ = ncols;
        npits_ = npits;
        nwumpus_ = nwumpus;
        cell_bin_t::initialize();
        grid_var_bin_t::initialize(nrows_, ncols_);
        grid_belief_t::initialize(nrows_, ncols_, grid_belief_t::manhattan_neighbourhood);
    }

    size_t hash() const { return 0; }

    int position() const { return pos_; }
    int heading() const { return heading_; }
    bool have_gold() const { return have_gold_; }
    bool dead() const { return dead_; }
    void set_position(int pos) { pos_ = pos; }
    void set_heading(int heading) { heading_ = heading; }

    bool consistent() const {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            if( pits_.domain(cell)->empty() ) return false;
        }
        return wumpus_.consistent() && (have_gold() || gold_.consistent());
    }

    bool in_gold_cell() const { return gold_.known() && (*gold_.begin() == pos_); }
    bool possible_gold_at(int cell) const { return gold_.contains(cell); }

    void clear() {
        pos_ = 0;
        heading_ = 0;
        have_gold_ = false;
        dead_ = false;
        gold_.clear();
        wumpus_.clear();
        for( int cell = 0; cell < rows_ * cols_; ++cell ) {
            pits_.domain(cell)->clear();
        }
    }

    void set_as_unknown() {
        gold_.set_as_unknown();
        wumpus_.set_as_unknown();
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            pits_.domain(cell)->set_as_unknown(grid_belief_t::manhattan_neighbourhood);
        }
    }

    const m_wumpus_belief_t& operator=(const m_wumpus_belief_t &bel) {
        pos_ = bel.pos_;
        heading_ = bel.heading_;
        have_gold_ = bel.have_gold_;
        dead_ = bel.dead_;
        gold_ = bel.gold_;
        wumpus_ = bel.wumpus_;
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            assert(pits_.domain(cell) != 0);
            *pits_.domain(cell) = *bel.pits_.domain(cell);
        }
        return *this;
    }

    bool operator==(const m_wumpus_belief_t &bel) const {
        for( int cell = 0; cell < nrows_ * ncols_; ++cell ) {
            if( *pits_.domain(cell) != *bel.pits_.domain(cell) ) return false;
        }
        return (pos_ == bel.pos_) && (heading_ == bel.heading_) &&
               (have_gold_ == bel.have_gold_) && (dead_ == bel.dead_) &&
               (gold_ == bel.gold_) && (wumpus_ == bel.wumpus_);
    }
    bool operator!=(const m_wumpus_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    void print(std::ostream &os) const {
        os << "pos=(" << (pos_ % ncols_)
           << "," << (pos_ / ncols_)
           << "," << heading_name(heading_) << ")"
           << ", have-gold=" << (have_gold_ ? "true" : "false")
           << ", dead=" << (dead_ ? "true" : "false")
           << std::endl;
        //os << "gold=" << gold_ << std::endl;
        os << "wumpus=" << wumpus_ << std::endl;
#if 0
        for( int r = 0; r < nrows_; ++r ) {
            for( int c = 0; c < ncols_; ++c ) {
                int cell = r * ncols_ + c;
                os << "pbin(" << c << "," << r << ")=" << *pits_.domain(cell) << std::endl;
            }
        }
#endif
    }

    int target_cell(int action) const {
        return ::target_cell(pos_, heading_, action, nrows_, ncols_, false);
    }
    int target_heading(int action) const {
        return ::target_heading(heading_, action);
    }

    bool applicable(int action) const {
        return applicable_for_position(action) &&
               applicable_for_gold(action) &&
               applicable_for_pits(action) &&
               applicable_for_wumpus(action);
    }

    bool possible_obs(int action, int obs) const {
        if( pos_ == OutsideCave ) {
            return obs == 0;
        } else {
            if( obs & 0x4 ) {
                // if dead, this observation cannot be combined with others
                return (obs == 4) && !no_hazard_at(pos_);
            } else {
                return possible_obs_for_gold(action, obs) &&
                       possible_obs_for_pits(action, obs) &&
                       possible_obs_for_wumpus(action, obs);
            }
        }
    }

    void progress(int action) {
        assert(applicable(action));
        progress_position(action);
        progress_gold(action);
        progress_pits(action);
        progress_wumpus(action);
    }

    void filter(int action, int obs) {
        grid_var_bin_t before(wumpus_);
        if( obs & 0x4 ) {
            dead_ = true;
        } else {
            if( pos_ != OutsideCave ) {
                filter_gold(action, obs);
                filter_pits(action, obs);
                filter_wumpus(action, obs);
            }
        }
        assert(consistent());
    }

    void progress_and_filter(int action, int obs) {
        progress(action);
        filter(action, obs);
    }

    // Knowledge-query methods
    bool pit_at(int cell) const { return pits_.domain(cell)->obj_at(); }
    bool wumpus_at(int cell) const { return wumpus_.necessary_obj_at(cell); }
    bool hazard_at(int cell) const { return pit_at(cell) || wumpus_at(cell); }
    bool no_pit_at(int cell) const { return pits_.domain(cell)->no_obj_at(); }
    bool no_wumpus_at(int cell) const { return wumpus_.necessary_no_obj_at(cell); }
    bool no_hazard_at(int cell) const { return no_pit_at(cell) && no_wumpus_at(cell); }

  private:
    bool applicable_for_position(int action) const {
        if( dead_ || (pos_ == OutsideCave) || (action == ActionNoop) ) {
            return false;
        } else if( action == ActionShoot ) {
            return narrows_ > 0;
        } else if( action == ActionExit ) {
            return pos_ == 0;
        } else if( action == ActionMoveForward ) {
            return target_cell(action) != pos_;
        } else {
            return true;
        }
    }
    void progress_position(int action) {
        if( action == ActionExit ) {
            pos_ = OutsideCave;
        } else if( action == ActionMoveForward ) {
            pos_ = target_cell(action);
        } else if( (action == ActionTurnLeft) || (action == ActionTurnRight) ) {
            heading_ = target_heading(action);
        }
    }

    bool applicable_for_gold(int action) const {
        return (action != ActionGrab) || in_gold_cell();
    }
    void progress_gold(int action) {
        if( action == ActionGrab ) {
            assert(!have_gold());
            assert(in_gold_cell());
            gold_.clear();
            have_gold_ = true;
        }
    }
    bool possible_obs_for_gold(int action, int obs) const {
        if( obs & Glitter ) {
            return gold_.possible_obj_at(pos_);
        } else {
            return gold_.empty() || gold_.possible_no_obj_at(pos_);
        }
    }
    void filter_gold(int action, int obs) {
        if( obs & Glitter ) {
            gold_.filter_obj_at(pos_);
        } else {
            gold_.filter_no_obj_at(pos_);
        }
    }

    bool applicable_for_pits(int action) const {
        return true;
    }
    void progress_pits(int action) {
        // nothing to do
    }
    bool possible_obs_for_pits(int action, int obs) const {
        std::pair<int, int> min_max_pits = pits_.domain(pos_)->num_surrounding_objs();    
        if( obs & Breeze ) {
            return min_max_pits.second > 0;
        } else {
            return min_max_pits.first == 0;
        }
    }
    void filter_pits(int action, int obs) {
        if( obs & Breeze ) {
            pit_filter(pos_, 1, true);
        } else {
            pit_filter(pos_, 0, false);
        }
    }

    bool applicable_for_wumpus(int action) const {
        return true;
    }
    void progress_wumpus(int action) {
        //if( action < ActionGrab ) {
        if( action == ActionMoveForward ) {
            // this is a move action, the wumpuses move non-det in the grid
            wumpus_.move_non_det();
        }
    }
    bool possible_obs_for_wumpus(int action, int obs) const {
        int wumpus = obs >> 3;
        if( wumpus == 0 ) {
            return wumpus_.possible_no_obj_adjacent_to(pos_, grid_var_bin_t::octile_neighbourhood);
        } else {
            --wumpus;
            int drow = (wumpus / 5) - 2, dcol = (wumpus % 5) - 2;
            //int drow = (wumpus / 3) - 1, dcol = (wumpus % 3) - 1;
            int row = pos_ / ncols_, col = pos_ % ncols_;
            int wrow = row + drow, wcol = col + dcol;
            if( (wrow >= 0) && (wrow < nrows_) && (wcol >= 0) && (wcol < ncols_) ) {
                int wpos = wrow * ncols_ + wcol;
                return wumpus_.possible_obj_at(wpos);
            } else {
                return false;
            }
        }
    }
    void filter_wumpus(int action, int obs) {
        int wumpus = obs >> 3;
        if( wumpus == 0 ) {
            return wumpus_.filter_no_obj_adjacent_to(pos_, grid_var_bin_t::octile_neighbourhood);
        } else {
            --wumpus;
            int drow = (wumpus / 5) - 2, dcol = (wumpus % 5) - 2;
            //int drow = (wumpus / 3) - 1, dcol = (wumpus % 3) - 1;
            int row = pos_ / ncols_, col = pos_ % ncols_;
            int wrow = row + drow, wcol = col + dcol;
            assert((wrow >= 0) && (wrow < nrows_) && (wcol >= 0) && (wcol < ncols_));
            int wpos = wrow * ncols_ + wcol;
            wumpus_.filter_obj_at(wpos);
        }
    }

    void pit_filter(int cell, int nobjs, bool at_least) {
        std::vector<bool> revised_cells;
        pits_.domain(cell)->filter(nobjs, at_least);
        pits_.ac3(cell, revised_cells);
    }
};

int m_wumpus_belief_t::nrows_ = 0;
int m_wumpus_belief_t::ncols_ = 0;
int m_wumpus_belief_t::npits_ = 0;
int m_wumpus_belief_t::nwumpus_ = 0;
std::list<m_wumpus_belief_t*> m_wumpus_belief_t::beliefs_;

inline std::ostream& operator<<(std::ostream &os, const m_wumpus_belief_t &bel) {
    bel.print(os);
    return os;
}

#endif

