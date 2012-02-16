#ifndef AGENT_H
#define AGENT_H

#include "wumpus_belief.h"
#include "random.h"

#include <cassert>
#include <iostream>
#include <queue>
#include <stdlib.h>
#include <limits.h>

#define COMPASS_ACTIONS

#ifdef COMPASS_ACTIONS
enum { MoveNorth = 0, MoveEast = 1, MoveSouth = 2, MoveWest = 3,
       Shoot = 4, Grab = 5, Exit = 6 };
#else
enum { Move = 0, TurnR = 1, TurnL = 2, Shoot = 3, Grab = 4, Exit = 5 };
#endif

enum { North = 0, East = 1, South = 2, West = 3 };

// position of agent and gold
enum { Unknown = -1, HaveGold = -2, OutsideCave = -3 };

// feedback
enum { Glitter = 0x1, Breeze = 0x2, Stench = 0x4, Fell = 8, Eaten = 9 };


const char* heading_string[] = { "north", "east", "south", "west" };

class state_t {
  protected:
    int rows_;
    int cols_;

    bool alive_;
    int pos_;
    int heading_;

    int gold_;
    std::vector<bool> possible_gold_;

    int npits_;
    int nwumpus_;
    int narrows_;

    wumpus_belief_t belief_;

  public:
    state_t(int rows = 0, int cols = 0, int npits = 0, int nwumpus = 0, int narrows = 0)
      : rows_(rows), cols_(cols),
        alive_(true), pos_(0), heading_(North), gold_(Unknown),
        npits_(npits), nwumpus_(nwumpus), narrows_(narrows) {
        possible_gold_ = std::vector<bool>(rows_ * cols_, true);
    }
    state_t(const state_t &state)
      : rows_(state.rows_), cols_(state.cols_),
        alive_(state.alive_), pos_(state.pos_),
        heading_(state.heading_),
        gold_(state.gold_), possible_gold_(state.possible_gold_),
        npits_(state.npits_), nwumpus_(state.nwumpus_),
        narrows_(state.narrows_), belief_(state.belief_) {
//std::cout << "state_t::copy const." << std::endl;
    }
    state_t(state_t &&state) 
      : rows_(state.rows_), cols_(state.cols_),
        alive_(state.alive_), pos_(state.pos_),
        heading_(state.heading_),
        gold_(state.gold_), possible_gold_(state.possible_gold_),
        npits_(state.npits_), nwumpus_(state.nwumpus_),
        narrows_(state.narrows_), belief_(std::move(state.belief_)) {
        //std::cout << "state_t::move const. for " << &state << std::endl;
    }
    ~state_t() { }

    size_t hash() const {
        return 0;
    }

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    int ncells() const { return rows_ * cols_; }

    bool inconsistent() const { return belief_.inconsistent(); }
    bool alive() const { return alive_; }
    bool dead() const { return !alive_; }
    int pos() const { return pos_; }
    int heading() const { return heading_; }
    int gold() const { return gold_; }
    bool have_gold() const { return gold_ == HaveGold; }
    bool in_cave() const { return pos_ != OutsideCave; }

    int npits() const { return npits_; }
    int nwumpus() const { return nwumpus_; }
    int narrows() const { return narrows_; }

    bool hazard_at(int cell) const {
        return belief_.hazard_at(cell);
    }
    bool no_hazard_at(int cell) const {
        return belief_.no_hazard_at(cell);
    }

    bool no_pit_at(int cell) const {
        return belief_.no_pit_at(cell);
    }
    bool no_wumpus_at(int cell) const {
        return belief_.no_wumpus_at(cell);
    }

    void set_as_unknown() {
        belief_.set_as_unknown();
    }

    int target_cell(int action) const {
        int npos = pos_;
        int row = pos_ / cols_;
        int col = pos_ % cols_;

#ifdef COMPASS_ACTIONS
        if( action == MoveNorth ) {
            if( row < rows_ - 1 ) npos = (row + 1) * cols_ + col;
        } else if( action == MoveEast ) {
            if( col < cols_ - 1 ) npos = row * cols_ + col + 1;
        } else if( action == MoveSouth ) {
            if( row > 0 ) npos = (row - 1) * cols_ + col;
        } else if( action == MoveWest ) {
            if( col > 0 ) npos = row * cols_ + col - 1;
        }
#else
        if( action == Move ) {
            if( heading_ == North ) {
                if( row < rows_ - 1 ) npos = (row + 1) * cols_ + col;
            } else if( heading_ == East ) {
                if( col < cols_ - 1 ) npos = row * cols_ + col + 1;
            } else if( heading_ == South ) {
                if( row > 0 ) npos = (row - 1) * cols_ + col;
            } else {
                if( col > 0 ) npos = row * cols_ + col - 1;
            }
        }
#endif

        return npos;
    }

    bool applicable(int action) const {
        if( pos_ == OutsideCave ) return false;

        if( action == Shoot ) {
            return narrows_ > 0;
        } else if( action == Grab ) {
            return pos_ == gold_;
        } else if( action == Exit ) {
            return pos_ == 0;
        }

#ifdef COMPASS_ACTIONS
        else {
            assert(action <= MoveWest);
            return target_cell(action) != pos_;
        }
#else
        else {
            assert(action <= TurnL);
            if( action == Move ) {
                return target_cell(action) != pos_;
            } else if( (action == TurnR) || (action == TurnL) ) {
                return true;
            }
        }
#endif
    }

    void apply(int action) {
        assert(applicable(action));

        if( action == Shoot ) {
            assert(0);
        } else if( action == Grab ) {
            gold_ = HaveGold;
            possible_gold_[pos_] = false;
        } else if( action == Exit ) {
            pos_ = OutsideCave;
        }

#ifdef COMPASS_ACTIONS
        else {
            assert(action <= MoveWest);
            pos_ = target_cell(action);
            if( hazard_at(pos_) ) alive_ = false;
        }
#else
        else {
            assert(action <= TurnL);
            if( action == Move ) {
                pos_ = target_cell(action);
                if( hazard_at(pos_) ) alive_ = false;
            } else if( (action == TurnR) || (action == TurnL) ) {
                heading_ += action == TurnR ? 1 : 5;
                heading_ = heading_ % 4;
            }
        }
#endif
    }

    void update(int obs) {
        if( obs == Fell ) {
            alive_ = false;
            belief_.pit_filter(pos_, 9, false);
        } else if( obs == Eaten ) {
            alive_ = false;
            belief_.wumpus_filter(pos_, 9, false);
        } else {
            assert((0 <= obs) && (obs < 8));
            if( pos_ != OutsideCave ) {
                if( obs & Glitter ) {
                    gold_ = pos_;
                    possible_gold_ = std::vector<bool>(rows_ * cols_, false);
                    possible_gold_[pos_] = true;
                } else {
                    possible_gold_[pos_] = false;
                }

                //std::cout << "pit update w/ obs=" << obs << std::endl;
                if( obs & Breeze ) {
                    belief_.pit_filter(pos_, 1, true);
                } else {
                    belief_.pit_filter(pos_, 0, false);
                }

                //std::cout << "wumpus update w/ obs=" << obs << std::endl;
                if( obs & Stench ) {
                    belief_.wumpus_filter(pos_, 1, true);
                } else {
                    belief_.wumpus_filter(pos_, 0, false);
                }
            }
        }
    }

    void apply_action_and_update(int action, int obs) {
        apply(action);
        update(obs);
    }

    bool possible_obs(int obs) {
        if( pos_ == OutsideCave ) return obs == 0;

        if( obs == Fell ) {
            return !no_pit_at(pos_);
        } else if( obs == Eaten ) {
            return !no_wumpus_at(pos_);
        } else {
            if( obs & Glitter ) {
                if( !possible_gold_[pos_] ) return false;
            } else {
                if( (gold_ != Unknown) && (gold_ == pos_) ) return false;
            }

            std::pair<int, int> npits = belief_.num_surrounding_pits(pos_);    
            if( obs & Breeze ) {
                if( npits.second == 0 ) return false;
            } else {
                if( npits.first > 0 ) return false;
            }
        
            std::pair<int, int> nwumpus = belief_.num_surrounding_wumpus(pos_);    
            if( obs & Stench ) {
                if( nwumpus.second == 0 ) return false;
            } else {
                if( nwumpus.first > 0 ) return false;
            }

            return true;
        }
        return false;
    }

    void print(std::ostream &os) const {
        os << "pos=(" << (pos_ % cols_) << "," << (pos_ / cols_) << ")"
           << ", heading=" << heading_string[heading_]
           << ", gold=" << gold_
           << ", alive=" << alive_
           << std::endl;
        os << belief_;
    }

    const state_t& operator=(const state_t &s) {
//std::cout << "state_t::operator=" << std::endl;
        rows_ = s.rows_;
        cols_ = s.cols_;
        alive_ = s.alive_;
        pos_ = s.pos_;
        heading_ = s.heading_;
        gold_ = s.gold_;
        possible_gold_ = s.possible_gold_;
        npits_ = s.npits_;
        nwumpus_ = s.nwumpus_;
        narrows_ = s.narrows_;
        belief_ = s.belief_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        if( (rows_ != s.rows_) || (cols_ != s.cols_) )
            return false;
        if( (alive_ != s.alive_) || (pos_ != s.pos_) )
            return false;
        if( (heading_ != s.heading_) || (gold_ != s.gold_) )
            return false;
        if( (npits_ != s.npits_) || (nwumpus_ != s.nwumpus_) || (narrows_ != s.narrows_) )
            return false;
        if( possible_gold_ != s.possible_gold_ )
            return false;
        if( belief_ != s.belief_ )
            return false;
        return true;
    }
    bool operator<(const state_t &s) const {
        assert(0); return false;
    }
    
};

inline std::ostream& operator<<(std::ostream &os, const state_t &state) {
    state.print(os);
    return os;
}

#endif

