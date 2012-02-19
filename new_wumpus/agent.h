#ifndef AGENT_H
#define AGENT_H

#include "wumpus_belief.h"
#include "random.h"

#include <cassert>
#include <iostream>
#include <queue>
#include <stdlib.h>
#include <limits.h>

// actions
enum { ActionMoveNorth = 0, ActionMoveEast = 1, ActionMoveSouth = 2, ActionMoveWest = 3 };
enum { ActionMoveForward = 0, ActionTurnRight = 1, ActionTurnLeft = 2, ActionNoop = 3 };
enum { ActionShoot = 4, ActionGrab = 5, ActionExit = 6 };

const char* compass_action_str[] = {
    "MoveNorth",
    "MoveEast",
    "MoveSouth",
    "MoveWest",
    "Shoot",
    "Grab",
    "Exit"
};

const char* no_compass_action_str[] = {
    "MoveForward",
    "TurnRight",
    "TurnLeft",
    "Shoot",
    "Grab",
    "Exit"
};

inline const char* action_name(int action, bool compass) {
    return compass ? compass_action_str[action] : no_compass_action_str[action];
}

// heading
enum { North = 0, East = 1, South = 2, West = 3 };

// position of agent and gold
enum { Unknown = -1, HaveGold = -2, OutsideCave = -3 };

// feedback
enum { Glitter = 0x1, Breeze = 0x2, Stench = 0x4, Fell = 8, Eaten = 9 };

const char* obs_str[] = {
    "(null,null,null)",
    "(glitter,null,null)",
    "(null,breeze,null)",
    "(glitter,breeze,null)",
    "(null,null,stench)",
    "(glitter,null,stench)",
    "(null,breeze,stench)",
    "(glitter,breeze,stench)",
    "Fell in Pit",
    "Eaten by Wumpus"
};

inline const char* obs_name(int obs) {
    return obs_str[obs];
}

const char* heading_str[] = { "north", "east", "south", "west" };

inline const char* heading_name(int heading) {
    return heading_str[heading];
}


inline int target_cell(int pos, int heading, int action, int rows, int cols, bool compass) {
    int npos = pos;
    int row = pos / cols;
    int col = pos % cols;

    if( compass ) {
        if( action == ActionMoveNorth ) {
            if( row < rows - 1 ) npos = (row + 1) * cols + col;
        } else if( action == ActionMoveEast ) {
            if( col < cols - 1 ) npos = row * cols + col + 1;
        } else if( action == ActionMoveSouth ) {
            if( row > 0 ) npos = (row - 1) * cols + col;
        } else if( action == ActionMoveWest ) {
            if( col > 0 ) npos = row * cols + col - 1;
        }
    } else {
        if( action == ActionMoveForward ) {
            if( heading == North ) {
                if( row < rows - 1 ) npos = (row + 1) * cols + col;
            } else if( heading == East ) {
                if( col < cols - 1 ) npos = row * cols + col + 1;
            } else if( heading == South ) {
                if( row > 0 ) npos = (row - 1) * cols + col;
            } else {
                if( col > 0 ) npos = row * cols + col - 1;
            }
        }
    }
    return npos;
}

inline int target_heading(int heading, int action) {
    if( (action == ActionTurnLeft) || (action == ActionTurnRight) ) {
        return (action == ActionTurnLeft ? heading + 5 : heading + 1) & 0x3;
    } else {
        return heading;
    }
}


class state_t {
  protected:
    bool alive_;
    int pos_;
    int heading_;
    int narrows_;
    int gold_;

    std::vector<bool> possible_gold_;
    std::vector<bool> visited_;

    wumpus_belief_t *belief_;

    static int rows_;
    static int cols_;
    //static int npits_;
    //static int nwumpus_;
    static bool compass_;

  public:
    state_t(int pos = 0, int heading = North, int narrows = 0)
      : alive_(true), pos_(pos), heading_(heading),
        narrows_(narrows), gold_(Unknown) {
        belief_ = wumpus_belief_t::allocate();
        possible_gold_ = std::vector<bool>(rows_ * cols_, true);
        visited_ = std::vector<bool>(rows_ * cols_, false);
        visited_[pos_] = true;
    }
    state_t(const state_t &state)
      : alive_(state.alive_), pos_(state.pos_), heading_(state.heading_),
        narrows_(state.narrows_), gold_(state.gold_),
        possible_gold_(state.possible_gold_), visited_(state.visited_) {
        belief_ = wumpus_belief_t::allocate();
        *belief_ = *state.belief_;
    }
    state_t(state_t &&state) 
      : alive_(state.alive_), pos_(state.pos_), heading_(state.heading_),
        narrows_(state.narrows_), gold_(state.gold_),
        possible_gold_(std::move(state.possible_gold_)),
        visited_(std::move(state.visited_)),
        belief_(state.belief_) {
        state.belief_ = 0;
    }
    ~state_t() {
        wumpus_belief_t::deallocate(belief_);
    }

    static void initialize(int rows, int cols, int npits, int nwumpus, bool compass) {
        rows_ = rows;
        cols_ = cols;
        //npits_ = npits;
        //nwumpus_ = nwumpus;
        compass_ = compass;
        std::cout << "state_t::initialize:"
                  << " rows=" << rows_
                  << ", cols=" << cols_
                  //<< ", npits=" << npits_
                  //<< ", nwumpus=" << nwumpus_
                  << ", compass-movements=" << (compass_ ? "yes" : "no")
                  << std::endl;
    }

    size_t hash() const {
        return 0;
    }

    int ncells() const { return rows_ * cols_; }
    bool inconsistent() const { return belief_->inconsistent(); }
    bool alive() const { return alive_; }
    bool dead() const { return !alive_; }
    int pos() const { return pos_; }
    int heading() const { return heading_; }
    int gold() const { return gold_; }
    bool have_gold() const { return gold_ == HaveGold; }
    bool in_gold_cell() const { return gold_ == pos_; }
    bool in_cave() const { return pos_ != OutsideCave; }
    bool visited(int cell) const { return visited_[cell]; }

    //int npits() const { return npits_; }
    //int nwumpus() const { return nwumpus_; }
    int narrows() const { return narrows_; }

    int target_cell(int action) const {
        return ::target_cell(pos_, heading_, action, rows_, cols_, true); // CHECK
    }
    int target_heading(int action) const {
        return ::target_heading(heading_, action);
    }

    bool hazard_at(int cell) const {
        return belief_->hazard_at(cell);
    }
    bool no_hazard_at(int cell) const {
        return belief_->no_hazard_at(cell);
    }

    bool no_pit_at(int cell) const {
        return belief_->no_pit_at(cell);
    }
    bool no_wumpus_at(int cell) const {
        return belief_->no_wumpus_at(cell);
    }

    void set_as_unknown() {
        belief_->set_as_unknown();
    }

    bool applicable(int action) const {
        if( pos_ == OutsideCave ) return false;
        if( !compass_ && (action == ActionNoop) ) return false;

        if( action == ActionShoot ) {
            return narrows_ > 0;
        } else if( action == ActionGrab ) {
            return pos_ == gold_;
        } else if( action == ActionExit ) {
            return pos_ == 0;
        }

        if( compass_ ) {
            assert(action <= ActionMoveWest);
            return target_cell(action) != pos_;
        } else {
            assert(action <= ActionTurnLeft);
            if( action == ActionMoveForward ) {
                return target_cell(action) != pos_;
            } else {
                return true;
            }
        }
    }

    void apply(int action) {
        assert(applicable(action));

        if( action == ActionShoot ) {
            assert(0);
        } else if( action == ActionGrab ) {
            gold_ = HaveGold;
            possible_gold_[pos_] = false;
        } else if( action == ActionExit ) {
            pos_ = OutsideCave;
        } else {
            if( compass_ ) {
                assert(action <= ActionMoveWest);
                pos_ = target_cell(action);
                if( hazard_at(pos_) ) alive_ = false;
            } else {
                assert(action <= ActionTurnLeft);
                if( action == ActionMoveForward ) {
                    pos_ = target_cell(action);
                    if( hazard_at(pos_) ) alive_ = false;
                } else {
                    heading_ = target_heading(action);
                }
            }
            visited_[pos_] = true;
        }
    }

    void update(int obs) {
        if( obs == Fell ) {
            alive_ = false;
            belief_->pit_filter(pos_, 9, false);
        } else if( obs == Eaten ) {
            alive_ = false;
            belief_->wumpus_filter(pos_, 9, false);
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
                    belief_->pit_filter(pos_, 1, true);
                } else {
                    belief_->pit_filter(pos_, 0, false);
                }

                //std::cout << "wumpus update w/ obs=" << obs << std::endl;
                if( obs & Stench ) {
                    belief_->wumpus_filter(pos_, 1, true);
                } else {
                    belief_->wumpus_filter(pos_, 0, false);
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

            std::pair<int, int> npits = belief_->num_surrounding_pits(pos_);    
            if( obs & Breeze ) {
                if( npits.second == 0 ) return false;
            } else {
                if( npits.first > 0 ) return false;
            }
        
            std::pair<int, int> nwumpus = belief_->num_surrounding_wumpus(pos_);    
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
           << ", heading=" << heading_name(heading_)
           << ", gold=" << gold_
           << ", alive=" << alive_
           << std::endl;
        os << *belief_;
    }

    const state_t& operator=(const state_t &s) {
        rows_ = s.rows_;
        cols_ = s.cols_;
        alive_ = s.alive_;
        pos_ = s.pos_;
        heading_ = s.heading_;
        gold_ = s.gold_;
        possible_gold_ = s.possible_gold_;
        //npits_ = s.npits_;
        //nwumpus_ = s.nwumpus_;
        narrows_ = s.narrows_;
        *belief_ = *s.belief_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        if( (rows_ != s.rows_) || (cols_ != s.cols_) )
            return false;
        if( (alive_ != s.alive_) || (pos_ != s.pos_) || (narrows_ != s.narrows_) )
            return false;
        if( (heading_ != s.heading_) || (gold_ != s.gold_) )
            return false;
        //if( (npits_ != s.npits_) || (nwumpus_ != s.nwumpus_) ) return false;
        if( possible_gold_ != s.possible_gold_ )
            return false;
        if( *belief_ != *s.belief_ )
            return false;
        return true;
    }
    bool operator<(const state_t &s) const {
        assert(0); return false;
    }
    
};

int state_t::rows_ = 0;
int state_t::cols_ = 0;
//int state_t::npits_ = 0;
//int state_t::nwumpus_ = 0;
bool state_t::compass_ = false;

inline std::ostream& operator<<(std::ostream &os, const state_t &state) {
    state.print(os);
    return os;
}

#endif

