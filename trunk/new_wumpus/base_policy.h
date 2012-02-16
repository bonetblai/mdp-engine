#ifndef BASE_POLICY_H
#define BASE_POLICY_H

#include "agent.h"

#include <cassert>
#include <iostream>
#include <queue>
#include <stdlib.h>
#include <limits.h>

struct open_list_cmp {
    bool operator()(const std::pair<int, int> &p1, const std::pair<int, int> &p2) {
        return p1.second > p2.second;
    }
};

struct __wumpus_base_policy_t {
    int rows_;
    int cols_;
    mutable std::vector<int> distances_;

    __wumpus_base_policy_t(int rows, int cols) : rows_(rows), cols_(cols) { }
    ~__wumpus_base_policy_t() { }

    int operator()(const state_t &state) const {
        if( state.have_gold() ) {
            // if have goal and at entry, just EXIT
            if( state.pos() == 0 ) return Exit;

            // must go home (outside cave)
            compute_distances(state, 0, true);
            int min_dist = INT_MAX, best_cell = -1;
            for( int p = 0; p < rows_ * cols_; ++p ) {
                if( adjacent(p, state.pos()) && (distances_[p] < min_dist) ) {
                    min_dist = distances_[p];
                    best_cell = p;
                }
            }
            assert(min_dist < INT_MAX);
            assert(distances_[state.pos()] == 1 + distances_[best_cell]);

            // move if right heading, else turn around
#ifdef COMPASS_ACTIONS
            return movement(state.pos(), best_cell);
#else
            if( state.heading() == heading(state.pos(), best) ) {
                return Move;
            } else {
                return TurnR;
            }
#endif
        } else {
            // move around safely
            std::vector<int> actions;
            actions.reserve(6);
            for( int action = 0; action <= Exit; ++action ) {
                if( state.applicable(action) ) {
                    int target_cell = state.target_cell(action);
                    if( state.no_hazard_at(target_cell) )
                        actions.push_back(action);
                }
            }
            if( actions.size() == 0 ) std::cout << "state: " << state;
            assert(!actions.empty());
            int action = actions[Random::uniform(actions.size())];
            assert(state.no_hazard_at(state.target_cell(action)));
            return action;
        }
    }

    void compute_distances(const state_t &state, int goal, bool safe) const {
        distances_ = std::vector<int>(rows_ * cols_, INT_MAX);

        std::priority_queue<std::pair<int, int>,
                            std::vector<std::pair<int, int> >,
                            open_list_cmp> queue;

        distances_[goal] = 0;
        queue.push(std::make_pair(goal, 0));
        while( !queue.empty() ) {
            std::pair<int, int> p = queue.top();
            queue.pop();
            if( p.second <= distances_[p.first] ) {
                int cell = p.first;
                int row = cell / cols_, col = cell % cols_;
                for( int dr = -1; dr < 2; ++dr ) {
                    if( (row + dr < 0) || (row + dr >= rows_) ) continue;
                    for( int dc = -1; dc < 2; ++dc ) {
                        if( (dr != 0) && (dc != 0) ) continue;
                        if( (col + dc < 0) || (col + dc >= cols_) ) continue;
                        int ncell = (row + dr) * cols_ + (col + dc);
                        if( state.hazard_at(ncell) ) continue;
                        if( safe && !state.no_hazard_at(ncell) ) continue;

                        int cost = 1 + p.second;
                        if( cost < distances_[ncell] ) {
                            distances_[ncell] = cost;
                            queue.push(std::make_pair(ncell, cost));
                        }
                    }
                }
            }
        }
        //std::cout << "distances:";
        //for( int p = 0; p < rows_ * cols_; ++p )
        //    std::cout << " " << distances_[p];
        //std::cout << std::endl;
    }

    int heading(int from, int to) const {
        int from_row = from / cols_, from_col = from % cols_;
        int to_row = to / cols_, to_col = to % cols_;
        assert((from_row == to_row) || (from_col == to_col));
        if( from_row != to_row ) {
            return from_row < to_row ? North : South;
        } else {
            return from_col < to_col ? East : West;
        }
    }

    bool adjacent(int pos1, int pos2) const {
        int row1 = pos1 / cols_, col1 = pos1 % cols_;
        int row2 = pos2 / cols_, col2 = pos2 % cols_;
        if( (row1 != row2) && (col1 != col2) ) return false;
        return (abs(row1 - row2) == 1) || (abs(col1 - col2) == 1);
    }

#ifdef COMPASS_ACTIONS
    int movement(int pos1, int pos2) const {
        int row1 = pos1 / cols_, col1 = pos1 % cols_;
        int row2 = pos2 / cols_, col2 = pos2 % cols_;
        assert((row1 == row2) || (col1 == col2));
        assert(pos1 != pos2);
        if( row1 == row2 ) {
            return col2 > col1 ? MoveEast : MoveWest;
        } else {
            return row2 > row1 ? MoveNorth : MoveSouth;
        }
    }
#endif

};

#endif

