#ifndef BASE_POLICY_H
#define BASE_POLICY_H

#include "wumpus.h"
#include "problem.h"
#include "heuristic.h"

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

class wumpus_distances_t {
    int n_move_actions_;
    bool compass_;
    mutable std::vector<int> distances_;

    mutable std::priority_queue<std::pair<int, int>,
                                std::vector<std::pair<int, int> >,
                                open_list_cmp> queue_;

    void _initialize_distances() const {
        int dim = compass_ ? rows_ * cols_ : 4 * rows_ * cols_;
        distances_ = std::vector<int>(dim, INT_MAX);
    }

    void _setup_goal_cells(int cell) const {
        if( compass_ ) {
            distances_[cell] = 0;
            queue_.push(std::make_pair(cell, 0));
        } else {
            for( int heading = 0; heading < 4; ++heading ) {
                int node = (cell << 2) + heading;
                distances_[node] = 0;
                queue_.push(std::make_pair(node, 0));
            }
        }
    }

    void _setup_goal_cells(const std::vector<int> &cells) const {
        for( int i = 0, isz = cells.size(); i < isz; ++i  )
            _setup_goal_cells(cells[i]);
    }

    void _compute_distances(const state_t &state, bool safe, bool compass) const {
        assert(!compass || ((int)distances_.size() == rows_ * cols_));
        assert(compass || ((int)distances_.size() == 4 * rows_ * cols_));
        while( !queue_.empty() ) {
            std::pair<int, int> p = queue_.top();
            queue_.pop();
            if( p.second <= distances_[p.first] ) {
                int node = p.first;
                int cell = compass ? node : (node >> 2);
                int heading = compass ? -1 : (node & 0x3);
                for( int action = 0; action < n_move_actions_; ++action ) {
                    int ncell = target_cell(cell, heading, action, rows_, cols_, compass_);
                    int nheading = target_heading(heading, action);
                    nheading = (2 + nheading) & 0x3; // compute regression
                    //if( action == 0 ) {
                    //    std::cout << "c=(" << (cell % cols_) << "," << (cell / cols_) << ")"
                    //              << ",h=" << heading << ",a=" << action << "  -->  "
                    //              << "c=(" << (ncell % cols_) << "," << (ncell / cols_) << ")"
                    //              << ",h=" << nheading << std::endl;
                    //}
                    if( state.hazard_at(ncell) ) continue;
                    if( safe && !state.no_hazard_at(ncell) ) continue;
                    int cost = p.second == INT_MAX ? p.second : 1 + p.second;
                    int new_node = compass ? ncell : ((ncell << 2) + nheading);
                    if( cost < distances_[new_node] ) {
                        //std::cout << "new: a=" << action
                        //          << ", node=(" << (ncell % cols_) << "," << (ncell / cols_) << ")"
                        //          << ", cost=" << cost << std::endl;
                        distances_[new_node] = cost;
                        queue_.push(std::make_pair(new_node, cost));
                    }
                }
            }
        }
        //print(std::cout);
    }

    void _compute_distances(const state_t &state, bool safe) const {
        _compute_distances(state, safe, compass_);
    }

  protected:
    int rows_;
    int cols_;

  public:
    wumpus_distances_t(int rows, int cols, bool compass)
      : n_move_actions_(0), compass_(compass), rows_(rows), cols_(cols) {
        n_move_actions_ = compass_ ? 1 + ActionMoveWest : 1 + ActionTurnLeft;
    }
    ~wumpus_distances_t() { }

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    int operator[](int i) const { return distances_[i]; }

    void compute_distances(const state_t &state, bool safe, const std::vector<int> &goals) const {
        _initialize_distances();
        _setup_goal_cells(goals);
        _compute_distances(state, safe);
    }

    void compute_distances(const state_t &state, bool safe, int goal) const {
        _initialize_distances();
        _setup_goal_cells(goal);
        _compute_distances(state, safe);
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

    int movement(int pos1, int pos2) const {
        int row1 = pos1 / cols_, col1 = pos1 % cols_;
        int row2 = pos2 / cols_, col2 = pos2 % cols_;
        assert((row1 == row2) || (col1 == col2));
        assert(pos1 != pos2);
        if( row1 == row2 ) {
            return col2 > col1 ? ActionMoveEast : ActionMoveWest;
        } else {
            return row2 > row1 ? ActionMoveNorth : ActionMoveSouth;
        }
    }

    void print(std::ostream &os) const {
        std::cout << "distances:";
        int dim = rows_ * cols_;
        if( !compass_ ) dim *= 4;
        for( int p = 0; p < dim; ++p )
            std::cout << " " << distances_[p];
        std::cout << std::endl;
    }
};


class __wumpus_base_policy_t {
  protected:
    bool compass_;
    wumpus_distances_t distances_;

  public:
    __wumpus_base_policy_t(int rows, int cols, bool compass)
        : compass_(compass), distances_(rows, cols, compass) { }
    ~__wumpus_base_policy_t() { }

    int operator()(const state_t &state) const {
        if( state.have_gold() ) {
            // if have goal and at entry, just EXIT
            if( state.pos() == 0 ) return ActionExit;

            // must go home (outside cave)
            distances_.compute_distances(state, 0, true);
            int min_dist = INT_MAX, best_cell = -1;
            for( int p = 0; p < distances_.rows() * distances_.cols(); ++p ) {
                if( distances_.adjacent(p, state.pos()) && (distances_[p] < min_dist) ) {
                    min_dist = distances_[p];
                    best_cell = p;
                }
            }
            assert(min_dist < INT_MAX);
            assert(distances_[state.pos()] == 1 + distances_[best_cell]);

            // move if right heading, else turn around
            if( compass_ ) {
                return distances_.movement(state.pos(), best_cell);
            } else {
                if( state.heading() == distances_.heading(state.pos(), best_cell) ) {
                    return ActionMoveForward;
                } else {
                    return ActionTurnRight;
                }
            }
        } else {
            // move around safely
            std::vector<int> actions;
            actions.reserve(6);
            for( int action = 0; action <= ActionExit; ++action ) {
                if( state.applicable(action) ) {
                    int ncell = state.target_cell(action);
                    if( state.no_hazard_at(ncell) )
                        actions.push_back(action);
                }
            }
            assert(!actions.empty());
            int action = actions[Random::uniform(actions.size())];
            assert(state.no_hazard_at(state.target_cell(action)));
            return action;
        }
    }

};


class wumpus_base_policy_t : public Policy::policy_t<state_t> {
  protected:
    int rows_;
    int cols_;
    bool compass_;
    __wumpus_base_policy_t base_;

  public:
    wumpus_base_policy_t(const Problem::problem_t<state_t> &problem, int rows, int cols, bool compass)
      : Policy::policy_t<state_t>(problem),
        rows_(rows), cols_(cols), compass_(compass),
        base_(rows_, cols_, compass_) {
    }
    virtual ~wumpus_base_policy_t() { }

    virtual Problem::action_t operator()(const state_t &s) const {
        return base_(s);
    }
    virtual const Policy::policy_t<state_t>* clone() const {
        return new wumpus_base_policy_t(problem(), rows_, cols_, compass_);
    }
    virtual void print_stats(std::ostream &os) const { }
};


struct shortest_distance_to_unvisited_cell_t : public Heuristic::heuristic_t<state_t> {
    const problem_t &problem_;
    bool compass_;
    wumpus_distances_t distances_;

  public:
    shortest_distance_to_unvisited_cell_t(const problem_t &problem, bool compass)
      : problem_(problem), compass_(compass),
        distances_(problem_.rows(), problem_.cols(), compass) { }
    virtual ~shortest_distance_to_unvisited_cell_t() { }

    virtual float value(const state_t &s) const {
        if( s.have_gold() ) return 0;
        if( s.in_gold_cell() || (s.n_possible_gold_places() == 0) ) return 1;

        std::vector<int> goals;
        goals.reserve(problem_.rows() * problem_.cols());
        for( int p = 0; p < problem_.rows() * problem_.cols(); ++p ) {
            if( s.possible_gold(p) && s.no_hazard_at(p) )
                goals.push_back(p);
        }
        distances_.compute_distances(s, true, goals);
        int node = compass_ ? s.pos() : ((s.pos() << 2) + s.heading());
        assert(distances_[node] != 0);

        std::cout << "pos=(" << (s.pos() % problem_.cols()) << "," << (s.pos() / problem_.cols()) << ")"
                  << ", heading=" << s.heading() << std::endl;
        std::cout << "gold: ";
        for( int p = 0; p < problem_.rows() * problem_.cols(); ++p )
            std::cout << " " << (s.possible_gold(p) ? 1 : 0);
        std::cout << std::endl;
        distances_.print(std::cout);

        return 1 + distances_[node];
    }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const state_t &s) const { return value(s); }
};

#endif

