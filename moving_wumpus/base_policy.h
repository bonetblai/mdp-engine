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

template<typename T> class wumpus_distances_t {
    int n_move_actions_;
    bool compass_;
    mutable std::vector<int> distances_;

    mutable std::priority_queue<std::pair<int, int>,
                                std::vector<std::pair<int, int> >,
                                open_list_cmp> queue_;

    void _initialize_distances() const {
        int dim = compass_ ? nrows_ * ncols_ : 4 * nrows_ * ncols_;
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

    void _compute_distances(const T &state, bool safe) const {
        assert(!compass_);
        assert(!compass_ || ((int)distances_.size() == nrows_ * ncols_));
        assert(compass_ || ((int)distances_.size() == 4 * nrows_ * ncols_));
        while( !queue_.empty() ) {
            std::pair<int, int> p = queue_.top();
            queue_.pop();
            assert(p.second < INT_MAX);
            if( p.second <= distances_[p.first] ) {
                int node = p.first;
                int cell = compass_ ? node : (node >> 2);
                int heading = compass_ ? -1 : (node & 0x3);
                for( int action = 0; action < n_move_actions_; ++action ) {
                    // compute regression
                    int ncell = cell, nheading = heading;
                    if( action == ActionMoveForward ) {
                        ncell = target_cell(cell, (2 + heading) & 0x3, action, nrows_, ncols_, compass_);
                    } else {
                        nheading = target_heading(heading, action);
                        nheading = (2 + nheading) & 0x3;
                    }
                    if( state.hazard_at(ncell) ) continue;
                    if( safe && !state.no_hazard_at(ncell) ) continue;
                    int cost = p.second == INT_MAX ? p.second : 1 + p.second;
                    int new_node = compass_ ? ncell : ((ncell << 2) + nheading);
                    if( cost < distances_[new_node] ) {
#if 0
                        std::cout << "new: a=" << action
                                  << ", node=(" << (ncell % ncols_)
                                  << "," << (ncell / ncols_)
                                  << "," << heading_name(nheading)
                                  << "), parent=(" << (cell % ncols_)
                                  << "," << (cell / ncols_)
                                  << "," << heading_name(heading)
                                  << "), cost=" << cost << std::endl;
#endif
                        distances_[new_node] = cost;
                        queue_.push(std::make_pair(new_node, cost));
                    }
                }
            }
        }
        //print(std::cout);
    }

  protected:
    int nrows_;
    int ncols_;

  public:
    wumpus_distances_t(int nrows, int ncols, bool compass)
      : n_move_actions_(0), compass_(compass), nrows_(nrows), ncols_(ncols) {
        n_move_actions_ = compass_ ? 1 + ActionMoveWest : 1 + ActionTurnLeft;
    }
    ~wumpus_distances_t() { }

    int nrows() const { return nrows_; }
    int ncols() const { return ncols_; }
    int operator[](int i) const { return distances_[i]; }

    void compute_distances(const T &state, bool safe, const std::vector<int> &goals) const {
        _initialize_distances();
        _setup_goal_cells(goals);
        _compute_distances(state, safe);
    }

    void compute_distances(const T &state, bool safe, int goal) const {
        _initialize_distances();
        _setup_goal_cells(goal);
        _compute_distances(state, safe);
    }

    int heading(int from, int to) const {
        int from_row = from / ncols_, from_col = from % ncols_;
        int to_row = to / ncols_, to_col = to % ncols_;
        assert((from_row == to_row) || (from_col == to_col));
        if( from_row != to_row ) {
            return from_row < to_row ? North : South;
        } else {
            return from_col < to_col ? East : West;
        }
    }

    bool adjacent(int pos1, int pos2) const {
        int row1 = pos1 / ncols_, col1 = pos1 % ncols_;
        int row2 = pos2 / ncols_, col2 = pos2 % ncols_;
        if( (row1 != row2) && (col1 != col2) ) return false;
        return (abs(row1 - row2) == 1) || (abs(col1 - col2) == 1);
    }

    int movement(int pos1, int pos2) const {
        int row1 = pos1 / ncols_, col1 = pos1 % ncols_;
        int row2 = pos2 / ncols_, col2 = pos2 % ncols_;
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
        int dim = nrows_ * ncols_;
        if( !compass_ ) dim *= 4;
        for( int p = 0; p < dim; ++p )
            std::cout << " " << distances_[p];
        std::cout << std::endl;
    }
};


template<typename T> class __wumpus_base_policy_t {
  protected:
    bool compass_;
    wumpus_distances_t<T> distances_;

  public:
    __wumpus_base_policy_t(int nrows, int ncols, bool compass)
        : compass_(compass), distances_(nrows, ncols, compass) { }
    ~__wumpus_base_policy_t() { }

    int operator()(const T &state) const {
        if( state.have_gold() ) {
            // if have goal and at entry, just EXIT
            if( state.position() == 0 ) return ActionExit;

            // must go home (outside cave)
            distances_.compute_distances(state, 0, true);
            int min_dist = INT_MAX, best_cell = -1;
            for( int p = 0; p < distances_.nrows() * distances_.ncols(); ++p ) {
                if( distances_.adjacent(p, state.position()) && (distances_[p] < min_dist) ) {
                    min_dist = distances_[p];
                    best_cell = p;
                }
            }
            assert(min_dist < INT_MAX);
            assert(distances_[state.position()] == 1 + distances_[best_cell]);

            // move if right heading, else turn around
            if( compass_ ) {
                return distances_.movement(state.position(), best_cell);
            } else {
                if( state.heading() == distances_.heading(state.position(), best_cell) ) {
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


template<typename T> class wumpus_base_policy_t : public Policy::policy_t<T> {
  protected:
    int nrows_;
    int ncols_;
    bool compass_;
    __wumpus_base_policy_t<T> base_;

  public:
    wumpus_base_policy_t(const Problem::problem_t<T> &problem, int nrows, int ncols, bool compass)
      : Policy::policy_t<T>(problem),
        nrows_(nrows), ncols_(ncols), compass_(compass),
        base_(nrows_, ncols_, compass_) {
    }
    virtual ~wumpus_base_policy_t() { }

    virtual Problem::action_t operator()(const T &s) const {
        return base_(s);
    }
    virtual const Policy::policy_t<T>* clone() const {
        return new wumpus_base_policy_t<T>(Policy::policy_t<T>::problem(), nrows_, ncols_, compass_);
    }
    virtual void print_stats(std::ostream &os) const { }
};


template<typename T> struct shortest_distance_to_unvisited_cell_t : public Heuristic::heuristic_t<T> {
    const template_problem_t<T> &problem_;
    bool compass_;
    wumpus_distances_t<T> distances_;

  public:
    shortest_distance_to_unvisited_cell_t(const template_problem_t<T> &problem, bool compass)
      : problem_(problem), compass_(compass),
        distances_(problem_.nrows(), problem_.ncols(), compass) { }
    virtual ~shortest_distance_to_unvisited_cell_t() { }

    virtual float value(const T &s) const {
        if( s.dead() ) return 10000;
        if( s.have_gold() ) return 0;
        if( s.in_gold_cell() ) return 1;

        std::vector<int> goals;
        goals.reserve(problem_.nrows() * problem_.ncols());
        for( int p = 0; p < problem_.nrows() * problem_.ncols(); ++p ) {
            //std::cout << "shortest: p=" << p << ": p-gold=" << (s.possible_gold_at(p) ? 1 : 0) << ", hazard=" << (s.hazard_at(p) ? 1 : 0) << std::endl;
            if( s.possible_gold_at(p) && !s.hazard_at(p) ) {
                goals.push_back(p);
            }
        }
        distances_.compute_distances(s, false, goals);
        int node = compass_ ? s.position() : ((s.position() << 2) + s.heading());
        assert(distances_[node] != 0);

        // cost is distance + 1 that counts grab action
        //int value = distances_[node] == INT_MAX ? distances_[node] : 1 + distances_[node];
        int value = distances_[node] == INT_MAX ? 10000 : 1 + distances_[node];

#if 0
        std::cout << "heuristic: "
                  << "pos=(" << (s.position() % problem_.ncols())
                  << "," << (s.position() / problem_.ncols())
                  << "," << heading_name(s.heading())
                  << "), value=" << value
                  << std::endl;

        std::cout << "gold/hazard: ";
        for( int p = 0; p < problem_.nrows() * problem_.ncols(); ++p ) {
            std::cout << " " << (s.possible_gold_at(p) ? 1 : 0)
                      << "/" << (s.hazard_at(p) ? 1 : 0);
        }
        std::cout << std::endl;
        distances_.print(std::cout);
#endif

        return value;
    }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const T &s) const { return value(s); }
};

#endif

