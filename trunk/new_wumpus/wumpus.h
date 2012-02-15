#ifndef WUMPUS_H
#define WUMPUS_H

#include "agent.h"
#include "problem.h"
#include "policy.h"

#include <vector>
#include <set>

class problem_t : public Problem::problem_t<state_t> {
  protected:
    int rows_;
    int cols_;
    int npits_;
    int nwumpus_;
    int narrows_;
    const state_t init_;

  public:
    problem_t(int rows, int cols, int npits, int nwumpus, int narrows, float dead_end_value = 1e3)
      : Problem::problem_t<state_t>(1.0, dead_end_value),
        rows_(rows), cols_(cols),
        npits_(npits), nwumpus_(nwumpus), narrows_(narrows),
        init_(rows_, cols_, npits_, nwumpus_, narrows_) {
        const_cast<state_t&>(init_).set_as_unknown();
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions(const state_t &s) const {
        return 1 + EXIT;
    }
    virtual bool applicable(const state_t &s, Problem::action_t a) const {
        return s.applicable(a);
    }
    virtual const state_t& init() const {
        return init_;
    }
    virtual bool terminal(const state_t &s) const {
        return !s.in_cave();
    }
    virtual bool dead_end(const state_t &s) const {
        return s.dead();
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return (a == EXIT) && !s.have_gold() ? 1000 : 1;
    }
    virtual void next(const state_t &s,
                      Problem::action_t a,
                      std::vector<std::pair<state_t, float> > &outcomes) const {

        outcomes.reserve(8);
        for( int obs = 0; obs < 10; ++obs ) {
            state_t next(s);
            next.apply(a);
            if( (next.pos() == -1) && (obs > 0) ) continue;
            next.update(obs);
            if( !next.inconsistent() ) {
                std::pair<state_t, float> p(next, 1.0);
                outcomes.push_back(p);
                //outcomes.push_back(std::make_pair(next, 1.0));
            }
        }
        assert(!outcomes.empty());

        for( int i = 0, isz = outcomes.size(); i < isz; ++i ) {
            outcomes[i].second /= (float)isz;
        }
    }
    virtual void print(std::ostream &os) const { }
};

inline std::ostream& operator<<(std::ostream &os, const problem_t &p) {
    p.print(os);
    return os;
}







void place_random_objects(std::vector<int> &state, int rows, int cols, int nobjs, std::set<int> &forbidden) {
    state = std::vector<int>(rows * cols, 0);
    for( int i = 0; i < nobjs; ++i ) {
        int cell = Random::uniform(rows * cols);
        while( (state[cell] == 1) || (forbidden.find(cell) != forbidden.end()) ) {
            cell = Random::uniform(rows * cols);
        }
        state[cell] = 1;
    }
}

class hidden_state_t : public state_t {
    std::vector<int> pits_;
    std::vector<int> wumpus_;

  public:
    hidden_state_t(int rows, int cols, int npits, int nwumpus, int narrows)
      : state_t(rows, cols, npits, nwumpus, narrows) { }
    ~hidden_state_t() { }

    void sample() {
        alive_ = true;
        pos_ = 0;
        heading_ = NORTH;
        gold_ = Random::uniform(rows_ * cols_);

        std::set<int> forbidden;
        forbidden.insert(pos_);
        forbidden.insert(gold_);
        place_random_objects(pits_, rows_, cols_, npits_, forbidden);
        place_random_objects(wumpus_, rows_, cols_, nwumpus_, forbidden);
    }

    int get_obs() {
        if( pos_ == -1 ) return 0;
        int breeze = num_surrounding_objs(pits_);
        int stench = num_surrounding_objs(wumpus_);
        int glitter = gold_ == pos_ ? 1 : 0;
        if( breeze == 9 ) {
            return FELL;
        } else if( stench == 9 ) {
            return EATEN;
        } else {
            int obs = 0;
            obs += glitter > 0 ? GLITTER : 0;
            obs += breeze > 0 ? BREEZE : 0;
            obs += stench > 0 ? STENCH : 0;
            return obs;
        }
    }

    int apply_action_and_get_obs(int action) {
        apply(action);
        return get_obs();
    }

    int num_surrounding_objs(std::vector<int> &objs) const {
        if( objs[pos_] == 1 ) return 9;

        int row = pos_ / cols_;
        int col = pos_ % cols_;

        int num = 0;
        for( int drow = -1; drow < 2; ++drow ) {
            int nrow = row + drow;
            if( (nrow < 0) || (nrow >= rows_) ) continue;
            for( int dcol = -1; dcol < 2; ++dcol ) {
                if( (drow != 0) && (dcol != 0) ) continue;
                int ncol = col + dcol;
                if( (ncol < 0) || (ncol >= cols_) ) continue;
                num += objs[nrow * cols_ + ncol];
            }
        }
        assert((0 <= num) && (num < 9));
        return num;
    }
};







struct wumpus_base_policy_t : public Policy::policy_t<state_t> {
    int rows_;
    int cols_;
    __wumpus_base_policy_t base_;
    wumpus_base_policy_t(const Problem::problem_t<state_t> &problem, int rows, int cols)
      : Policy::policy_t<state_t>(problem),
        rows_(rows), cols_(cols), base_(rows, cols) {
    }
    virtual ~wumpus_base_policy_t() { }

    virtual Problem::action_t operator()(const state_t &s) const {
        return base_(s);
    }
    virtual const Policy::policy_t<state_t>* clone() const {
        return new wumpus_base_policy_t(problem(), rows_, cols_);
    }
    virtual void print_stats(std::ostream &os) const { }
};






















#endif

