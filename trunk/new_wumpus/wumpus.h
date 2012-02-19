#ifndef WUMPUS_H
#define WUMPUS_H

#include "agent.h"
#include "problem.h"
#include "policy.h"

#include <vector>
#include <set>

#define GOAL_IS_HAVE_GOLD

class problem_t : public Problem::problem_t<state_t> {
  protected:
    int rows_;
    int cols_;
    int npits_;
    int nwumpus_;
    int narrows_;
    const state_t init_;

  public:
    problem_t(int rows, int cols, int npits, int nwumpus, int narrows, float dead_end_value = 1e5)
      : Problem::problem_t<state_t>(1.0, dead_end_value),
        rows_(rows), cols_(cols),
        npits_(npits), nwumpus_(nwumpus), narrows_(narrows) {
        const_cast<state_t&>(init_).set_as_unknown();
#ifdef GOAL_IS_HAVE_GOLD
        std::cout << "problem_t::goal: HAVE_GOLD" << std::endl;
#else
        std::cout << "problem_t::goal: EXIT" << std::endl;
#endif
    }
    virtual ~problem_t() { }

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    int npits() const { return npits_; }
    int nwumpus() const { return nwumpus_; }
    int narrows() const { return narrows_; }

    virtual Problem::action_t number_actions(const state_t &s) const {
        return 1 + ActionExit;
    }
    virtual bool applicable(const state_t &s, Problem::action_t a) const {
#ifdef GOAL_IS_HAVE_GOLD
        return (a != ActionExit) && s.applicable(a);
#else
        return s.applicable(a);
#endif
    }
    virtual const state_t& init() const {
        return init_;
    }
    virtual bool terminal(const state_t &s) const {
#ifdef GOAL_IS_HAVE_GOLD
        return s.have_gold();
#else
        return !s.in_cave();
#endif
    }
    virtual bool dead_end(const state_t &s) const {
        return s.dead();
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return (a == ActionExit) && !s.have_gold() ? 1e4 : 1;
    }
    virtual void next(const state_t &s,
                      Problem::action_t a,
                      std::vector<std::pair<state_t, float> > &outcomes) const {

        assert(s.applicable(a));
        state_t next_a = s;
        next_a.apply(a);

        std::vector<std::pair<int, float> > possible_obs;
        possible_obs.reserve(10);
        for( int obs = 0; obs < 10; ++obs ) {
            if( next_a.possible_obs(obs) ) {
                possible_obs.push_back(std::make_pair(obs, 1));
            }
        }
        assert(!possible_obs.empty());

        outcomes.reserve(possible_obs.size());
        for( int i = 0, isz = possible_obs.size(); i < isz; ++i ) {
            int obs = possible_obs[i].first;
            float p = 1.0 / (float)possible_obs.size();
            state_t next_ao = next_a;
            next_ao.update(obs);
            assert(!next_ao.inconsistent());
            //std::cout << "obs=" << obs << " is consistent" << std::endl;
            //std::cout << "next_ao: " << next_ao;

            bool found = false;
            for( int j = 0, jsz = outcomes.size(); j < jsz; ++j ) {
                if( outcomes[j].first == next_ao ) {
                    found = true;
                    outcomes[j].second += p;
                    break;
                }
            }

            if( !found ) {
                outcomes.push_back(std::make_pair(next_ao, p));
            }
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
    hidden_state_t(int narrows) : state_t(narrows) { }
    ~hidden_state_t() { }

    void sample(int npits, int nwumpus) {
        alive_ = true;
        pos_ = 0;
        heading_ = North;
        gold_ = Random::uniform(rows_ * cols_);

        std::set<int> forbidden;
        forbidden.insert(pos_);
        forbidden.insert(gold_);
        place_random_objects(pits_, rows_, cols_, npits, forbidden);
        place_random_objects(wumpus_, rows_, cols_, nwumpus, forbidden);
    }

    int get_obs() {
        if( pos_ == OutsideCave ) return 0;
        int breeze = num_surrounding_objs(pits_);
        int stench = num_surrounding_objs(wumpus_);
        int glitter = gold_ == pos_ ? 1 : 0;
        if( breeze == 9 ) {
            return Fell;
        } else if( stench == 9 ) {
            return Eaten;
        } else {
            int obs = 0;
            obs += glitter > 0 ? Glitter : 0;
            obs += breeze > 0 ? Breeze : 0;
            obs += stench > 0 ? Stench : 0;
            return obs;
        }
    }

    int apply_action_and_get_obs(int action) {
        assert(alive_);
        apply(action);
        alive_ = (pos_ == OutsideCave) || (!pits_[pos_] && !wumpus_[pos_]);
        assert(alive_ || pits_[pos_] || wumpus_[pos_]);
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

#endif

