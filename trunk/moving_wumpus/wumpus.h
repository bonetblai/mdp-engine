#ifndef WUMPUS_H
#define WUMPUS_H

#include "agent.h"
#include "problem.h"
#include "policy.h"

#include <vector>
#include <set>

#define GOAL_IS_HAVE_GOLD   0
#define GOAL_IS_EXIT        1

template<typename T> class template_problem_t : public Problem::problem_t<T> {
  protected:
    int nrows_;
    int ncols_;
    int npits_;
    int nwumpus_;
    int narrows_;
    int goal_type_;
    const T init_;

  public:
    template_problem_t(int nrows, int ncols, int npits, int nwumpus, int narrows, int goal_type = GOAL_IS_HAVE_GOLD, float dead_end_value = 1e5)
      : Problem::problem_t<T>(1.0, dead_end_value),
        nrows_(nrows), ncols_(ncols), npits_(npits), nwumpus_(nwumpus), narrows_(narrows),
        goal_type_(goal_type) {
        const_cast<T&>(init_).set_as_unknown();
        if( goal_type_ == GOAL_IS_HAVE_GOLD )
            std::cout << "template_problem_t::goal: HAVE_GOLD" << std::endl;
        else
            std::cout << "template_problem_t::goal: EXIT" << std::endl;
    }
    virtual ~template_problem_t() { }

    int nrows() const { return nrows_; }
    int ncols() const { return ncols_; }
    int npits() const { return npits_; }
    int nwumpus() const { return nwumpus_; }
    int narrows() const { return narrows_; }

    virtual Problem::action_t number_actions(const T &s) const {
        return 1 + ActionExit;
    }
    virtual bool applicable(const T &s, Problem::action_t a) const {
        return goal_type_ == GOAL_IS_HAVE_GOLD ? (a != ActionExit) && s.applicable(a) : s.applicable(a);
    }
    virtual const T& init() const {
        return init_;
    }
    virtual bool terminal(const T &s) const {
        return goal_type_ == GOAL_IS_HAVE_GOLD ? s.have_gold() : !s.in_cave();
    }
    virtual bool dead_end(const T &s) const {
        return s.dead();
    }
    virtual float cost(const T &s, Problem::action_t a) const {
        return (a == ActionExit) && !s.have_gold() ? 1e4 : 1;
    }

    virtual void next(const T &s, Problem::action_t a, std::vector<std::pair<T, float> > &outcomes) const {

        //std::cout << "action=" << a << std::endl;
        //std::cout << "state=" << s;
        assert(s.applicable(a));
        T next_a = s;
        next_a.apply(a);
        //std::cout << "next_state=" << next_a;

        int possible_obs[208], nobs = 0;
        //int possible_obs[80], nobs = 0;
        for( int obs = 0; obs < 208; ++obs ) {
            if( next_a.possible_obs(a, obs) ) {
                possible_obs[nobs++] = obs;
            }
        }
        assert(nobs > 0);

        outcomes.clear();
        outcomes.reserve(nobs);
        float p = 1.0 / (float)nobs;
        for( int i = 0; i < nobs; ++i ) {
            int obs = possible_obs[i];
            T next_ao = next_a;
            next_ao.update(a, obs);
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

template<typename T> inline std::ostream& operator<<(std::ostream &os, const template_problem_t<T> &p) {
    p.print(os);
    return os;
}

// template instantiation
typedef template_problem_t<state_t> problem_t;
typedef template_problem_t<m_state_t> m_problem_t;



void place_random_objects(std::vector<int> &state, int nrows, int ncols, int nobjs, std::set<int> &forbidden) {
    state = std::vector<int>(nrows * ncols, 0);
    for( int i = 0; i < nobjs; ++i ) {
        int cell = Random::uniform(nrows * ncols);
        while( (state[cell] == 1) || (forbidden.find(cell) != forbidden.end()) ) {
            cell = Random::uniform(nrows * ncols);
        }
        state[cell] = 1;
    }
}



class hidden_state_t {
    int nrows_;
    int ncols_;
    int npits_;
    int nwumpus_;
    int narrows_;

    int pos_;
    int heading_;
    int narrows_left_;
    bool have_gold_;
    bool dead_;
    int gold_pos_;
    std::vector<int> pits_;
    std::vector<int> wumpus_;

  public:
    hidden_state_t(int nrows, int ncols, int npits, int nwumpus, int narrows)
      : nrows_(nrows), ncols_(ncols), npits_(npits), nwumpus_(nwumpus), narrows_(narrows),
        pos_(0), heading_(North), narrows_left_(narrows_), have_gold_(false), dead_(false),
        gold_pos_(-1) { }
    ~hidden_state_t() { }

    int position() const { return pos_; }
    int heading() const { return heading_; }
    bool have_gold() const { return have_gold_; }
    bool dead() const { return dead_; }

    void sample(int pos, int heading, int npits, int nwumpus) {
        pos_ = pos;
        heading_ = heading;
        have_gold_ = false;
        dead_ = false;
        gold_pos_ = Random::uniform(nrows_ * ncols_);
        std::cout << "gold-pos=(" << (gold_pos_ % ncols_) << "," << (gold_pos_ / ncols_) << ")" << std::endl;

        std::set<int> forbidden;
        forbidden.insert(pos_);
        forbidden.insert(gold_pos_);
        pits_ = std::vector<int>(nrows_ * ncols_, 0);
        wumpus_ = std::vector<int>(nrows_ * ncols_, 0);
        place_random_objects(pits_, nrows_, ncols_, npits, forbidden);
        place_random_objects(wumpus_, nrows_, ncols_, nwumpus, forbidden);
    }

    int get_obs() {
        if( pos_ == OutsideCave ) return 0;
        int breeze = num_surrounding_objs(pits_);
        int stench = num_surrounding_objs(wumpus_);
        int glitter = gold_pos_ == pos_ ? 1 : 0;
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

    bool applicable(int action) const { assert(0); }
    void apply(int action) { assert(0); }

    int apply_action_and_get_obs(int action) {
        assert(!dead_);
        apply(action);
        dead_ = (pos_ != OutsideCave) && (pits_[pos_] || wumpus_[pos_]);
        return get_obs();
    }

    int num_surrounding_objs(std::vector<int> &objs) const {
        if( objs[pos_] == 1 ) return 9;

        int row = pos_ / ncols_;
        int col = pos_ % ncols_;

        int num = 0;
        for( int drow = -1; drow < 2; ++drow ) {
            int nrow = row + drow;
            if( (nrow < 0) || (nrow >= nrows_) ) continue;
            for( int dcol = -1; dcol < 2; ++dcol ) {
                if( (drow != 0) && (dcol != 0) ) continue;
                int ncol = col + dcol;
                if( (ncol < 0) || (ncol >= ncols_) ) continue;
                num += objs[nrow * ncols_ + ncol];
            }
        }
        assert((0 <= num) && (num < 9));
        return num;
    }
};

#endif

