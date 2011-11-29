#include <cassert>
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <limits>
#include <strings.h>

//#define  DISCOUNT  .95
#define  DISCOUNT  1

#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "mcts.h"
#include "dispatcher.h"

#include "theory.h"

#define FLAG_CELL 0
#define OPEN_CELL 1
#define OBS_0     0
#define OBS_1     1
#define OBS_2     2
#define OBS_3     3
#define OBS_4     4
#define OBS_5     5
#define OBS_6     6
#define OBS_7     7
#define OBS_8     8
#define OBS_DEAD  9
#define OBS_FLAG 10

theory_t *theory = 0;

std::vector<float> default_probabilities_;
std::vector<float> tag_probability_;
std::vector<std::vector<int> > supported_obs_;
std::vector<std::vector<int> > refuted_obs_;

inline int pcell_2_cnfcell(int cell) {
    int c = cell % 2, r = cell / 2;
    return (r+1) * (2 + 2) + (c+1);
}

inline int cnfcell_2_pcell(int cell) {
    int c = cell % (2 + 2), r = cell / (2 + 2);
    return (r-1) * 2 + (c-1);
}

inline int valid_pcell(int cell) {
    int c = cell % (2 + 2), r = cell / (2 + 2);
    return (r > 0) && (r < (2 + 2) - 1) && (c > 0) && (c < (2 + 2) - 1);
}

struct state_t {
    int nbombs_;
    bool dead_;
    std::vector<unsigned char> obs_;
    std::vector<float> prob_obs_;
    std::set<int> literals_;
 
  public:
    state_t(int size = 0, int nbombs = 0)
      : nbombs_(nbombs), dead_(false) {
        obs_ = std::vector<unsigned char>(size, 0xFF);
        prob_obs_ = std::vector<float>(10 * size, 0);
        initial_preprocess();
    }
    state_t(const state_t &s) { *this = s; }
    ~state_t() { }

    size_t hash() const { return 0; } // TODO

    bool terminal() const { return nbombs_ == 0; }
    bool is_dead_end() const { return dead_; }
    bool applicable(int act, int cell) const {
        int index = cell >> 1;
        int shift = cell & 1 ? 4 : 0;
        int obs = (obs_[index] >> shift) & 0xF;
        return obs == 0xF;
    }

    float probability(int act, int cell, int obs) const {
        assert(act == OPEN_CELL);
        assert(obs != OBS_FLAG);
        return prob_obs_[10*cell + obs];
    }
    void apply(int act, int cell, int obs, const std::vector<int> *refuted_tags = 0) {
        dead_ = dead_ || (obs == OBS_DEAD);
        nbombs_ += act == FLAG_CELL ? -1 : 0;
        int index = cell >> 1;
        int shift = cell & 1 ? 4 : 0;
        int mask = cell & 1 ? 0xF : 0XF0;
        obs_[index] = (obs_[index] & mask) | (obs << shift);
        if( act == OPEN_CELL ) {
            //std::cout << "Open: (cell=" << cell << ",obs=" << obs << ")" << std::endl;
            for( unsigned i = 0, isz = refuted_tags->size(); i < isz; ++i ) {
                int tag = 257 * pcell_2_cnfcell(cell) + (*refuted_tags)[i];
                literals_.insert(-(1+tag));
                //std::cout << "  force literal: -(cell=" << cell << ",t=" << (*refuted_tags)[i] << ")=" << -(1+tag) << std::endl;
            }
        }
        incremental_preprocess();
    }
    float cost(int act, int cell) const {
        float p = probability(OPEN_CELL, cell, OBS_DEAD);
        return act == OPEN_CELL ? 1*(1-p) + 1000*p : 1000*(1-p) + 1*p;
    }

    const state_t& operator=(const state_t &s) {
        nbombs_ = s.nbombs_;
        dead_ = s.dead_;
        obs_ = s.obs_;
        prob_obs_ = s.prob_obs_;
        literals_ = s.literals_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return (nbombs_ == s.nbombs_) && (dead_ == s.dead_) && (obs_ == s.obs_);
    }
    bool operator!=(const state_t &s) const { return !(*this == s); }
    bool operator<(const state_t &s) const {
        if( (nbombs_ < s.nbombs_) || ((nbombs_ == s.nbombs_) && !dead_ && s.dead_) ) {
            return true;
        } else if( (nbombs_ == s.nbombs_) && (dead_ == s.dead_) ) {
            assert(obs_.size() == s.obs_.size());
            for( unsigned i = 0; i < obs_.size(); ++i ) {
                if( obs_[i] < s.obs_[i] ) {
                    return true;
                } else if( obs_[i] > s.obs_[i] ) {
                    return false;
                }
            }
            return false;
        } else {
            return false;
        }
    }
    void print(std::ostream &os) const {
        os << "(" << nbombs_
           << ",<";
        for( unsigned i = 0; i < obs_.size(); ++i ) {
            if( (obs_[i] & 0xF) != 0xF )
                os << 2*i << ":" << (obs_[i] & 0xF) << ",";
            if( (obs_[i] >> 4) != 0xF )
                os << 2*i+1 << ":" << (obs_[i] >> 4) << ",";
        }
        os << ">)";
    }

    void set_default_probabilities() {
        bcopy(&default_probabilities_[0], &prob_obs_[0], sizeof(float) * prob_obs_.size());
    }

    void initial_preprocess() {
        //std::cout << "begin: initial preprocess" << std::endl;
        set_default_probabilities();
        incremental_preprocess();
        //std::cout << "end: initial preprocess" << std::endl;
    }

    void incremental_preprocess() {
        //std::cout << "begin: incremental preprocess: " << literals_.size() << std::endl;

        std::set<int> new_literals;
        theory->deduction(literals_, new_literals);
        literals_.insert(new_literals.begin(), new_literals.end());

        // adjust probabilities given new implied literals: first process
        // negative literals that reduce support and then positive literals
        // that assert observations-

        // negative literals: decrease support
        for( std::set<int>::const_iterator it = new_literals.begin(); it != new_literals.end(); ++it ) {
            int lit = *it;
            assert(lit != 0);
            int tag = lit < 0 ? -lit - 1 : lit - 1;
            if( valid_pcell(tag / 257) ) {
                int cell = cnfcell_2_pcell(tag / 257);
                int t = tag % 257;
                if( lit < 0 ) {
                    float p = tag_probability_[t];
                    //std::cout << "reducing support: t=" << t << ", p=" << p << std::endl;
                    assert(p > 0);
                    for( unsigned i = 0, isz = supported_obs_[t].size(); i < isz; ++i ) {
                        int n = supported_obs_[t][i];
                        prob_obs_[10*cell + n] -= p;
                        //std::cout << "    (cell=" << cell << ",n=" << n << "), tag=" << t << ", new-p=" << prob_obs_[10*cell + n] << std::endl;
                        assert(prob_obs_[10*cell + n] >= 0);
                    }
                } else {
                }
            }
        }

#if 0
        // positive literals: assert observations
        for( std::set<int>::const_iterator it = new_literals.begin(); it != new_literals.end(); ++it ) {
            int lit = *it;
            assert(lit != 0);
            if( lit > 0 ) {
                int tag = lit - 1;
                int cell = tag / 257;
                int t = tag % 257;
                for( unsigned i = 0, isz = supported_obs_[t].size(); i < isz; ++i ) {
                    int obs = supported_obs_[t][i];
                    prob_obs_[10*cell + obs] = 1;
                    std::cout << "set support: obs=" << obs << ", new-p=1.0" << std::endl;
                }
                for( unsigned i = 0, isz = refuted_obs_[t].size(); i < isz; ++i ) {
                    int obs = refuted_obs_[t][i];
                    prob_obs_[10*cell + obs] = 0;
                    std::cout << "set support: obs=" << obs << ", new-p=0.0" << std::endl;
                }
            }
        }
#endif

        //std::cout << "end: incremental preprocess" << std::endl;
    }
};

inline std::ostream& operator<<(std::ostream &os, const state_t &s) {
    s.print(os);
    return os;
}

class problem_t : public Problem::problem_t<state_t> {
  public:
    int size_;
    int nbombs_;
    state_t *init_;
    std::vector<std::vector<int> > refuted_tags_;
    std::vector<std::vector<int> > support_tags_;

  public:
    problem_t(int size, int nbombs)
      : size_(size), nbombs_(nbombs), init_(0) {

        // refuted/support tags for observations
        refuted_tags_ = std::vector<std::vector<int> >(10, std::vector<int>());
        support_tags_ = std::vector<std::vector<int> >(10, std::vector<int>());
        supported_obs_ = std::vector<std::vector<int> >(257, std::vector<int>());
        refuted_obs_ = std::vector<std::vector<int> >(257, std::vector<int>());
        for( int t = 0; t < 256; ++t ) {
            refuted_tags_[9].push_back(t);
            int nbits_in_tag = 0;
            for( int aux = t; aux != 0; aux = aux >> 1 ) {
                if( (aux & 1) != 0 ) ++nbits_in_tag;
            }
            assert(nbits_in_tag <= 8);
            support_tags_[nbits_in_tag].push_back(t);
            supported_obs_[t].push_back(nbits_in_tag);
            for( int obs = 0; obs < 9; ++obs ) {
                if( obs != nbits_in_tag ) {
                    refuted_tags_[obs].push_back(t);
                }
            }
        }

        support_tags_[9].push_back(256);
        supported_obs_[256].push_back(9);
        for( int obs = 0; obs < 8; ++obs ) {
            refuted_tags_[obs].push_back(256);
        }

        // set (global) default probabilities
        default_probabilities_ = std::vector<float>(10 * size_, 0);
        for( int p = 0; p < size_; ++p ) {
            for( int obs = 0; obs < 9; ++obs )
                default_probabilities_[10*p + obs] = (float)support_tags_[obs].size() / 512.0;
            default_probabilities_[10*p + 9] = 0.5;
        }

        // set (global) tag probability
        tag_probability_ = std::vector<float>(257, 0);
        for( int t = 0; t < 256; ++t ) tag_probability_[t] = 1.0 / 512.0;
        tag_probability_[256] = 0.5;

        // set (global) supported/refuted obs

        // create logical theory
        theory = new theory_t(2, 2, nbombs_);
        theory->create_manager();

        // create initial state
        init_ = new state_t(size_, nbombs_);
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions(const state_t &s) const {
        return 2*size_; // for each cell, either open it or put a flag
    }
    virtual bool applicable(const state_t &s, Problem::action_t a) const {
        return s.applicable(a & 1, a >> 1);
    }
    virtual const state_t& init() const { return *init_; }
    virtual bool terminal(const state_t &s) const { return s.terminal(); }
    virtual bool dead_end(const state_t &s) const { return s.is_dead_end(); }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return s.cost(a & 1, a >> 1);
    }
    virtual void next(const state_t &s, Problem::action_t a, std::vector<std::pair<state_t, float> > &outcomes) const {
        std::cout << "next" << s << " w/ a=" << a << " is:" << std::endl;

        ++expansions_;
        outcomes.clear();

        int cell = a >> 1;
        int act = a & 1;

        float mass = 0;
        if( act == FLAG_CELL ) {
            // this is a flag action, it has only 1 outcome
            outcomes.reserve(1);
            state_t next = s;
            next.apply(act, cell, OBS_FLAG);
            outcomes.push_back(std::make_pair(next, 1.0));
            std::cout << "    " << next << " w.p. " << 1.0 << std::endl;
            mass += 1;
        } else {
            // this is an open action, it has 10 possible outcomes:
            // observing a number between 0..8 and DEAD. The possible
            // outcomes are those with non-zero probability
            outcomes.reserve(10);
            for( int obs = 0; obs < 10; ++obs ) {
                float p = s.probability(act, cell, obs);
                if( p > 0 ) {
                    state_t next = s;
                    //std::cout << "obs=" << obs << ", rt.sz=" << refuted_tags_[obs].size() << std::endl;
                    next.apply(act, cell, obs, &refuted_tags_[obs]);
                    outcomes.push_back(std::make_pair(next, p));
                    std::cout << "    " << next << " w.p. " << p << std::endl;
                    mass += p;
                }
            }
        }
        std::cout << "done with expansion: mass=" << mass << std::endl;
        assert(!outcomes.empty());
        assert(mass == 1.0);
    }
    virtual void print(std::ostream &os) const { }
};

inline std::ostream& operator<<(std::ostream &os, const problem_t &p) {
    p.print(os);
    return os;
}

#if 0
class min_min_t : public Heuristic::heuristic_t<state_t> {
  public:
    min_min_t() { }
    virtual ~min_min_t() { }
    virtual float value(const state_t &s) const { return (float)s.heuristic_; }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const state_t &s) const { return value(s); }
};
#endif

