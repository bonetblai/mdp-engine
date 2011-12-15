#include <cassert>
#include <cstring>
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <limits>

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

#define INNER     0
#define CORNER1   1
#define CORNER2   2
#define CORNER3   3
#define CORNER4   4
#define SIDE1     5
#define SIDE2     6
#define SIDE3     7
#define SIDE4     8
#define NTYPES    9

theory_t *theory = 0;

std::vector<float> default_probabilities_;
std::vector<float> tag_probability_[NTYPES];
std::vector<std::vector<int> > supported_obs_[NTYPES];
std::list<std::pair<int, std::vector<int> > > initially_refuted_tags_;

inline int cell_type(int cell) {
    int c = cell % 2, r = cell / 2;
    if( (r > 0) && (r < 2 - 1) && (c > 0) && (c < 2 - 1) ) {
        return INNER;
    } else if( (r == 0) || (r == 2 - 1) ) {
        if( c == 0 ) {
            return r == 0 ? CORNER1 : CORNER3;
        } else if( c == 2 - 1 ) {
            return r == 0 ? CORNER2 : CORNER4;
        } else {
            return r == 0 ? SIDE1 : SIDE4;
        }
    } else {
        assert((c == 0) || (c == 2 - 1));
        return c == 0 ? SIDE2 : SIDE3;
    }
}

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
        if( (act == FLAG_CELL) && (nbombs_ == 0) ) return false;
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
        //std::cout << "computing apply for obs=" << obs << std::endl;
        incremental_preprocess(cell, refuted_tags);
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
            if( (obs_[i] & 0xF) != 0xF ) {
                int obs = obs_[i] & 0xF;
                 char c_obs = obs == 10 ? 'F' : (obs == 9 ? 'X' : (char)(obs+'0')); 
                os << 2*i << ":" << c_obs << ",";
            }
            if( (obs_[i] >> 4) != 0xF ) {
                int obs = obs_[i] >> 4;
                 char c_obs = obs == 10 ? 'F' : (obs == 9 ? 'X' : (char)(obs+'0')); 
                //os << 2*i+1 << ":" << (char)(obs+'0') << ",";
                os << 2*i+1 << ":" << c_obs << ",";
            }
        }
        os << ">";
        if( dead_ ) os << ",DE";
        os << ")";
    }

    void set_default_probabilities() {
        memcpy(&prob_obs_[0], &default_probabilities_[0], sizeof(float) * prob_obs_.size());
    }

    void initial_preprocess() {
        //std::cout << "begin: initial preprocess" << std::endl;
        set_default_probabilities();

#if 0
        std::cout << "dump1:" << std::endl;
        for( int cell = 0, sz = prob_obs_.size() / 10; cell < sz; ++cell ) {
            for( int obs = 0; obs < 10; ++obs ) {
                if( prob_obs_[10*cell + obs] > 0 && cell == 3 )
                    std::cout << "P(obs(cell=" << cell << ")=" << obs << ")=" << prob_obs_[10*cell + obs] << std::endl;
            }
        }
#endif

        for( std::list<std::pair<int, std::vector<int> > >::const_iterator it = initially_refuted_tags_.begin(); it != initially_refuted_tags_.end(); ++it ) {
            incremental_preprocess(it->first, &it->second);
        }

#if 0
        std::cout << "dump2:" << std::endl;
        for( int cell = 0, sz = prob_obs_.size() / 10; cell < sz; ++cell ) {
            for( int obs = 0; obs < 10; ++obs ) {
                if( prob_obs_[10*cell + obs] > 0 && cell == 3 )
                    std::cout << "P(obs(cell=" << cell << ")=" << obs << ")=" << prob_obs_[10*cell + obs] << std::endl;
            }
        }
#endif

        //std::cout << "end: initial preprocess" << std::endl;
    }

    void incremental_preprocess(int cell = -1, const std::vector<int> *refuted_tags = 0) {
        //std::cout << "begin: incremental preprocess: " << literals_.size() << std::endl;

assert(cell == cnfcell_2_pcell(pcell_2_cnfcell(cell)));

        std::set<int> extra_literals, new_literals;
        if( refuted_tags != 0 ) {
            assert(cell >= 0);
            for( unsigned i = 0, isz = refuted_tags->size(); i < isz; ++i ) {
                int t = (*refuted_tags)[i];
                //if( cell == 3 ) std::cout << "    tag t=" << t << " refuted by obs" << std::endl;
                int tag = (pcell_2_cnfcell(cell) << 9) + t;
                extra_literals.insert(-(1+tag));
            }
        }

        theory->deduction(literals_, extra_literals, new_literals);
        //literals_.insert(extra_literals.begin(), extra_literals.end());
        literals_.insert(new_literals.begin(), new_literals.end());

        //std::cout << "       new-literals: " << new_literals.size() << std::endl;

        // adjust probabilities given new implied literals: first process
        // negative literals that reduce support and then positive literals
        // that assert observations-

        // negative literals: decrease support
        for( std::set<int>::const_iterator it = new_literals.begin(); it != new_literals.end(); ++it ) {
            int lit = *it;
            assert(lit != 0);
            int tag = lit < 0 ? -lit - 1 : lit - 1;
            if( valid_pcell(tag >> 9) ) {
                int cell = cnfcell_2_pcell(tag >> 9);
                int type = cell_type(cell);
                int t = tag & 0b111111111;
                assert((t >= 0) && (t < 512));
                if( lit < 0 ) {
                    float p = tag_probability_[type][t];
                    if( p > 0 ) {
                        //if( cell == 3 ) std::cout << "    new literal: (cell=" << cell << ",t=" << t << "), p=" << p << std::endl;
                        for( unsigned i = 0, isz = supported_obs_[type][t].size(); i < isz; ++i ) {
                            int n = supported_obs_[type][t][i];
                            //if( cell == 3 ) std::cout << "    cell=" << cell << ", obs=" << n << ", prob=" << prob_obs_[10*cell + n] << std::endl;
                            prob_obs_[10*cell + n] -= p;
                            assert(prob_obs_[10*cell + n] >= 0);
                        }
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

#if 1
        // normalize probabilities
        for( int cell = 0, sz = prob_obs_.size() / 10; cell < sz; ++cell ) {
            float mass = 0;
            for( int obs = 0; obs < 10; ++obs )
                mass += prob_obs_[10*cell + obs];
            if( mass > 0 ) {
                for( int obs = 0; obs < 10; ++obs )
                    prob_obs_[10*cell + obs] /= mass;
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
    std::vector<std::vector<int> > tags_for_type_;
    std::vector<std::vector<int> > refuted_tags_[NTYPES];

  public:
    problem_t(int size, int nbombs)
      : size_(size), nbombs_(nbombs), init_(0) {

        // calculate the tags applicable to each cell type
std::cout << "calculate applicable tags" << std::endl;
        tags_for_type_.reserve(NTYPES);
        for( int type = 0; type < NTYPES; ++type ) {
            std::set<int> tags;
            for( int t = 0; t < 512; ++t ) {
                int tag = t;
                if( type == CORNER1 ) {
                    tag = t & 0b110110000;
                } else if( type == CORNER2 ) {
                    tag = t & 0b011011000;
                } else if( type == CORNER3 ) {
                    tag = t & 0b000110110;
                } else if( type == CORNER4 ) {
                    tag = t & 0b000011011;
                } else if( type == SIDE1 ) {
                    tag = t & 0b111111000;
                } else if( type == SIDE2 ) {
                    tag = t & 0b110110110;
                } else if( type == SIDE3 ) {
                    tag = t & 0b011011011;
                } else if( type == SIDE4 ) {
                    tag = t & 0b000111111;
                }
                tags.insert(tag);
            }
            std::vector<int> unique_tags;
            unique_tags.reserve(tags.size());
            for( std::set<int>::const_iterator it = tags.begin(); it != tags.end(); ++it )
                unique_tags.push_back(*it);
            tags_for_type_.push_back(unique_tags);
        }

std::cout << "calculate refuted/support tags" << std::endl;
        // refuted/support tags for observations
        std::vector<std::vector<int> > support_tags[NTYPES];
        for( int type = 0; type < NTYPES; ++type ) {
            support_tags[type] = std::vector<std::vector<int> >(10, std::vector<int>());
            supported_obs_[type] = std::vector<std::vector<int> >(512, std::vector<int>());
            refuted_tags_[type] = std::vector<std::vector<int> >(10, std::vector<int>());
            for( int i = 0, isz = tags_for_type_[type].size(); i < isz; ++i ) {
                int t = tags_for_type_[type][i];
                int nbits_in_tag = 0;
                for( int aux = t; aux != 0; aux = aux >> 1 ) {
                    if( (aux & 1) != 0 ) ++nbits_in_tag;
                }
                assert(nbits_in_tag <= 9);
                if( (t >> 4) & 1 ) {
                    support_tags[type][9].push_back(t);
                    supported_obs_[type][t].push_back(9);
                    for( int obs = 0; obs < 9; ++obs ) {
                        refuted_tags_[type][obs].push_back(t);
                    }
                } else {
                    support_tags[type][nbits_in_tag].push_back(t);
                    supported_obs_[type][t].push_back(nbits_in_tag);
                    for( int obs = 0; obs < 10; ++obs ) {
                        if( obs != nbits_in_tag ) {
                            refuted_tags_[type][obs].push_back(t);
                        }
                    }
                }
            }
        }

std::cout << "calculate probability of tags" << std::endl;
        // set (global) probabilities for tags
        for( int type = 0; type < NTYPES; ++type ) {
            float p = 1.0 / 512.0;
            if( (type > 0) && (type < SIDE1) ) {
                p = 1.0 / 16.0;
            } else if( type >= SIDE1 ) {
                p = 1.0 / 64.0;
            }
            tag_probability_[type] = std::vector<float>(512, 0.0);
            float mass = 0;
            for( int i = 0, isz = tags_for_type_[type].size(); i < isz; ++i ) {
                int t = tags_for_type_[type][i];
                tag_probability_[type][t] = p;
                mass += p;
            }
            assert(mass == 1);
        }

std::cout << "calculate default probabilities for obs" << std::endl;
        // set (global) default probabilities for obs
        default_probabilities_ = std::vector<float>(10 * size_, 0);
        for( int cell = 0; cell < size_; ++cell ) {
            float mass = 0;
            int type = cell_type(cell);
            for( int obs = 0; obs < 10; ++obs ) {
                for( int i = 0, isz = support_tags[type][obs].size(); i < isz; ++i ) {
                    int t = support_tags[type][obs][i];
                    float p = tag_probability_[type][t];
                    default_probabilities_[10*cell + obs] += p;
                    mass += p;
                }
            }
            assert(mass == 1.0);
        }

std::cout << "calculate initial refuted tags given knowledge about #bombs" << std::endl;
        if( nbombs_ < 8 ) {
            // no observation with more than nbombs is possible,
            // so remove all tags incompatible with this number
            for( int cell = 0; cell < size_; ++cell ) {
                std::vector<int> refuted;
                int type = cell_type(cell);
                for( int obs = 1 + nbombs_; obs < 9; ++obs ) {
                    for( int i = 0, isz = support_tags[type][obs].size(); i < isz; ++i ) {
                        int t = support_tags[type][obs][i];
                        refuted.push_back(t);
if( cell == 3 ) std::cout << "    tag (cell=" << cell << ",t=" << t << ") is refuted at init by #bombs=" << nbombs_ << std::endl;
                    }
                }
                for( int i = 0, isz = support_tags[type][9].size(); i < isz; ++i ) {
                    int t = support_tags[type][9][i];
                    assert(t & 0b000010000);
                    int nbits = 0;
                    for( int aux = t; aux != 0; aux = aux >> 1 ) {
                        nbits += aux & 1 ? 1 : 0;
                    }
                    if( nbits > nbombs_ ) {
                        refuted.push_back(t);
if( cell == 3 ) std::cout << "    tag (cell=" << cell << ",t=" << t << ") is refuted at init by #bombs=" << nbombs_ << std::endl;
                    }
                }
                refuted.push_back(0);
                initially_refuted_tags_.push_back(std::make_pair(cell, refuted));
            }
        }

std::cout << "create logical theory" << std::endl;
        // create logical theory
        theory = new theory_t(2, 2, nbombs_);
        theory->create_manager();

std::cout << "create initial state" << std::endl;
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
        //std::cout << "cost(a=" << a << ",s=" << s << ")=" << s.cost(a&1,a>>1) << std::endl;
        return s.cost(a & 1, a >> 1);
    }
    virtual void next(const state_t &s, Problem::action_t a, std::vector<std::pair<state_t, float> > &outcomes) const {
        assert(s.nbombs_ >= 0);

        ++expansions_;
        outcomes.clear();

        int cell = a >> 1;
        int act = a & 1;
        int type = cell_type(cell);
        std::cout << "next" << s << " w/ a=(" << (act == FLAG_CELL ? "FLAG:" : "OPEN:") << cell << ") is:" << std::endl;

        float mass = 0;
        if( act == FLAG_CELL ) {
            // this is a FLAG action, it has only 1 outcome
            outcomes.reserve(1);
            state_t next = s;
            next.apply(act, cell, OBS_FLAG);
            outcomes.push_back(std::make_pair(next, 1.0));
            std::cout << "    " << next << " w.p. " << 1.0 << std::endl;
            mass += 1;
        } else {
            // this is an OPEN action, it has 10 possible outcomes:
            // observing a number between 0..8 and DEAD. The possible
            // outcomes are those with non-zero probability
            outcomes.reserve(10);
            for( int obs = 0; obs < 10; ++obs ) {
                float p = s.probability(act, cell, obs);
                if( p > 0 ) {
                    state_t next = s;
                    next.apply(act, cell, obs, &refuted_tags_[type][obs]);
                    outcomes.push_back(std::make_pair(next, p));
                    std::cout << "    " << next << " w.p. " << p << std::endl;
                    mass += p;
                }
            }
        }
        //std::cout << "done with expansion: mass=" << mass << std::endl;
        assert(!outcomes.empty());
        assert(fabs(mass - 1.0) < 0.00001);
    }
    virtual void print(std::ostream &os) const { }
};

inline std::ostream& operator<<(std::ostream &os, const problem_t &p) {
    p.print(os);
    return os;
}

class min_min_t : public Heuristic::heuristic_t<state_t> {
  public:
    min_min_t() { }
    virtual ~min_min_t() { }
    virtual float value(const state_t &s) const { return (float)s.nbombs_; }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const state_t &s) const { return value(s); }
};

