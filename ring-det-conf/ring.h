#include <cassert>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>
#include <limits>
#include <set>

#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "dispatcher.h"

#define DISCOUNT 1.00

// 2 bits indicate: open=0, closed=1, locked=2
// 8 bits indicate position of agent
// 8 bits indicate position of key

#define OPEN        0
#define CLOSED      1
#define LOCKED      2
#define UNKNOWN     3
#define AGENT       0xFF

#define MOVE_FWD    0
#define MOVE_BWD    1
#define CLOSE       2
#define LOCK        3
#define GRAB_KEY    4

inline unsigned rotation(unsigned x) {
    return (x << 16) | (x >> 16);
}

struct belief_component_t : public std::set<int> {
   
    belief_component_t() { }
    ~belief_component_t() { }

    unsigned hash() const {
        int i = 0;
        unsigned value = 0;
        for( const_iterator it = begin(); it != end(); ++it, ++i ) {
            value = value ^ (i & 0x1 ? rotation(*it) : *it);
        }
        return value;
    }

    int window_status() const {
        int rv = -1;
        for( const_iterator it = begin(); it != end(); ++it ) {
            int status = *it & 0x3;
            if( rv == -1 )
                rv = status;
            else if( rv != status )
                return UNKNOWN;
        }
        assert(rv != -1);
        return rv;
    }

    void apply_close(int window_pos, belief_component_t &result) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            int status = *it & 0x3;
            int key_pos = ((*it) >> 2) & 0xFF;
            int agent_pos = ((*it) >> 10) & 0xFF;
            if( (status != LOCKED) && (agent_pos == window_pos) )
                result.insert(CLOSED | (key_pos << 2) | (agent_pos << 10));
            else
                result.insert(*it);
        }
    }
    void apply_lock(int window_pos, belief_component_t &result) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            int status = *it & 0x3;
            int key_pos = ((*it) >> 2) & 0xFF;
            int agent_pos = ((*it) >> 10) & 0xFF;
            if( (status == CLOSED) && (key_pos == AGENT) && (agent_pos == window_pos) )
                result.insert(LOCKED | (AGENT << 2) | (agent_pos << 10));
            else
                result.insert(*it);
        }
    }
    void apply_move(int dim, int steps, belief_component_t &result) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            int status = *it & 0x3;
            int key_pos = ((*it) >> 2) & 0xFF;
            int agent_pos = ((*it) >> 10) & 0xFF;
            int new_pos = (agent_pos + steps) % dim;
            new_pos = new_pos < 0 ? -new_pos : new_pos;
            result.insert(status | (key_pos << 2) | (new_pos << 10));
        }
    }
    void apply_grab_key(belief_component_t &result) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            int status = *it & 0x3;
            int key_pos = ((*it) >> 2) & 0xFF;
            int agent_pos = ((*it) >> 10) & 0xFF;
            if( key_pos == agent_pos )
                result.insert(status | (AGENT << 2) | (agent_pos << 10));
            else
                result.insert(*it);
        }
    }

    void print(std::ostream &os) const {
        os << "{";
        for( const_iterator it = begin(); it != end(); ++it )
            os << *it << ",";
        os << "}";
    }
};

inline std::ostream& operator<<(std::ostream &os, const belief_component_t &bc) {
    bc.print(os);
    return os;
}

struct belief_t {
    std::vector<belief_component_t> components_;
    static int dim_;

    belief_t() : components_(dim_) { }
    ~belief_t() { }

    static void initialize(int dim) {
        dim_ = dim;
    }

    unsigned hash() const {
        unsigned value = 0;
        for( int i = 0; i < dim_; ++i )
            value = value ^ components_[i].hash();
        return value;
    }

    void clear() {
        for( int i = 0; i < dim_; ++i )
            components_[i].clear();
    }

    bool locked() const {
        for( int i = 0; i < dim_; ++i ) {
            if( components_[i].window_status() != LOCKED )
                return false;
        }
        return true;
    }
    bool goal() const { return locked(); }

    void apply(int action, belief_t &result) const {
        result.clear();
        for( int i = 0; i < dim_; ++i ) {
            if( action == MOVE_FWD ) {
                components_[i].apply_move(dim_, 1, result.components_[i]);
            } else if( action == MOVE_BWD ) {
                components_[i].apply_move(dim_, -1, result.components_[i]);
            } else if( action == CLOSE ) {
                components_[i].apply_close(i, result.components_[i]);
            } else if( action == LOCK ) {
                components_[i].apply_lock(i, result.components_[i]);
            } else if( action == GRAB_KEY ) {
                components_[i].apply_grab_key(result.components_[i]);
            }
        }
    }

    const belief_t& operator=(const belief_t &bel) {
        for( int i = 0; i < dim_; ++i ) {
            components_[i] = bel.components_[i];
        }
        return *this;
    }
    bool operator==(const belief_t &bel) const {
        for( int i = 0; i < dim_; ++i ) {
            if( components_[i] != bel.components_[i] )
                return false;
        }
        return true;
    }
    bool operator!=(const belief_t &bel) const {
        return *this == bel ? false : true;
    }
    bool operator<(const belief_t &bel) const {
        for( int i = 0; i < dim_; ++i ) {
            if( components_[i] < bel.components_[i] )
                return true;
            else if( components_[i] != bel.components_[i] )
                return false;
        }
        return false;
    }

    void print(std::ostream &os) const {
        for( int i = 0; i < dim_; ++i ) {
            os << "comp[" << i << "]=" << components_[i] << std::endl;
        }
    }
};

int belief_t::dim_ = 0;

inline std::ostream& operator<<(std::ostream &os, const belief_t &b) {
    b.print(os);
    return os;
}

struct state_t : public belief_t { };


struct problem_t : public Problem::problem_t<state_t> {
    int dim_;
    state_t init_;

    problem_t(int dim) : Problem::problem_t<state_t>(DISCOUNT), dim_(dim) {
        // set initial state
        for( int window = 0; window < dim_; ++window) {
            for( int agent_pos = 0; agent_pos < dim_; ++agent_pos ) {
                for( int status = 0; status < 3; ++status ) {
#if 0
                    for( int key_pos = 0; key_pos < dim_; ++key_pos ) {
                        int sample = status | (key_pos << 2) | (agent_pos << 10);
                        init_.components_[window].insert(sample);
                    }
#endif
                    int sample = status | (AGENT << 2) | (agent_pos << 10);
                    init_.components_[window].insert(sample);
                }
            }
        }
        std::cout << "Initial Belief:" << std::endl << init_ << std::endl;
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions(const state_t &s) const {
        //return 5;
        return 4;
    }
    virtual bool applicable(const state_t &s, Problem::action_t a) const {
        return true;
    }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return s.goal();
    }
    virtual bool dead_end(const state_t &s) const {
        return false;
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return 1;
    }
    virtual void next(const state_t &s,
                      Problem::action_t a,
                      std::vector<std::pair<state_t, float> > &outcomes) const {

        //std::cout << "next " << s << " w/ a=" << a << " is:" << std::endl;
        ++expansions_;
        outcomes.clear();
        outcomes.reserve(1);

        state_t next;
        s.apply(a, next);
        outcomes.push_back(std::make_pair(next, 1));
        //std::cout << "    " << next << " w.p. " << 1 << std::endl;
    }
    virtual void print(std::ostream &os) const { }

};

struct cardinality_heuristic_t : public Heuristic::heuristic_t<state_t> {
  public:
    cardinality_heuristic_t() { }
    virtual ~cardinality_heuristic_t() { }
    virtual float value(const state_t &s) const {
        int value = 0;
        for( int i = 0; i < s.dim_; ++i ) {
            int compsz = s.components_[i].size();
            value = compsz > value ? compsz : value;
        }
        return value - 1;
    }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const state_t &s) const { return value(s); }
};

struct window_heuristic_t : public Heuristic::heuristic_t<state_t> {
  public:
    window_heuristic_t() { }
    virtual ~window_heuristic_t() { }
    virtual float value(const state_t &s) const {
        int value = 0;
        for( int i = 0; i < s.dim_; ++i ) {
            value += s.components_[i].window_status() != LOCKED ? 2 : 0;
        }
        return value;
    }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const state_t &s) const { return value(s); }
};


