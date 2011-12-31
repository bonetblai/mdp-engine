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

#define MOVE_BWD    0
#define MOVE_FWD    1
#define CLOSE       2
#define LOCK        3
#define GRAB_KEY    4

inline unsigned rotation(unsigned x) {
    return (x << 16) | (x >> 16);
}

template<typename T> class cow_vector_t {
  private:
    mutable int nrefs_;
    int capacity_;
    int size_;
    T *vector_;

  public:
    cow_vector_t() : nrefs_(1), capacity_(0), size_(0), vector_(0) {
    }
    cow_vector_t(const cow_vector_t &vec)
      : nrefs_(1), capacity_(0), size_(0), vector_(0) {
        *this = vec;
    }
    ~cow_vector_t() {
        delete[] vector_;
    }

    static void deallocate(const cow_vector_t *vec) {
        assert(vec != 0);
        if( --vec->nrefs_ == 0 ) delete vec;
    }
    static cow_vector_t* ref(const cow_vector_t *vec) {
        assert(vec != 0);
        ++vec->nrefs_;
        return const_cast<cow_vector_t*>(vec);
    }

    void reserve(int ncapacity) {
        if( capacity_ < ncapacity ) {
            capacity_ = ncapacity;
            T *nvector = new T[capacity_];
            for( int i = 0; i < size_; ++i )
                nvector[i] = vector_[i];
            delete[] vector_;
            vector_ = nvector;
        }
    }

    int nrefs() const { return nrefs_; }
    int size() const { return size_; }
    void clear() { size_ = 0; }

    void push_back(const T &element) {
        if( size_ == capacity_ ) reserve(1 + capacity_);
        vector_[size_++] = element;
    }
    void insert(const T &element) {
        if( capacity_ == 0 ) {
            reserve(1);
            vector_[0] = element;
            ++size_;
        } else {
            int i = 0;
            while( (vector_[i] < element) && (i < size_) ) ++i;
            if( (i == size_) || (vector_[i] != element) ) {
                if( size_ == capacity_ ) reserve(1 + capacity_);
                for( int j = size_ - 1; i <= j; --j ) {
                    vector_[1+j] = vector_[j];
                }
                vector_[i] = element;
                ++size_;
            }
        }
    }

    const T& operator[](int i) const { return vector_[i]; }
    cow_vector_t& operator=(const cow_vector_t &vec) {
        reserve(vec.size_);
        for( int i = 0; i < vec.size_; ++i )
            vector_[i] = vec[i];
        size_ = vec.size_;
        return *this;
    }
    bool operator==(const cow_vector_t &vec) const {
        if( size_ != vec.size_ )
            return false;
        else {
            for( int i = 0; i < size_; ++i ) {
                if( vector_[i] != vec[i] )
                    return false;
            }
            return true;
        }
    }
    bool operator!=(const cow_vector_t &vec) const {
        return *this == vec ? false : true;
    }

    struct const_iterator {
        const T *ptr_;
        const_iterator(const T *ptr) : ptr_(ptr) { }
        ~const_iterator() { }
        const_iterator& operator++() {
            ++ptr_;
            return *this;
        }
        const T& operator*() const { return *ptr_; }
        bool operator==(const const_iterator &it) const {
            return ptr_ == it.ptr_;
        }
        bool operator!=(const const_iterator &it) const {
            return ptr_ != it.ptr_;
        }
    };
    const_iterator begin() const {
        return const_iterator(vector_);
    }
    const_iterator end() const {
        return const_iterator(&vector_[size_]);
    }

};

struct cow_belief_component_t {
    cow_vector_t<int> *cow_vector_;
   
    cow_belief_component_t()
      : cow_vector_(new cow_vector_t<int>) { }
    cow_belief_component_t(const cow_belief_component_t &bc)
      : cow_vector_(ref(bc.cow_vector_)) { }
    ~cow_belief_component_t() { deallocate(cow_vector_); }

    static void deallocate(cow_vector_t<int> *vec) {
        cow_vector_t<int>::deallocate(vec);
    }
    static cow_vector_t<int>* ref(const cow_vector_t<int> *vec) {
        return cow_vector_t<int>::ref(vec);
    }

    unsigned hash() const {
        unsigned value = 0;
        for( int i = 0; i < cow_vector_->size(); ++i ) {
            int sample = (*cow_vector_)[i];
            value = value ^ (i & 0x1 ? rotation(sample) : sample);
        }
        return value;
    }

    int nrefs() const {
        return cow_vector_->nrefs();
    }
    void reserve(int cap) {
        cow_vector_->reserve(cap);
    }
    void clear() {
        if( nrefs() > 1 ) {
            deallocate(cow_vector_);
            cow_vector_ = new cow_vector_t<int>;
        } else
            cow_vector_->clear();
    }
    int size() const {
        return cow_vector_->size();
    }
    void insert(int e) {
        if( nrefs() > 1 ) {
            deallocate(cow_vector_);
            cow_vector_t<int> *nvec = new cow_vector_t<int>(*cow_vector_);
            cow_vector_ = nvec;
        }
        cow_vector_->insert(e);
    }

    int operator[](int i) const { return (*cow_vector_)[i]; }
    const cow_belief_component_t& operator=(const cow_belief_component_t &bc) {
        deallocate(cow_vector_);
        cow_vector_ = ref(bc.cow_vector_);
        return *this;
    }
    bool operator==(const cow_belief_component_t &bc) const {
        return *cow_vector_ == *bc.cow_vector_;
    }
    bool operator!=(const cow_belief_component_t &bc) const {
        return *cow_vector_ != *bc.cow_vector_;
    }

    int window_status() const {
        int rv = -1;
        for( int i = 0; i < cow_vector_->size(); ++i ) {
            int sample = (*cow_vector_)[i];
            int status = sample & 0x3;
            if( rv == -1 )
                rv = status;
            else if( rv != status )
                return UNKNOWN;
        }
        assert(rv != -1);
        return rv;
    }

    void apply_close(int window_pos, cow_belief_component_t &result, bool non_det = false) const {
        result.reserve(size());
        for( int i = 0; i < cow_vector_->size(); ++i ) {
            int sample = (*cow_vector_)[i];
            int status = sample & 0x3;
            int key_pos = (sample >> 2) & 0xFF;
            int agent_pos = (sample >> 10) & 0xFF;
            if( (status != LOCKED) && (agent_pos == window_pos) ) {
                result.insert(CLOSED | (key_pos << 2) | (agent_pos << 10));
            } else if( non_det && (status != LOCKED) ) {
                result.insert(OPEN | (key_pos << 2) | (agent_pos << 10));
                result.insert(CLOSED | (key_pos << 2) | (agent_pos << 10));
            } else {
                result.insert(sample);
            }
        }
    }
    void apply_lock(int window_pos, cow_belief_component_t &result, bool non_det = false) const {
        result.reserve(size());
        for( int i = 0; i < cow_vector_->size(); ++i ) {
            int sample = (*cow_vector_)[i];
            int status = sample & 0x3;
            int key_pos = (sample >> 2) & 0xFF;
            int agent_pos = (sample >> 10) & 0xFF;
            if( (status == CLOSED) && (key_pos == AGENT) && (agent_pos == window_pos) ) {
                result.insert(LOCKED | (AGENT << 2) | (agent_pos << 10));
            } else if( non_det && (status != LOCKED) ) {
                result.insert(OPEN | (key_pos << 2) | (agent_pos << 10));
                result.insert(CLOSED | (key_pos << 2) | (agent_pos << 10));
            } else {
                result.insert(sample);
            }
        }
    }
    void apply_move(int dim, int steps, cow_belief_component_t &result, bool non_det = false) const {
        result.reserve(size());
        for( int i = 0; i < cow_vector_->size(); ++i ) {
            int sample = (*cow_vector_)[i];
            int status = sample & 0x3;
            int key_pos = (sample >> 2) & 0xFF;
            int agent_pos = (sample >> 10) & 0xFF;
            int new_pos = (agent_pos + steps) % dim;
            new_pos = new_pos < 0 ? -new_pos : new_pos;
            if( non_det && (status != LOCKED) ) {
                result.insert(OPEN | (key_pos << 2) | (new_pos << 10));
                result.insert(CLOSED | (key_pos << 2) | (new_pos << 10));
            } else {
                result.insert(status | (key_pos << 2) | (new_pos << 10));
            }
        }
    }
    void apply_grab_key(cow_belief_component_t &result, bool non_det = false) const {
        result.reserve(size());
        for( int i = 0; i < cow_vector_->size(); ++i ) {
            int sample = (*cow_vector_)[i];
            int status = sample & 0x3;
            int key_pos = (sample >> 2) & 0xFF;
            int agent_pos = (sample >> 10) & 0xFF;
            if( key_pos == agent_pos ) {
                if( non_det && (status != LOCKED) ) {
                    result.insert(OPEN | (AGENT << 2) | (agent_pos << 10));
                    result.insert(CLOSED | (AGENT << 2) | (agent_pos << 10));
                } else {
                    result.insert(status | (AGENT << 2) | (agent_pos << 10));
                }
            } else {
                if( non_det && (status != LOCKED) ) {
                    result.insert(OPEN | (key_pos << 2) | (agent_pos << 10));
                    result.insert(CLOSED | (key_pos << 2) | (agent_pos << 10));
                } else {
                    result.insert(sample);
                }
            }
        }
    }

    void print(std::ostream &os) const {
        os << "{";
        for( int i = 0; i < cow_vector_->size(); ++i ) {
            int sample = (*cow_vector_)[i];
            os << sample << ",";
        }
        os << "}";
    }
};

inline std::ostream& operator<<(std::ostream &os, const cow_belief_component_t &bc) {
    bc.print(os);
    return os;
}


struct myset {
    int *set_;
    int capacity_;
    int size_;
    myset() : set_(0), capacity_(0), size_(0) { }
    myset(const myset &ms) : set_(0), capacity_(0), size_(0) { *this = ms; }
    ~myset() { delete[] set_; }
    void reserve(int new_capacity) {
        if( capacity_ < new_capacity ) {
            capacity_ = new_capacity;
            int *nset = new int[capacity_];
            memcpy(nset, set_, size_ * sizeof(int));
            delete[] set_;
            set_ = nset;
        }
    }
    void clear() { size_ = 0; }
    int size() const { return size_; }
    void push_back(int e) {
        if( size_ == capacity_ ) reserve(1 + capacity_);
        set_[size_++] = e;
    }
    void insert(int e) {
        int i = 0;
        while( (set_[i] < e) && (i < size_) ) ++i;
        if( set_[i] != e ) {
            if( size_ == capacity_ ) reserve(1 + capacity_);
            for( int j = size_ - 1; i <= j; --j ) {
                set_[1+j] = set_[j];
            }
            set_[i] = e;
            ++size_;
        }
    }

    int operator[](int i) const { return set_[i]; }
    const myset& operator=(const myset &ms) {
        reserve(ms.size_);
        memcpy(set_, ms.set_, ms.size_ * sizeof(int));
        size_ = ms.size_;
        return *this;
    }
    bool operator==(const myset &ms) const {
        return size_ != ms.size_ ? false : memcmp(set_, ms.set_, size_ * sizeof(int)) == 0;
    }
    bool operator!=(const myset &ms) const {
        return *this == ms ? false : true;
    }

    struct const_iterator {
        const int *ptr_;
        const_iterator(const int *ptr) : ptr_(ptr) { }
        ~const_iterator() { }
        const_iterator& operator++() {
            ++ptr_;
            return *this;
        }
        int operator*() const { return *ptr_; }
        bool operator==(const const_iterator &it) const {
            return ptr_ == it.ptr_;
        }
        bool operator!=(const const_iterator &it) const {
            return ptr_ != it.ptr_;
        }
    };
    const_iterator begin() const {
        return const_iterator(set_);
    }
    const_iterator end() const {
        return const_iterator(&set_[size_]);
    }

};

struct belief_component_t : public myset { //: public std::set<int> {
   
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

    void apply_close(int window_pos, belief_component_t &result, bool non_det = false) const {
        result.reserve(size());
        for( const_iterator it = begin(); it != end(); ++it ) {
            int status = *it & 0x3;
            int key_pos = ((*it) >> 2) & 0xFF;
            int agent_pos = ((*it) >> 10) & 0xFF;
            if( (status != LOCKED) && (agent_pos == window_pos) ) {
                result.insert(CLOSED | (key_pos << 2) | (agent_pos << 10));
            } else if( non_det && (status != LOCKED) ) {
                result.insert(OPEN | (key_pos << 2) | (agent_pos << 10));
                result.insert(CLOSED | (key_pos << 2) | (agent_pos << 10));
            } else {
                result.insert(*it);
            }
        }
    }
    void apply_lock(int window_pos, belief_component_t &result, bool non_det = false) const {
        result.reserve(size());
        for( const_iterator it = begin(); it != end(); ++it ) {
            int status = *it & 0x3;
            int key_pos = ((*it) >> 2) & 0xFF;
            int agent_pos = ((*it) >> 10) & 0xFF;
            if( (status == CLOSED) && (key_pos == AGENT) && (agent_pos == window_pos) ) {
                result.insert(LOCKED | (AGENT << 2) | (agent_pos << 10));
            } else if( non_det && (status != LOCKED) ) {
                result.insert(OPEN | (key_pos << 2) | (agent_pos << 10));
                result.insert(CLOSED | (key_pos << 2) | (agent_pos << 10));
            } else {
                result.insert(*it);
            }
        }
    }
    void apply_move(int dim, int steps, belief_component_t &result, bool non_det = false) const {
        result.reserve(size());
        for( const_iterator it = begin(); it != end(); ++it ) {
            int status = *it & 0x3;
            int key_pos = ((*it) >> 2) & 0xFF;
            int agent_pos = ((*it) >> 10) & 0xFF;
            int new_pos = (agent_pos + steps) % dim;
            new_pos = new_pos < 0 ? -new_pos : new_pos;
            if( non_det && (status != LOCKED) ) {
                result.insert(OPEN | (key_pos << 2) | (new_pos << 10));
                result.insert(CLOSED | (key_pos << 2) | (new_pos << 10));
            } else {
                result.insert(status | (key_pos << 2) | (new_pos << 10));
            }
        }
    }
    void apply_grab_key(belief_component_t &result, bool non_det = false) const {
        result.reserve(size());
        for( const_iterator it = begin(); it != end(); ++it ) {
            int status = *it & 0x3;
            int key_pos = ((*it) >> 2) & 0xFF;
            int agent_pos = ((*it) >> 10) & 0xFF;
            if( key_pos == agent_pos ) {
                if( non_det && (status != LOCKED) ) {
                    result.insert(OPEN | (AGENT << 2) | (agent_pos << 10));
                    result.insert(CLOSED | (AGENT << 2) | (agent_pos << 10));
                } else {
                    result.insert(status | (AGENT << 2) | (agent_pos << 10));
                }
            } else {
                if( non_det && (status != LOCKED) ) {
                    result.insert(OPEN | (key_pos << 2) | (agent_pos << 10));
                    result.insert(CLOSED | (key_pos << 2) | (agent_pos << 10));
                } else {
                    result.insert(*it);
                }
            }
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

struct state_t {
    //std::vector<belief_component_t> components_;
    std::vector<cow_belief_component_t> components_;
    static int dim_;

    state_t() : components_(dim_) { }
    state_t(const state_t &bel) : components_(bel.components_) { }
    ~state_t() { }

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

    void apply(int action, state_t &result, bool non_det = false) const {
        result.clear();
        for( int i = 0; i < dim_; ++i ) {
            if( action == MOVE_FWD ) {
                components_[i].apply_move(dim_, 1, result.components_[i], non_det);
            } else if( action == MOVE_BWD ) {
                components_[i].apply_move(dim_, -1, result.components_[i], non_det);
            } else if( action == CLOSE ) {
                components_[i].apply_close(i, result.components_[i], non_det);
            } else if( action == LOCK ) {
                components_[i].apply_lock(i, result.components_[i], non_det);
            } else if( action == GRAB_KEY ) {
                components_[i].apply_grab_key(result.components_[i], non_det);
            }
        }
    }

    const state_t& operator=(const state_t &bel) {
        for( int i = 0; i < dim_; ++i ) {
            components_[i] = bel.components_[i];
        }
        return *this;
    }
    bool operator==(const state_t &bel) const {
        for( int i = 0; i < dim_; ++i ) {
            if( components_[i] != bel.components_[i] )
                return false;
        }
        return true;
    }
    bool operator!=(const state_t &bel) const {
        return *this == bel ? false : true;
    }
#if 0
    bool operator<(const state_t &bel) const {
        for( int i = 0; i < dim_; ++i ) {
            if( components_[i] < bel.components_[i] )
                return true;
            else if( components_[i] != bel.components_[i] )
                return false;
        }
        return false;
    }
#endif

    void print(std::ostream &os) const {
        for( int i = 0; i < dim_; ++i ) {
            os << "comp[" << i << "]=" << components_[i] << std::endl;
        }
    }
};

int state_t::dim_ = 0;

inline std::ostream& operator<<(std::ostream &os, const state_t &b) {
    b.print(os);
    return os;
}

struct problem_t : public Problem::problem_t<state_t> {
    int dim_;
    bool non_det_;
    state_t init_;

    problem_t(int dim, bool non_det = false)
      : Problem::problem_t<state_t>(DISCOUNT), dim_(dim), non_det_(non_det) {
        // set initial belief
        for( int window = 0; window < dim_; ++window) {
            init_.components_[window].reserve(2 * dim_);
            for( int agent_pos = 0; agent_pos < dim_; ++agent_pos ) {
                for( int status = 0; status < 2; ++status ) {
#if 1
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
        //std::cout << "Initial Belief:" << std::endl << init_ << std::endl;
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions(const state_t &s) const {
        return 5;
        //return 4;
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
        s.apply(a, next, non_det_);
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

class fwd_random_policy_t : public Policy::policy_t<state_t> {
  public:
    fwd_random_policy_t(const Problem::problem_t<state_t> &problem)
      : Policy::policy_t<state_t>(problem) { }
    virtual ~fwd_random_policy_t() { }
    virtual Problem::action_t operator()(const state_t &s) const {
        return 1 + Random::uniform(3);
    }
    virtual const Policy::policy_t<state_t>* clone() const {
        return new fwd_random_policy_t(problem());
    }
    virtual void print_stats(std::ostream &os) const { }
};



