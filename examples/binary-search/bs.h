#include <iostream>
#include <iomanip>
#include <strings.h>

#define DISCOUNT .95

class belief_state_t {
  protected:
    unsigned bitmap_;
    int hidden_;

  public:
    belief_state_t(int hidden = 0) : bitmap_(0), hidden_(hidden) { }
    belief_state_t(const belief_state_t &s) : bitmap_(s.bitmap_), hidden_(s.hidden_) { }
    ~belief_state_t() { }

    size_t hash() const {
        return bitmap_;
    }

#if 0 // CHECK
    belief_state_t apply(Problem::action_t a) const {
        std::pair<int, int> dir = direction(a);
        return belief_state_t(x_ + dir.first, y_ + dir.second);
    }
#endif

    const belief_state_t& operator=( const belief_state_t &s) {
        bitmap_ = s.bitmap_;
        hidden_ = s.hidden_;
        return *this;
    }
    bool operator==(const belief_state_t &s) const {
        return (bitmap_ == s.bitmap_) && (hidden_ == s.hidden_);
    }
    bool operator!=(const belief_state_t &s) const {
        return (bitmap_ != s.bitmap_) || (hidden_ != s.hidden_);
    }
    bool operator<(const belief_state_t &s) const {
        return (bitmap_ < s.bitmap_) || ((bitmap_ == s.bitmap_) && (hidden_ < s.hidden_));
    }
    void print(std::ostream &os) const {
        //os << "(" << x_ << "," << y_ << "," << wind_ << ")";
        assert(0);
    }
    friend class problem_t;
};

inline std::ostream& operator<<(std::ostream &os, const belief_state_t &s) {
    s.print(os);
    return os;
}

class problem_t : public Problem::problem_t<belief_state_t> {
    int dim_;
    mutable belief_state_t init_tmp_;

  public:
    problem_t(int dim) : Problem::problem_t<belief_state_t>(DISCOUNT), dim_(dim) {
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions(const belief_state_t &s) const {
        return dim_;
    }
    virtual const belief_state_t& init() const {
        init_tmp_ = belief_state_t(Random::random(0, dim_));
        return init_tmp_;
    }
    virtual bool terminal(const belief_state_t &s) const {
        return false; // CHECK
    }
    virtual bool dead_end(const belief_state_t &s) const {
        return false;
    }
    virtual bool applicable(const belief_state_t &s, ::Problem::action_t a) const {
        return true;
    }
    virtual float min_absolute_cost() const { return 1; }
    virtual float max_absolute_cost() const { return 1; }
    virtual float cost(const belief_state_t &s, Problem::action_t a) const {
        return 1;
    }
    virtual int max_action_branching() const {
        return dim_;
    }
    virtual int max_state_branching() const {
        return 2;
    }
    virtual void next(const belief_state_t &s, Problem::action_t a, std::vector<std::pair<belief_state_t,float> > &outcomes) const {
#if 0 // CHECK
        ++expansions_;
        outcomes.clear();
        //outcomes.reserve(8);
        belief_state_t next_s = s.apply(a);
        assert(next_s.in_lake(rows_, cols_));
#if 0
        for( int nwind = 0; nwind < 8; ++nwind ) {
            float p = wind_transition_[s.wind_ * 8 + nwind];
            if( p > 0 ) {
                next_s.wind_ = nwind;
                outcomes.push_back(std::make_pair(next_s, p));
            }
        }
#endif
        const std::vector<std::pair<int, float> > &sparse_transition = sparse_wind_transition_[s.wind_];
        outcomes.reserve(sparse_transition.size());
        for( int i = 0; i < int(sparse_transition.size()); ++i ) {
            const std::pair<int, float> &p = sparse_transition[i];
            next_s.wind_ = p.first;
            outcomes.push_back(std::make_pair(next_s, p.second));
        }
#endif
    }
    virtual void print(std::ostream &os) const { }
};

inline std::ostream& operator<<(std::ostream &os, const problem_t &p) {
    p.print(os);
    return os;
}

