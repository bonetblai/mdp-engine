#include <cassert>
#include <iostream>
#include <iomanip>

#define MAXOUTCOMES   10

#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "mcts.h"
#include "dispatcher.h"

using namespace std;

const Problem::action_t up = 0;
const Problem::action_t left = 1;
const Problem::action_t down = 2;
const Problem::action_t right = 3;

class bits_t {
    size_t bits_;
  public:
    bits_t(size_t bits) : bits_(bits) { }
    ~bits_t() { }
    void print(ostream &os) const {
        for( size_t i = 8*sizeof(size_t); i > 0; --i )
            os << ((bits_ >> (i-1)) % 2);
    }
};

inline ostream& operator<<(ostream &os, const bits_t &b) {
    b.print(os);
    return os;
}

class state_t {
    size_t d0_;
    size_t d1_;
    size_t d2_;
  public:
    state_t(size_t d0 = 0, size_t d1 = 0, size_t d2 = 0) : d0_(d0), d1_(d1), d2_(d2) { }
    state_t(const state_t &s) : d0_(s.d0_), d1_(s.d1_), d2_(s.d2_) { }
    ~state_t() { }
    size_t hash() const { return d1_ ^ d2_; }
    size_t rows() const { return (d0_>>4) & 0xF; }
    size_t cols() const { return (d0_>>8) & 0xF; }
    bool applicable(size_t rows, size_t cols, Problem::action_t a) const {
        return true;
        switch( a ) {
            case ::up: return blank() >= cols;
            case ::right: return blank() % cols < cols-1;
            case ::down: return blank() + cols < rows*cols;
            case ::left: return blank() % cols > 0;
        }
        return false;
    }
    void set_tile(size_t t, size_t pos) {
        size_t shift = pos < 8 ? pos << 2 : (pos - 8) << 2;
        if( pos < 8 ) {
            d1_ &= ~(0xF << shift);
            d1_ |= t<<shift;
        } else {
            d2_ &= ~(0xF << shift);
            d2_ |= t << shift;
        }
    }
    void set_blank(size_t pos) {
        d0_ &= ~(0xF);
        d0_ |= pos;
        set_tile(0, pos);
    }
    size_t blank() const {
        return d0_ & 0xF;
    }
    size_t tile(size_t pos) const {
        size_t shift = pos < 8 ? pos << 2 : (pos - 8) << 2;
        return pos < 8 ? (d1_ >> shift) & 0xF : (d2_ >> shift) & 0xF;
    }
    void up(size_t rows, size_t cols) {
        size_t b = blank();
        if( b >= cols ) {
            size_t p = b-cols, t = tile(p);
            set_tile(t, b);
            set_blank(p);
        }
    }
    void right(size_t rows, size_t cols) {
        size_t b = blank();
        if( b%cols < cols-1 ) {
            size_t p = b+1, t = tile(p);
            set_tile(t, b);
            set_blank(p);
        }
    }
    void down(size_t rows, size_t cols) {
        size_t b = blank();
        if( b+cols < rows*cols ) {
            size_t p = b+cols, t = tile(p);
            set_tile(t, b);
            set_blank(p);
        }
    }
    void left(size_t rows, size_t cols) {
        size_t b = blank();
        if( b%cols > 0 ) {
            size_t p = b-1,
            t = tile(p);
            set_tile(t, b);
            set_blank(p);
        }
    }
    void random_moves(size_t rows, size_t cols, size_t n = 0) {
        for( size_t i = 0; i < n; ++i ) {
            size_t m = Random::uniform(4);
            if( m == 0 )
                up(rows, cols);
            else if( m == 1 )
                right(rows, cols);
            else if( m == 2 )
                down(rows, cols);
            else
                left(rows, cols);
        }
    }
    void set_goal(size_t rows, size_t cols) {
        d0_ = d1_ = d2_ = 0;
        d0_ |= (rows<<4);
        d0_ |= (cols<<8);
        set_blank(0);
        for( size_t t = 0; t < rows*cols; ++t )
            set_tile(t, 1+t);
    }
    size_t manhattan() const {
        size_t sum = 0;
        for( size_t p = 0; p < rows()*cols(); ++p )
            if( p != blank() ) {
                size_t t = tile(p);
                int r = ((1+t)/cols()) - (p/cols()), c = ((1+t)%cols()) - (p%cols());
                sum += (r<0?-r:r) + (c<0?-c:c);
            }
        return sum;
    }
    const state_t& operator=(const state_t &s) {
        d0_ = s.d0_;
        d1_ = s.d1_;
        d2_ = s.d2_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return (d0_ == s.d0_) && (d1_ == s.d1_) && (d2_ == s.d2_);
    }
    bool operator!=(const state_t &s) const {
        return (d0_ != s.d0_) || (d1_ != s.d1_) || (d2_ != s.d2_);
    }
    bool operator<(const state_t &s) const {
        return (d0_ < s.d0_) ||
               ((d0_ == s.d0_) && (d1_ < s.d1_)) ||
               ((d0_ == s.d0_) && (d1_ == s.d1_) && (d2_ < s.d2_));
    }
    void print(ostream &os) const {
        os << "|" << setw(2) << blank() << "|";
        for( size_t i = 0; i < rows() * cols(); ++i ) {
            os << (i == blank() ? 0 : 1+tile(i));
            os << (i < rows() * cols() - 1 ? "," : "|");
        }
    }
    friend class problem_t;
};

inline ostream& operator<<(ostream &os, const state_t &s) {
    s.print(os);
    return os;
}

class problem_t : public Problem::problem_t<state_t> {
    size_t rows_;
    size_t cols_;
    float p_;
    state_t init_;
    state_t goal_;

  public:
    problem_t(size_t rows, size_t cols, const state_t &init, float p = 1.0)
      : rows_(rows), cols_(cols), p_(p), init_(init) {
        goal_.set_goal(rows_, cols_);
    }
    virtual ~problem_t() { }

    virtual ::Problem::action_t number_actions() const { return 4; }
    virtual bool applicable(const state_t &s, ::Problem::action_t a) const {
        return s.applicable(rows_, cols_, a);
    }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return s == goal_;
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return terminal(s) ? 0 : 1;
    }
    virtual void next(const state_t &s, Problem::action_t a, pair<state_t,float> *outcomes, unsigned &osize) const {
        ++expansions_;
        unsigned i = 0;
        if( p_ > 0 ) {
            outcomes[i++] = make_pair(s, p_);
            if( a == ::up )
                outcomes[0].first.up(rows_, cols_);
            else if( a == ::right )
                outcomes[0].first.right(rows_, cols_);
            else if( a == ::down )
                outcomes[0].first.down(rows_, cols_);
            else
                outcomes[0].first.left(rows_, cols_);
        }
        if( 1-p_ > 0 ) {
            outcomes[i++] = make_pair(s, 1-p_);
#if 0
            if( a != up ) {
                outcomes[i] = make_pair(s, (1-p_)/3);
                outcomes[i++].first.up(rows_, cols_); 
            }
            if( a != right ) {
                outcomes[i] = make_pair(s, (1-p_)/3);
                outcomes[i++].first.right(rows_, cols_); 
            }
            if( a != down ) {
                outcomes[i] = make_pair(s, (1-p_)/3);
                outcomes[i++].first.down(rows_, cols_); 
            }
            if( a != left ) {
                outcomes[i] = make_pair(s, (1-p_)/3);
                outcomes[i++].first.left(rows_, cols_); 
            }
#endif
        }
        osize = i;
    }
    virtual void next(const state_t &s, Problem::action_t a, vector<pair<state_t,float> > &outcomes) const {
        pair<state_t, float> tmp[MAXOUTCOMES];
        unsigned osize = 0;
        next(s, a, &tmp[0], osize);
        outcomes.clear();
        outcomes.reserve(MAXOUTCOMES);
        for( unsigned i = 0; i < osize; ++i )
            outcomes.push_back(tmp[i]);
    }
    virtual void print(ostream &os) const { }
};

inline ostream& operator<<(ostream &os, const problem_t &p) {
    p.print(os);
    return os;
}

class manhattan_t : public Heuristic::heuristic_t<state_t> {
  public:
    manhattan_t() { }
    virtual ~manhattan_t() { }
    virtual float value(const state_t &s) const { return (float)s.manhattan(); }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(ostream &os) const { }
    float operator()(const state_t &s) const { return value(s); }
};

void usage(ostream &os) {
    os << endl
       << "usage: puzzle [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-k <n>] [-K <f>] [-p <f>] [-s <n>] [-v <n>] <rows> <cols>"
       << endl << endl
       << "  -a <n>    Algorithm bitmask: 1=vi, 2=slrtdp, 4=ulrtdp, 8=blrtdp, 16=ilao, 32=plain-check, 64=elrtdp, 128=hdp-i, 256=hdp, 512=ldfs+, 1024=ldfs."
       << endl
       << "  -b <n>    Visits bound for blrtdp. Default: 0."
       << endl
       << "  -e <f>    Epsilon. Default: 0."
       << endl
       << "  -f        Formatted output."
       << endl
       << "  -g <f>    Parameter for epsilon-greedy. Default: 0."
       << endl
       << "  -h <n>    Heuristics: 0=zero, 1=minmin, 2=hdp(0). Default: 0."
       << endl
#if 0
       << "  -k <n>    Kappa consistency level. Default: 0."
       << endl
       << "  -K <f>    Used to define kappa measures. Default: 2."
       << endl
#endif
       << "  -p <f>    Parameter p in [0,1]. Default: 1."
       << endl
       << "  -s <n>    Random seed. Default: 0."
       << endl
       << "  -v <n>    Verbosity. Default: 0."
       << endl
       << "  <rows>    Rows <= ?."
       << endl
       << "  <cols>    Cols <= ?."
       << endl << endl;
}

int main(int argc, const char **argv) {
    size_t rows = 0;
    size_t cols = 0;

    float p = 1.0;
    unsigned bitmap = 0;
    int h = 0;
    bool formatted = false;

    cout << fixed;
    Algorithm::parameters_t parameters;

    // parse arguments
    ++argv;
    --argc;
    while( argc > 1 ) {
        if( **argv != '-' ) break;
        switch( (*argv)[1] ) {
            case 'a':
                bitmap = strtoul(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            case 'b':
                parameters.rtdp.bound_ = strtol(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            case 'e':
                parameters.epsilon_ = strtod(argv[1], 0);
                argv += 2;
                argc -= 2;
                break;
            case 'f':
                formatted = true;
                ++argv;
                --argc;
                break;
            case 'g':
                parameters.rtdp.epsilon_greedy_ = strtod(argv[1], 0);
                argv += 2;
                argc -= 2;
                break;
            case 'h':
                h = strtol(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            case 'p':
                p = strtod(argv[1], 0);
                argv += 2;
                argc -= 2;
                break;
            case 's':
                parameters.seed_ = strtoul(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            default:
                usage(cout);
                exit(-1);
        }
    }

    if( argc == 2 ) {
        rows = strtoul(argv[0], 0, 0);
        cols = strtoul(argv[1], 0, 0);
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    Random::seeds(parameters.seed_);
    state_t init;
    init.set_goal(rows, cols);
    init.random_moves(rows, cols, 500);
    problem_t problem(rows, cols, init, p);

    // create heuristic
    Heuristic::heuristic_t<state_t> *heuristic = 0;
    if( h == 1 ) {
        heuristic = new Heuristic::min_min_heuristic_t<state_t>(problem);
    } else if( h == 2 ) {
        //heuristic = new Heuristic::hdp_heuristic_t<state_t>(problem, eps, 0);
    } else if( h == 3 ) {
        heuristic = new manhattan_t;
    } 

    // solve problem with algorithms
    vector<Dispatcher::result_t<state_t> > results;
    Dispatcher::solve(problem, heuristic, problem.init(), bitmap, parameters, results);

    // print results
    if( !results.empty() ) {
        cout << "seed=" << parameters.seed_ << endl;
        if( formatted ) Dispatcher::print_result<state_t>(cout, 0);
        for( unsigned i = 0; i < results.size(); ++i ) {
            Dispatcher::print_result(cout, &results[i]);
        }
    }

#if 0
    // evaluate policy
    unsigned ntrials = 500;
    unsigned depth = 100;
    unsigned width = 10;
    unsigned rdepth = 10;

    cout << "evaluation of policies:" << endl;

    start_time = Utils::read_time_in_seconds();
    cout << "  optimal=" << setprecision(5)
         << Policy::evaluation(Policy::hash_policy_t<state_t>(problem, hash),
                               problem.init(), ntrials, depth)
         << setprecision(2);
    cout << " " << Utils::read_time_in_seconds() - start_time << endl;

    if( heuristic != 0 ) {
        Policy::greedy_t<state_t> greedy(problem, *heuristic);
        for( int nesting = 0; nesting < 1; ++nesting ) {
            start_time = Utils::read_time_in_seconds();
            cout << "  nrollout(" << nesting << ", greedy)=" << setprecision(5)
                 << Policy::evaluation(Policy::nested_rollout_t<state_t>(problem, greedy, width, rdepth, nesting),
                                       problem.init(), ntrials, depth)
                 << setprecision(2);
            cout << " " << Utils::read_time_in_seconds() - start_time << endl;
        }
    }

    Policy::random_t<state_t> random(problem);
    for( int nesting = 0; nesting < 1; ++nesting ) {
        start_time = Utils::read_time_in_seconds();
        cout << "  nrollout(" << nesting << ", random)=" << setprecision(5)
             << Policy::evaluation(Policy::nested_rollout_t<state_t>(problem, random, width, rdepth, nesting),
                                   problem.init(), ntrials, depth)
             << setprecision(2);
        cout << " " << Utils::read_time_in_seconds() - start_time << endl;
    }

    Policy::mcts_t<state_t> uct(problem, random, 1e5, 100, -4); 
#if 0
    for( int i = 0; i < 1e8; ++i ) {
        uct.search_tree(problem.init(), 0);
        //uct.SEARCH(problem.init());
    }
    cout << "size=" << uct.size() << endl;
    //cout << "size=" << uct.SIZE() << endl;
    unsigned count_sum = 0;
    for( Problem::action_t a = 0; a < problem.number_actions(); ++a ) {
        count_sum += uct.count(problem.init(), a);
        cout << "  optimal(" << a << ")=" << setprecision(5)
             << hash.QValue(problem.init(), a) << endl;
        cout << "uct-value(" << a << ")=" << setprecision(5)
             << uct.value(problem.init(), a)
             << ", count=" << uct.count(problem.init(), a)
             << endl;
    }
    cout << "count-sum=" << count_sum << endl;
#endif

#if 1
    start_time = Utils::read_time_in_seconds();
    cout << "  uct(random)=" << setprecision(5)
         << Policy::evaluation(uct, problem.init(), ntrials, depth)
         << setprecision(2);
    cout << " " << Utils::read_time_in_seconds() - start_time << endl;
#endif

#endif

    // free resources
    for( unsigned i = 0; i < results.size(); ++i ) {
        delete results[i].hash_;
    }
    delete heuristic;

    exit(0);
}

