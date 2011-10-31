#include <cassert>
#include <iostream>
#include <iomanip>

#define DISCOUNT   .95

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
    unsigned bits_;
  public:
    bits_t(unsigned bits) : bits_(bits) { }
    ~bits_t() { }
    void print(ostream &os) const {
        for( unsigned i = 8*sizeof(unsigned); i > 0; --i )
            os << ((bits_ >> (i-1)) % 2);
    }
};

inline ostream& operator<<(ostream &os, const bits_t &b) {
    b.print(os);
    return os;
}

class state_t {
    unsigned d0_;
    unsigned d1_;
    unsigned d2_;
  public:
    state_t(unsigned d0 = 0, unsigned d1 = 0, unsigned d2 = 0) : d0_(d0), d1_(d1), d2_(d2) { }
    state_t(const state_t &s) : d0_(s.d0_), d1_(s.d1_), d2_(s.d2_) { }
    ~state_t() { }
    size_t hash() const { return d1_ ^ d2_; }
    unsigned rows() const { return (d0_>>4) & 0xF; }
    unsigned cols() const { return (d0_>>8) & 0xF; }
    bool applicable(unsigned rows, unsigned cols, Problem::action_t a) const {
        switch( a ) {
            case ::up: return blank() >= cols;
            case ::right: return blank() % cols < cols-1;
            case ::down: return blank() + cols < rows*cols;
            case ::left: return blank() % cols > 0;
        }
        return false;
    }
    void set_tile(unsigned t, unsigned pos) {
        unsigned shift = pos < 8 ? pos << 2 : (pos - 8) << 2;
        if( pos < 8 ) {
            d1_ &= ~(0xF << shift);
            d1_ |= t<<shift;
        } else {
            d2_ &= ~(0xF << shift);
            d2_ |= t << shift;
        }
    }
    void set_blank(unsigned pos) {
        d0_ &= ~(0xF);
        d0_ |= pos;
        set_tile(0, pos);
    }
    unsigned blank() const {
        return d0_ & 0xF;
    }
    unsigned tile(unsigned pos) const {
        unsigned shift = pos < 8 ? pos << 2 : (pos - 8) << 2;
        return pos < 8 ? (d1_ >> shift) & 0xF : (d2_ >> shift) & 0xF;
    }
    void up(unsigned rows, unsigned cols) {
        unsigned b = blank();
        if( b >= cols ) {
            unsigned p = b-cols, t = tile(p);
            set_tile(t, b);
            set_blank(p);
        }
    }
    void right(unsigned rows, unsigned cols) {
        unsigned b = blank();
        if( b%cols < cols-1 ) {
            unsigned p = b+1, t = tile(p);
            set_tile(t, b);
            set_blank(p);
        }
    }
    void down(unsigned rows, unsigned cols) {
        unsigned b = blank();
        if( b+cols < rows*cols ) {
            unsigned p = b+cols, t = tile(p);
            set_tile(t, b);
            set_blank(p);
        }
    }
    void left(unsigned rows, unsigned cols) {
        unsigned b = blank();
        if( b%cols > 0 ) {
            unsigned p = b-1,
            t = tile(p);
            set_tile(t, b);
            set_blank(p);
        }
    }
    void random_moves(unsigned rows, unsigned cols, unsigned n = 0) {
        for( unsigned i = 0; i < n; ++i ) {
            unsigned m = Random::uniform(4);
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
    void set_goal(unsigned rows, unsigned cols) {
        d0_ = d1_ = d2_ = 0;
        d0_ |= (rows<<4);
        d0_ |= (cols<<8);
        set_blank(0);
        for( unsigned t = 0; t < rows*cols; ++t )
            set_tile(t, 1+t);
    }
    unsigned manhattan() const {
        unsigned sum = 0;
        for( unsigned p = 0; p < rows()*cols(); ++p )
            if( p != blank() ) {
                unsigned t = tile(p);
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
        for( unsigned i = 0; i < rows() * cols(); ++i ) {
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
    unsigned rows_;
    unsigned cols_;
    float p_;
    state_t init_;
    state_t goal_;

  public:
    problem_t(unsigned rows, unsigned cols, const state_t &init, float p = 1.0)
      : rows_(rows), cols_(cols), p_(p), init_(init) {
        goal_.set_goal(rows_, cols_);
    }
    virtual ~problem_t() { }

    virtual ::Problem::action_t number_actions() const { return 4; }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return s == goal_;
    }
    virtual bool applicable(const state_t &s, ::Problem::action_t a) const {
        return s.applicable(rows_, cols_, a);
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return terminal(s) ? 0 : 1;
    }
    virtual void next(const state_t &s, Problem::action_t a, pair<state_t,float> *outcomes, unsigned &osize) const { }
    virtual void next(const state_t &s, Problem::action_t a, vector<pair<state_t,float> > &outcomes) const {
        ++expansions_;
        outcomes.clear();
        outcomes.reserve(2);
        if( p_ > 0 ) {
            outcomes.push_back(make_pair(s, p_));
            if( a == ::up ) {
                outcomes.back().first.up(rows_, cols_);
            } else if( a == ::right ) {
                outcomes.back().first.right(rows_, cols_);
            } else if( a == ::down ) {
                outcomes.back().first.down(rows_, cols_);
            } else {
                outcomes.back().first.left(rows_, cols_);
            }
        }
        if( 1-p_ > 0 ) {
            outcomes.push_back(make_pair(s, 1 - p_));
        }
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

void evaluate_policies(const Problem::problem_t<state_t> &problem, const Heuristic::heuristic_t<state_t> *heuristic, const vector<Dispatcher::result_t<state_t> > &results, unsigned max_width, float uct_parameter) {

    unsigned evaluation_trials = 100;
    unsigned evaluation_depth = 50;
    unsigned rollout_width = 50;
    unsigned rollout_depth = 50;
    float start_time = 0;

    cout << "evaluation of policies:" << endl;

    // Optimal policy (if available)
    if( !results.empty() ) {
        const Problem::hash_t<state_t> &hash = *results[0].hash_;
        start_time = Utils::read_time_in_seconds();
        cout << "  optimal=" << setprecision(5)
             << Policy::evaluation(Policy::hash_policy_t<state_t>(problem, hash),
                                   problem.init(),
                                   evaluation_trials,
                                   evaluation_depth)
             << setprecision(2);
        cout << " (" << Utils::read_time_in_seconds() - start_time << " secs)" << endl;
    }

    // Rollouts wrt heuristic greedy (base) policy (if available)
    if( heuristic != 0 ) {
        Policy::greedy_t<state_t> greedy(problem, *heuristic);
        start_time = Utils::read_time_in_seconds();
        for( int nesting = 0; nesting < 2; ++nesting ) {
            start_time = Utils::read_time_in_seconds();
            cout << "  nrollout(" << nesting << ", greedy)=" << setprecision(5)
                 << Policy::evaluation(Policy::nested_rollout_t<state_t>(problem,
                                                                         greedy,
                                                                         rollout_width,
                                                                         rollout_depth,
                                                                         nesting),
                                       problem.init(),
                                       evaluation_trials,
                                       evaluation_depth)
                 << setprecision(2);
            cout << " (" << Utils::read_time_in_seconds() - start_time << " secs)" << endl;
        }
    }

    // Random policy
    Policy::random_t<state_t> random(problem);
    start_time = Utils::read_time_in_seconds();
    cout << "  random=" << setprecision(5)
         << Policy::evaluation(random,
                               problem.init(),
                               evaluation_trials,
                               evaluation_depth)
         << setprecision(2);
    cout << " (" << Utils::read_time_in_seconds() - start_time << " secs)" << endl;

    // Rollouts wrt random base policy
    for( int nesting = 0; nesting < 1; ++nesting ) {
        start_time = Utils::read_time_in_seconds();
        cout << "  nrollout(" << nesting << ", random)=" << setprecision(5)
             << Policy::evaluation(Policy::nested_rollout_t<state_t>(problem,
                                                                     random,
                                                                     rollout_width,
                                                                     rollout_depth,
                                                                     nesting),
                                   problem.init(),
                                   evaluation_trials,
                                   evaluation_depth)
             << setprecision(2);
        cout << " (" << Utils::read_time_in_seconds() - start_time << " secs)" << endl;
    }

    // UCT Policies wrt random base policy
    for( unsigned width = 2; width <= max_width; width *= 2 ) {
        Policy::mcts_t<state_t> uct(problem, random, width, 50, uct_parameter); 
        start_time = Utils::read_time_in_seconds();
        cout << "  uct(random, width=" << width << ", p=" << uct_parameter << ")="
             << setprecision(5)
             << Policy::evaluation(uct,
                                   problem.init(),
                                   evaluation_trials,
                                   evaluation_depth)
             << setprecision(2);
        cout << " (" << Utils::read_time_in_seconds() - start_time << " secs)" << endl;
    }
}

void usage(ostream &os) {
    os << endl
       << "usage: puzzle [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-p <f>] [-s <n>] <rows> <cols>"
       << endl << endl
       << "  -a <n>    Algorithm bitmask: 1=vi, 2=slrtdp, 4=ulrtdp, 8=blrtdp, 16=ilao, 32=plain-check, 64=elrtdp, 128=hdp-i, 256=hdp, 512=ldfs+, 1024=ldfs."
       << endl
       << "  -b <n>    Visits bound for blrtdp. Default: inf."
       << endl
       << "  -e <f>    Epsilon. Default: 0."
       << endl
       << "  -f        Formatted output."
       << endl
       << "  -g <f>    Parameter for epsilon-greedy. Default: 0."
       << endl
       << "  -h <n>    Heuristics: 0=zero, 1=minmin. Default: 0."
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
       << "  <rows>    Rows <= ?."
       << endl
       << "  <cols>    Cols <= ?."
       << endl << endl;
}

int main(int argc, const char **argv) {
    unsigned rows = 0;
    unsigned cols = 0;

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

    // evaluate policies
    //evaluate_policies(problem, heuristic, results, 128, -.15);

    // free resources
    for( unsigned i = 0; i < results.size(); ++i ) {
        delete results[i].hash_;
    }
    delete heuristic;

    exit(0);
}

