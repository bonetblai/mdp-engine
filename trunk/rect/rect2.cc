#include <iostream>
#include <iomanip>

#define MAXOUTCOMES   4

#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "mcts.h"
#include "dispatcher.h"

using namespace std;

const Problem::action_t fwd = 0;
const Problem::action_t left = 1;
const Problem::action_t right = 2;

typedef unsigned short ushort_t;

class state_t {
    ushort_t row_;
    ushort_t col_;

  public:
    state_t(ushort_t row = 0, ushort_t col = 0) : row_(row), col_(col) { }
    state_t(const state_t &s) : row_(s.row_), col_(s.col_) { }
    ~state_t() { }
    size_t hash() const { return row_ ^ col_; }
    unsigned row() const { return row_; }
    unsigned col() const { return col_; }
    void fwd(unsigned rows) { if( row() < rows - 1 ) ++row_; }
    void left(unsigned cols) { if( col() > 0 ) --col_; }
    void right(unsigned cols) { if( col() < cols - 1 ) ++col_; }
    const state_t& operator=( const state_t &s) {
        row_ = s.row_;
        col_ = s.col_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return (row_ == s.row_) && (col_ == s.col_);
    }
    bool operator!=(const state_t &s) const {
        return (row_ != s.row_) || (col_ != s.col_);
    }
    bool operator<(const state_t &s) const {
        return (row_ < s.row_) || ((row_ == s.row_) && (col_ < s.col_));
    }
    void print(ostream &os) const {
        os << "( " << row() << " , " << col() << " )";
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

  public:
    problem_t(unsigned rows, unsigned cols, float p = 1.0)
      : rows_(rows), cols_(cols), p_(p), init_(0, cols / 2) {
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions() const { return 3; }
    virtual bool applicable(const state_t &s, ::Problem::action_t a) const {
        return true;
    }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return s.row() == rows_ - 1;
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return terminal(s) ? 0 : 1;
    }
    virtual void next(const state_t &s, Problem::action_t a, pair<state_t, float> *outcomes, unsigned &osize) const {
        ++expansions_;
        unsigned i = 0;
        if( a != fwd ) {
            outcomes[i++] = make_pair(s, 1.0);
            if( a == ::left ) {
                outcomes[0].first.left(cols_);
            } else if( a == ::right ) {
                outcomes[0].first.right(cols_);
            }
        } else if( a == fwd ) {
            if( p_ > 0 ) {
                outcomes[i] = make_pair(s, p_);
                outcomes[i++].first.fwd(rows_);
            }
            if( 1 - p_ > 0 ) {
                outcomes[i] = make_pair(s, (1 - p_) / 2);
                outcomes[i++].first.left(cols_);
                outcomes[i] = make_pair(s, (1 - p_) / 2);
                outcomes[i++].first.right(cols_);
            }
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
    os << "usage: rect [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-p <f>] [-s <n>] <rows> <cols>"
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
       << "  <rows>    Rows <= 2^16."
       << endl
       << "  <cols>    Cols <= 2^16."
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
    problem_t problem(rows, cols, p);

    // create heuristic
    Heuristic::heuristic_t<state_t> *heuristic = 0;
    if( h == 1 ) {
        heuristic = new Heuristic::min_min_heuristic_t<state_t>(problem);
    } else if( h == 2 ) {
        //heuristic = new Heuristic::hdp_heuristic_t<state_t>(problem, eps, 0);
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
    evaluate_policies(problem, heuristic, results, 128, -.15);

    // free resources
    for( unsigned i = 0; i < results.size(); ++i ) {
        delete results[i].hash_;
    }
    delete heuristic;

    exit(0);
}

