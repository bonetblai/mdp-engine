#include <cassert>
#include <iostream>
#include <map>
#include <vector>

#define  MAXOUTCOMES  10

#include "parsing.h"
#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "mcts.h"
#include "dispatcher.h"

using namespace std;

class state_t {
    short x_;
    short y_;
    short dx_;
    short dy_;

  public:
    state_t(short x = 0, short y = 0, short dx = 0, short dy = 0)
      : x_(x), y_(y), dx_(dx), dy_(dy) {
    }
    state_t(const state_t &s)
      : x_(s.x_), y_(s.y_), dx_(s.dx_), dy_(s.dy_) {
    }
    ~state_t() { }

    short x() const { return x_; }
    short y() const { return y_; }
    short dx() const { return dx_; }
    short dy() const { return dy_; }

    size_t hash() const {
        return (x_ | (y_<<16)) ^ (dx_ | (dy_<<16));
    }

    const state_t& operator=(const state_t &s) {
        x_ = s.x_;
        y_ = s.y_;
        dx_ = s.dx_;
        dy_ = s.dy_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return (x_ == s.x_) && (y_ == s.y_) && (dx_ == s.dx_) && (dy_ == s.dy_);
    }
    bool operator!=(const state_t &s) const {
        return (x_ != s.x_) || (y_ != s.y_) || (dx_ != s.dx_) || (dy_ != s.dy_);
    }
    bool operator<(const state_t &s) const {
        return (x_ < s.x_) ||
               ((x_ == s.x_) && (y_ < s.y_)) ||
               ((x_ == s.x_) && (y_ == s.y_) && (dx_ < s.dx_)) ||
               ((x_ == s.x_) && (y_ == s.y_) && (dx_ == s.dx_) && (dy_ < s.dy_));
    }
    void print(ostream &os) const {
        os << "(" << x_ << "," << y_ << "," << dx_ << "," << dy_ << ")";
    }
    friend class problem_t;
};

inline ostream& operator<<(ostream &os, const state_t &s) {
    s.print(os);
    return os;
}

class ecache_t : public tr1::unordered_map<size_t, pair<state_t, state_t> > { };
//class ecache_t : public map<size_t,pair<state_t,state_t> > { };

class problem_t : public Problem::problem_t<state_t> {
    const grid_t &grid_;
    float p_;
    size_t rows_;
    size_t cols_;
    state_t init_;
    vector<state_t> inits_;
    vector<state_t> goals_;
    mutable ecache_t *ecache_[9];

  public:
    problem_t(grid_t &grid, float p = 1.0)
      : grid_(grid), p_(p), rows_(grid.rows()), cols_(grid.cols()),
        init_(numeric_limits<short>::max(), numeric_limits<short>::max(),
              numeric_limits<short>::max(), numeric_limits<short>::max()) {
        for( size_t i = 0; i < grid_.starts().size(); ++i ) {
            size_t s = grid_.start(i);
            inits_.push_back(state_t(s / cols_, s % cols_));
        }
        for( size_t i = 0; i < grid_.goals().size(); ++i ) {
            size_t s = grid_.goal(i);
            goals_.push_back(state_t(s / cols_, s % cols_));
        }
        for( size_t i = 0; i < 9; ++i )
            ecache_[i] = new ecache_t[rows_*cols_];
    }
    virtual ~problem_t() {
        for( size_t i = 0; i < 9; ++i )
            delete[] ecache_[i];
    }
    virtual Problem::action_t number_actions() const { return 9; }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return (s != init_) && grid_.goal_pos(s.x(), s.y());
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return terminal(s) ? 0 : 1;
    }
    virtual void next(const state_t &s, Problem::action_t a, pair<state_t,float> *outcomes, unsigned &osize) const {
        ++expansions_;
        unsigned i = 0;
        if( s == init_ ) {
            for( size_t j = 0; j < inits_.size(); ++j )
                outcomes[i++] = make_pair(inits_[j], 1.0/(float)inits_.size());
        } else {
            size_t off = s.x() * cols_ + s.y();
            size_t key = (((unsigned short)s.dx())<<16) | (unsigned short)s.dy();
            ecache_t::const_iterator ci = ecache_[a][off].find(key);
            if( ci != ecache_[a][off].end() ) {
                outcomes[i++] = make_pair((*ci).second.first, p_);
                outcomes[i++] = make_pair((*ci).second.second, 1 - p_);
            } else {
                pair<state_t,state_t> entry;
                short ox = 0, oy = 0, ux = (a/3)-1, uy = (a%3)-1;
                if( p_ > 0.0 ) {
                    int dx = s.dx() + ux, dy = s.dy() + uy;
                    int x = s.x() + dx, y = s.y() + dy;
                    int rv = grid_.valid_path(s.x(), s.y(), x, y, ox, oy);
                    if( rv == 0 )
                        entry.first = state_t(x, y, dx, dy);
                    else
                        entry.first = state_t(ox, oy, 0, 0);
                    outcomes[i++] = make_pair(entry.first, p_);
                }
                if( 1 - p_ > 0.0 ) {
                    int dx = s.dx(), dy = s.dy();
                    int x = s.x() + dx, y = s.y() + dy;
                    int rv = grid_.valid_path(s.x(), s.y(), x, y, ox, oy);
                    if( rv == 0 )
                        entry.second = state_t(x, y, dx, dy);
                    else
                        entry.second = state_t(ox, oy, 0, 0);
                    outcomes[i++] = make_pair(entry.second, 1 - p_);
                }
                ecache_[a][off].insert(make_pair(key, entry));
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
    os << "usage: race [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-p <f>] [-s <n>] <file>"
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
       << "  <file>    Racetrack file."
       << endl << endl;
}

int main(int argc, const char **argv) {
    FILE *is = 0;

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

    if( argc == 1 ) {
        is = fopen(argv[0], "r");
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    Random::seeds(parameters.seed_);
    grid_t grid;
    grid.parse(cout, is);
    problem_t problem(grid, p);
    fclose(is);

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

