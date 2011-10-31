#include <iostream>
#include <iomanip>
#include <strings.h>

#define DISCOUNT 1

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
    short wind_;

  public:
    enum { Away = 0, Down = 1, Cross = 2, Up = 3, Into = 4 }; // tacks

  public:
    state_t(short x = 0, short y = 0, short wind = 0)
      : x_(x), y_(y), wind_(wind) { }
    state_t(const state_t &s)
      : x_(s.x_), y_(s.y_), wind_(s.wind_) { }
    ~state_t() { }

    size_t hash() const {
        return (x_ << ((8*sizeof(short)) + 3)) | (y_ << 3) | wind_;
    }

    int tack(Problem::action_t a) const {
        int d = Utils::abs<int>(a - wind_);
        return d < 8 - d ? d : 8 - d;
    }
    bool in_lake(short rows, short cols) const {
        return (x_ >= 0) && (x_ < rows) && (y_ >= 0) && (y_ < cols);
    }
    pair<int, int> direction(Problem::action_t a) const {
        switch( a ) {
            case 0: return make_pair(0, 1);
            case 1: return make_pair(1, 1);
            case 2: return make_pair(1, 0);
            case 3: return make_pair(1, -1);
            case 4: return make_pair(0, -1);
            case 5: return make_pair(-1, -1);
            case 6: return make_pair(-1, 0);
            case 7: return make_pair(-1, 1);
            default: return make_pair(-1, -1);
        }
    }

    state_t apply(Problem::action_t a) const {
        pair<int, int> dir = direction(a);
        return state_t(x_ + dir.first, y_ + dir.second);
    }

    const state_t& operator=( const state_t &s) {
        x_ = s.x_;
        y_ = s.y_;
        wind_ = s.wind_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return (x_ == s.x_) && (y_ == s.y_) && (wind_ == s.wind_);
    }
    bool operator!=(const state_t &s) const {
        return (x_ != s.x_) || (y_ != s.y_) || (wind_ != s.wind_);
    }
    bool operator<(const state_t &s) const {
        return (x_ < s.x_) ||
               ((x_ == s.x_) && (y_ < s.y_)) ||
               ((x_ == s.x_) && (y_ == s.y_) && (wind_ < s.wind_));
    }
    void print(ostream &os) const {
        os << "(" << x_ << "," << y_ << "," << wind_ << ")";
    }
    friend class problem_t;
};

inline ostream& operator<<(ostream &os, const state_t &s) {
    s.print(os);
    return os;
}

class problem_t : public Problem::problem_t<state_t> {
    int rows_;
    int cols_;
    float wind_transition_[64];
    float costs_[5];
    state_t init_;
    state_t goal_;

    static const float default_wind_transition_[];
    static const float default_costs_[];

  public:
    problem_t(int rows, int cols,
              int init_x = 0, int init_y = 0,
              int goal_x = numeric_limits<int>::max(),
              int goal_y = numeric_limits<int>::max(),
              float *wind_transition = 0, float *costs = 0)
      : rows_(rows), cols_(cols), init_(init_x, init_y), goal_(goal_x, goal_y) {

        if( (goal_x == numeric_limits<int>::max()) ||
            (goal_y == numeric_limits<int>::max()) ) {
            goal_ = state_t(rows - 1, cols - 1);
        }
        
        if( wind_transition != 0 ) {
            bcopy(wind_transition, wind_transition_, 64 * sizeof(float));
        } else {
            bcopy(default_wind_transition_, wind_transition_, 64 * sizeof(float));
        }

        if( costs != 0 ) {
            bcopy(costs, costs_, 5 * sizeof(float));
        } else {
            bcopy(default_costs_, costs_, 5 * sizeof(float));
        }
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions() const { return 8; }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return (s.x_ == goal_.x_) && (s.y_ == goal_.y_);
    }
    virtual bool applicable(const state_t &s, ::Problem::action_t a) const {
        return s.tack(a) == state_t::Into ? false : s.apply(a).in_lake(rows_, cols_);
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return terminal(s) ? 0 : costs_[s.tack(a)];
    }
    virtual void next(const state_t &s, Problem::action_t a, pair<state_t, float> *outcomes, unsigned &osize) const { }
    virtual void next(const state_t &s, Problem::action_t a, vector<pair<state_t,float> > &outcomes) const {
        ++expansions_;
        outcomes.clear();
        outcomes.reserve(8);
        state_t next_s = s.apply(a);
        assert(next_s.in_lake(rows_, cols_));
        for( int nwind = 0; nwind < 8; ++nwind ) {
            float p = wind_transition_[s.wind_ * 8 + nwind];
            if( p > 0 ) {
                next_s.wind_ = nwind;
                outcomes.push_back(make_pair(next_s, p));
            }
        }
    }
    virtual void print(ostream &os) const { }
};

const float problem_t::default_wind_transition_[] = {
    0.4, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3,
    0.4, 0.3, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.4, 0.3, 0.3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.4, 0.3, 0.3, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.4, 0.2, 0.4, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.4, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.4,
    0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3
};

const float problem_t::default_costs_[] = {
    1, 2, 3, 4, numeric_limits<float>::max()
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
    os << "usage: sailing [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-p <f>] [-s <n>] <rows> <cols>"
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
    problem_t problem(rows, cols);

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

        const Problem::hash_t<state_t> &hash = *results[0].hash_;
        float value = 0;
        for( Problem::hash_t<state_t>::const_iterator it = hash.begin(); it != hash.end(); ++it ) {
            value += it->second->value();
        }
        value /= hash.size();
        cout << "avg value = " << value << endl;
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

