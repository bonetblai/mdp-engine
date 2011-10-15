#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <math.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <float.h>
#include <assert.h>

#define  MAXOUTCOMES  10

#include "parsing.h"
#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"
#include "evaluation.h"

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
    virtual Problem::action_t last_action() const { return 9; }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return (s != init_) && grid_.goal_pos(s.x(), s.y());
    }
    virtual void next(const state_t &s, Problem::action_t a, pair<state_t,float> *outcomes, size_t &osize) const {
        ++expansions_;
        size_t i = 0;
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
                if( 1-p_ > 0.0 ) {
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
    virtual void next(const state_t &s, Problem::action_t a, vector<pair<state_t,float> > &outcomes) const { }
    virtual void print(ostream &os) const { }
};

void usage(ostream &os) {
    os << endl
       << "usage: race [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-k <n>] [-K <f>] [-p <f>] [-s <n>] [-v <n>] <file>"
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
       << "  <file>    Racetrack file."
       << endl << endl;
}

const char *name[] = {
    "vi", "slrtdp", "ulrtdp", "blrtdp", "ilao", "elrtdp", "check", "hdp-i", "hdp", "ldfs+", "ldfs"
};

size_t (*table[])(const Problem::problem_t<state_t>&, Problem::hash_t<state_t>&, const Algorithm::parameters_t&) = {
    Algorithm::value_iteration<state_t>,
    Algorithm::standard_lrtdp<state_t>,
    Algorithm::uniform_lrtdp<state_t>,
    Algorithm::bounded_lrtdp<state_t>,
    Algorithm::improved_lao<state_t>,
    Algorithm::standard_lrtdp<state_t>,
    Algorithm::plain_check<state_t>,
    0, //Algorithm::hdp_i<state_t>,
    Algorithm::hdp_driver<state_t>,
    Algorithm::ldfs_plus_driver<state_t>,
    Algorithm::ldfs_driver<state_t>,
    0
};

int main(int argc, const char **argv) {
    FILE *is = 0;
    float p = 1.0;
    unsigned alg = 0;
    int h = 0;
    unsigned long seed = 0;
    bool formatted = false;

    Algorithm::parameters_t parameters;

    // parse arguments
    ++argv;
    --argc;
    while( argc > 1 ) {
        if( **argv != '-' ) break;
        switch( (*argv)[1] ) {
            case 'a':
                alg = strtoul(argv[1], 0, 0);
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
                seed = strtoul(argv[1], 0, 0);
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
    }
    else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    Random::seeds(seed);
    grid_t grid;
    grid.parse(cout, is);
    problem_t problem(grid, p);
    fclose(is);

    // solve problem with algorithms
    unsigned first = numeric_limits<unsigned>::max();
    for( unsigned i = 0; (i < 12) && (table[i] != 0); ++i ) {
        if( (alg>>i) % 2 ) {
            Random::seeds(seed);
            first = Utils::min(first, i);
            Heuristic::heuristic_t<state_t> *heuristic = 0;

            if( h == 1 )
                heuristic = new Heuristic::min_min_heuristic_t<state_t>(problem);
            else if( h == 2 )
                ;//heuristic = new Heuristic::hdp_heuristic_t<state_t>(problem, eps, 0 );

            float start_time = Utils::read_time_in_seconds();
            Problem::hash_t<state_t> hash(problem, new Heuristic::wrapper_t<state_t>(heuristic));
            problem.clear_expansions();
            size_t t = (*table[i])(problem, hash, parameters);
            float end_time = Utils::read_time_in_seconds();
            float htime = !heuristic ? 0 : heuristic->total_time();
            float dtime = !heuristic ? 0 : heuristic->eval_time();
            float atime = end_time - start_time - dtime;

            if( formatted ) {
                if( i == first ) {
                    cout << setw(4) << "#" << " "
                              << setw(7) << "alg" << " "
                              << setw(12) << "V*(s0)" << " "
                              << setw(7) << "seed" << " "
                              << setw(12) << "trials" << " "
                              << setw(12) << "updates" << " "
                              << setw(12) << "expansions" << " "
                              << setw(12) << "h(s0)" << " "
                              << setw(12) << "hsize" << " "
                              << setw(12) << "psize" << " "
                              << setw(12) << "atime" << " "
                              << setw(12) << "htime" << endl;
                }
                cout << setw(4) << (1<<i) << " "
                          << setw(7) << name[i] << " "
                          << setw(12) << setprecision(9) << hash.value(problem.init()) << " "
                          << setw(7) << seed << " "
                          << setw(12) << t << " "
                          << setw(12) << hash.updates() << " "
                          << setw(12) << problem.expansions() << " "
                          << setw(12) << setprecision(9) << (!heuristic ? 0 : heuristic->value(problem.init())) << " "
                          << setw(12) << hash.size() << " "
                          << setw(12) << (i == 7 ? 0 : problem.policy_size(hash, problem.init())) << " "
                          << setw(12) << setprecision(9) << atime << " "
                          << setw(12) << setprecision(9) << htime << endl;
            }
            else {
                cout << (1<<i) << " "
                          << name[i] << " "
                          << setprecision(9) << hash.value(problem.init()) << " "
                          << seed << " "
                          << t << " "
                          << hash.updates() << " "
                          << problem.expansions() << " "
                          << setprecision(9) << (!heuristic ? 0 : heuristic->value(problem.init())) << " "
                          << hash.size() << " "
                          << (i == 7 ? 0 : problem.policy_size(hash, problem.init())) << " "
                          << setprecision(9) << atime << " "
                          << setprecision(9) << htime << endl;
            }
            delete heuristic;
        }
    }
    exit(0);
}

