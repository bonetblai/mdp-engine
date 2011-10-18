#include <cassert>
#include <iostream>
#include <math.h>

#define MAXOUTCOMES   10

#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "mcts.h"
#include "dispatcher.h"

using namespace std;

#define XVER          (version&0x1)
#define YVER          (version&0x2)
#define ZVER          (version&0x4)

int version = 0;
float kappa_table[] = { 0.0, 0.1, 0.3, 0.6 };

const Problem::action_t up = 1;
const Problem::action_t right = 2;
const Problem::action_t down = 3;
const Problem::action_t left = 4;
const char *asymb[] = { "-", "^", ">", "v", "<" };

class state_t {
    unsigned s_;
  protected:
    static size_t size_;
  public:
    state_t(unsigned x, unsigned y) : s_((x*size_) + y) { }
    state_t(unsigned s = 0) : s_(s) { }
    ~state_t() { }
    size_t hash() const { return s_; }
    unsigned s() const { return s_; }
    unsigned x() const { return s_ / size_; }
    unsigned y() const { return s_ % size_; }
    state_t move(Problem::action_t a) const {
        switch( a ) {
            case 1:
                if( y() == 0 ) return s();
                return state_t(x(), y() - 1);
                break;
            case 2:
                if( x() == size_ - 1 ) return s();
                return state_t(x() + 1, y());
                break;
            case 3:
                if( y() == size_ - 1 ) return s();
                return state_t(x(), y() + 1);
                break;
            case 4:
                if( x() == 0 ) return s();
                return state_t(x() - 1, y());
                break;
        }
        return s_;
    }

    const state_t& operator=(const state_t &s) {
        s_ = s.s_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return s_ == s.s_;
    }
    bool operator!=(const state_t &s) const {
        return s_ != s.s_;
    }
    bool operator<(const state_t &s) const {
        return s_ < s.s_;
    }
    void print(ostream &os) const {
        os << "( " << x() << " , " << y() << " )";
    }
    friend class problem_t;
};

size_t state_t::size_ = 0;

inline ostream& operator<<(ostream &os, const state_t &s) {
    s.print(os);
    return os;
}

class problem_t : public Problem::problem_t<state_t> {
    size_t size_;
    state_t init_;
    state_t goal_;
    char *water_;
    float p_;
  public:
    problem_t(size_t size, float p, const state_t &init = state_t(0,0), const state_t &goal = state_t(0,0))
     : size_(size), init_(init), goal_(goal), p_(p) {
        state_t::size_ = size_;
        water_ = new char[size_ * size_];
        for( size_t x = 0; x < size_; ++x )
            for( size_t y = 0; y < size_; ++y )
                if( Random::real() < p_ )
                    water_[(x * size_) + y] = 1 + (lrand48() % (XVER ? 2 : 3));
    }
    virtual ~problem_t() { delete[] water_; }
    size_t size() const { return size_; }
    unsigned water(size_t x, size_t y) const { return water_[(x * size_) + y]; }
    const state_t& goal() const { return goal_; }
    virtual Problem::action_t number_actions() const { return 5; }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s ) const { return s == goal_; }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return terminal(s) ? 0 : 1;
    }
    virtual void next(const state_t &s, Problem::action_t a, pair<state_t, float> *outcomes, unsigned &osize) const {
        ++expansions_;
        unsigned i = 0;
        float e = kappa_table[water(s.x(), s.y())];
        float e2 = e*e;
        if( YVER ) {
            outcomes[i++] = make_pair(s.move(a), 1.0-e-e2);                                    // kappa = 0
            if( e - e2 > 0.0 ) {
                outcomes[i++] = make_pair(s.move(1 + (a % 4)), (e - e2) / 2.0);                // kappa = 1
                outcomes[i++] = make_pair(s.move(1 + ((a+2) % 4)), (e - e2) / 2.0);            // kappa = 1
            }
            if( e2 > 0.0 ) outcomes[i++] = make_pair(s, 2 * e2);                               // kappa = 2
        } else if( ZVER ) {
            outcomes[i++] = make_pair(s.move(a), 1.0 - e2);                                    // kappa = 0
            if( e2 > 0.0 ) {
                outcomes[i++] = make_pair(s.move(1 +(a % 4)), e2 / 2.0);                       // kappa = 2
                outcomes[i++] = make_pair(s.move(1 +((a+2) % 4)), e2 / 2.0);                   // kappa = 2
            }
        } else {
            outcomes[i++] = make_pair(s.move(a), 1.0 - e - e2);                                // kappa = 0
            if( e - e2 > 0.0 ) {
                outcomes[i++] = make_pair((s.move(a)).move(1 + (a % 4)), (e - e2) / 4.0 );     // kappa = 1
                outcomes[i++] = make_pair((s.move(a)).move(1 + ((a+2) % 4)), (e - e2) / 4.0 ); // kappa = 1
                outcomes[i++] = make_pair(s.move(1 + (a % 4)), (e - e2) / 4.0 );               // kappa = 1
                outcomes[i++] = make_pair(s.move(1 + ((a+2) % 4)), (e - e2) / 4.0 );           // kappa = 1
            }
            if( e2 > 0.0 ) outcomes[i++] = make_pair(s, 2 * e2);                               // kappa = 2
        }
        osize = i;
    }
    virtual void next(const state_t &s, Problem::action_t a, vector<pair<state_t, float> > &outcomes) const {
        pair<state_t, float> tmp[MAXOUTCOMES];
        unsigned osize = 0;
        next(s, a, &tmp[0], osize);
        outcomes.clear();
        outcomes.reserve(MAXOUTCOMES);
        for( unsigned i = 0; i < osize; ++i )
            outcomes.push_back(tmp[i]);
    }
    virtual void print(ostream &os) const {
        os << "size = " << size_ << endl
           << "init = " << init_ << endl
           << "goal = " << goal_ << endl;
        for( size_t x = 0; x < size_; ++x ) os << "--";
        os << "---" << endl;
        for( size_t y = 0; y < size_; ++y ) {
            os << "|";
            for( size_t x = 0; x < size_; ++x ) {
                os << " " << water(x,y);
            }
            os << " |" << endl;
        }
        for( size_t x = 0; x < size_; ++x ) os << "--";
        os << "---" << endl;
    }
    void print_solution(ostream &os, const Problem::hash_t<state_t> &hash) const {
        for( size_t x = 0; x < size_; ++x ) os << "--";
        os << "---" << endl;
        for( size_t y = 0; y < size_; ++y ) {
            os << "|";
            for( size_t x = 0; x < size_; ++x ) {
                if( terminal(state_t(x, y)) ) {
                    os << " *";
                } else {
                    pair<Problem::action_t, float> p = hash.bestQValue(state_t(x, y));
                    os << " " << asymb[p.first];
                }
            }
            os << " |" << endl;
        }
        for( size_t x = 0; x < size_; ++x ) os << "--";
        os << "---" << endl;
    }
};

inline ostream& operator<<(ostream &os, const problem_t &p) {
    p.print(os);
    return os;
}

void
usage(ostream &os) {
    os << "usage: wet [-a <n>] [-b <n>] [-e <f>] [-g <f>] [-h <n>] [-p <f>] [-s <n>] [-v <n>] [-X] [-Y|-Z] <size>"
       << endl;
}

int main(int argc, const char **argv) {
    size_t size = 0;

    float p = 0.0;
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
            case 'X':
            case 'Y':
            case 'Z':
                version += 1 << (int)((*argv)[1] - 'X');
                ++argv;
                --argc;
                if( ZVER && YVER ) {
                    cout << "Y and Z cannot be used simultaneously." << endl;
                    exit(-1);
                }
                break;
            default:
                usage(cout);
                exit(-1);
        }
    }

    if( argc == 1 ) {
        size = strtoul(*argv, 0, 0);
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    Random::seeds(parameters.seed_);
    state_t init(Random::uniform(size), Random::uniform(size));
    state_t goal(Random::uniform(size), Random::uniform(size));
    problem_t problem(size, p, init, goal);
    //problem.print(cout);

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

