#include <iostream>
#include <iomanip>
#include <tr1/unordered_set>

#define MAXOUTCOMES   10

#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "mcts.h"
#include "dispatcher.h"

using namespace std;

const Problem::action_t onelfwd = 0;
const Problem::action_t onerfwd = 1;

class bits_t {
    unsigned bits_;

  public:
    bits_t(unsigned bits) : bits_(bits) { }
    ~bits_t() { }
    void print(ostream &os) const {
        for( unsigned i = 8 * sizeof(unsigned); i > 0; --i )
            os << ((bits_ >> (i-1)) % 2);
    }
};

inline ostream& operator<<(ostream &os, const bits_t &b) {
    b.print(os);
    return os;
}

class state_t {
    unsigned data1_;
    unsigned data2_;

  public:
    state_t() : data1_(0), data2_(0) { }
    state_t(const state_t &s) : data1_(s.data1_), data2_(s.data2_) { }
    ~state_t() { }
    size_t hash() const { return data1_ ^ data2_; }
    unsigned depth() const { return data1_ >> 26; }
    unsigned branch1() const { return data1_ & ~(63 << 26); }
    pair<unsigned, unsigned> branch() const {
        return make_pair(branch1(), data2_);
    }
    unsigned onebits() const {
        unsigned count = 0;
        pair<unsigned, unsigned> b = branch();
        for( unsigned i = 0; i < 8 * sizeof(unsigned); ++i ) {
            count += b.first % 2;
            b.first = b.first >> 1;
            count += b.second % 2;
            b.second = b.second >> 1;
        }
        return count;
    }
    void onelfwd(unsigned n) {
        if( depth() < n-1 ) {
            unsigned d = depth() + 1;
            data1_ = (d << 26) | branch1();
        }
    }
    void onerfwd(unsigned n) {
        if( depth() < n-1 ) {
            unsigned d = depth() + 1;
            data1_ = (d << 26) | branch1();
            if( depth() <= 32 )
                data2_ ^= (1 << (depth() - 1));
            else
                data1_ ^= (1 << (depth() - 33));
        }
    }
    void onebwd() {
        if( depth() > 0 ) {
            if( depth() <= 32 )
                data2_ &= ~(1 << (depth() - 1));
            else
                data1_ &= ~(1 << (depth() - 33));
            unsigned d = depth() - 1;
            data1_ = (d << 26) | branch1();
        }
    }
    const state_t& operator=(const state_t &s) {
        data1_ = s.data1_;
        data2_ = s.data2_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return (data1_ == s.data1_) && (data2_ == s.data2_);
    }
    bool operator!=(const state_t &s) const {
        return (data1_ != s.data1_) || (data2_ != s.data2_);
    }
    bool operator<(const state_t &s) const {
        return (data1_ < s.data1_) ||
               ((data1_ == s.data1_) && (data2_ < s.data2_));
    }
    void print(ostream &os) const {
        bits_t b1(branch1()), b2(data2_);
        os << "( " << depth()
           << " , [" << branch1()
           << "|" << data2_
           << "]:" << b1
           << "|" << b2 << " )";
    }
    friend class problem_t;
};

inline ostream& operator<<(ostream &os, const state_t &s) {
    s.print(os);
    return os;
}

class problem_t : public Problem::problem_t<state_t> {
    unsigned n_;
    float p_;
    float q_;
    float r_;
    state_t init_;
    tr1::unordered_set<state_t, Hash::hash_function_t<state_t> > noisy_;

  public:
    problem_t(unsigned n, float p, float q = 0.0, float r = 0.0)
      : n_(n), p_(p), q_(q), r_(r) {
        fill_noisy_states();
    }
    virtual ~problem_t() { }

    void fill_noisy_states() {
        if( r_ > 0.0 ) {
            Problem::hash_t<state_t> hash(*this);
            Algorithm::generate_space(*this, init_, hash);
            for( Problem::hash_t<state_t>::const_iterator hi = hash.begin(); hi != hash.end(); ++hi ) {
                if( Random::real() < r_ ) noisy_.insert((*hi).first);
            }
        }
    }
    bool noisy(const state_t &s) const {
        return r_ > 0.0 ? (noisy_.find(s) != noisy_.end()) : false;
    }

    virtual Problem::action_t number_actions() const { return 2; }
    virtual bool applicable(const state_t &s, ::Problem::action_t a) const {
        return true;
    }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return s.depth() == n_ - 1;
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return terminal(s) ? 0 : 1;
    }
    virtual void next(const state_t &s, Problem::action_t a, pair<state_t, float> *outcomes, unsigned &osize) const {
        ++expansions_;
        unsigned i = 0;
        if( a == Problem::noop ) {
            outcomes[i++] = make_pair(s, 1.0);
        } else {
            unsigned j = 0;
            float p = pow(p_, 1 + s.onebits());
            float q = noisy(s) ? p * q_ : 0.0;
            if( p - q > 0 ) outcomes[i++] = make_pair(s, p - q);
            if( q > 0 ) outcomes[i++] = make_pair(s, q);
            if( 1-p > 0 ) outcomes[i++] = make_pair(s, 1 - p);
            if( a == onelfwd ) {
                if( p - q > 0 ) outcomes[j++].first.onelfwd(n_);
                if( q > 0 ) outcomes[j++].first.onerfwd(n_);
            } else if( a == onerfwd ) {
                if( p - q > 0 ) outcomes[j++].first.onerfwd(n_);
                if( q > 0 ) outcomes[j++].first.onelfwd(n_);
            }
            if( 1 - p > 0 ) outcomes[j++].first.onebwd();
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

void evaluate_policies(const Problem::problem_t<state_t> &problem, const Heuristic::heuristic_t<state_t> *heuristic, const vector<Dispatcher::result_t<state_t> > &results) {

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
        cout << " " << Utils::read_time_in_seconds() - start_time << endl;
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
            cout << " " << Utils::read_time_in_seconds() - start_time << endl;
        }
    }

    // Rollouts wrt random base policy
    Policy::random_t<state_t> random(problem);
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
        cout << " " << Utils::read_time_in_seconds() - start_time << endl;
    }

    // UCT Policies wrt random base policy
    Policy::mcts_t<state_t> uct(problem, random, 1e4, 50, -.15); 

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

    start_time = Utils::read_time_in_seconds();
    cout << "  uct(random)=" << setprecision(5)
         << Policy::evaluation(uct,
                               problem.init(),
                               evaluation_trials,
                               evaluation_depth)
         << setprecision(2);
    cout << " " << Utils::read_time_in_seconds() - start_time << endl;
}

void usage(ostream &os) {
    os << "usage: tree [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-k <n>] [-K <f>] [-p <f>] [-q <f>] [-r <f>] [-s <n>] [-v <n>] <size>"
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
       << "  -q <f>    Parameter q in [0,1]. Default: 1/2."
       << endl
       << "  -r <f>    Parameter r in [0,1]. Default: 0."
       << endl
       << "  -s <n>    Random seed. Default: 0."
       << endl
       << "  -v <n>    Verbosity. Default: 0."
       << endl
       << "  <size>    Depth of tree <= 58."
       << endl << endl;
}

int main(int argc, const char **argv) {
    unsigned size = 0;
    float q = 0.5;
    float r = 0.0;

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
            case 'q':
                q = strtod(argv[1], 0);
                argv += 2;
                argc -= 2;
                break;
            case 'r':
                r = strtod(argv[1], 0);
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
        size = strtoul(argv[0], 0, 0);
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    Random::seeds(parameters.seed_);
    problem_t problem(size, p, q, r);

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
    evaluate_policies(problem, heuristic, results);

    // free resources
    for( unsigned i = 0; i < results.size(); ++i ) {
        delete results[i].hash_;
    }
    delete heuristic;

    exit(0);
}

