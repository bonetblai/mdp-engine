#include <iostream>
#include <iomanip>
#include <tr1/unordered_set>

#include "tree.h"

using namespace std;

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
    os << "usage: tree [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-p <f>] [-q <f>] [-r <f>] [-s <n>] <size>"
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
       << "  -q <f>    Parameter q in [0,1]. Default: 1/2."
       << endl
       << "  -r <f>    Parameter r in [0,1]. Default: 0."
       << endl
       << "  -s <n>    Random seed. Default: 0."
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

