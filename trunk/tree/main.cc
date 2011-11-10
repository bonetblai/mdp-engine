#include <iostream>
#include <vector>

#include "tree.h"

#include "../evaluation.h"

using namespace std;

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

    if( argc == 11 ) {
        size = strtoul(argv[0], 0, 0);
        policy = strtoul(argv[1], 0, 0);
        rollout_width = strtoul(argv[2], 0, 0);
        rollout_depth = strtoul(argv[3], 0, 0);
        rollout_nesting = strtoul(argv[4], 0, 0);
        uct_width = strtoul(argv[5], 0, 0);
        uct_depth = strtoul(argv[6], 0, 0);
        uct_parameter = strtod(argv[7], 0);
        ao_width = strtoul(argv[8], 0, 0);
        ao_depth = strtoul(argv[9], 0, 0);
        ao_parameter = strtod(argv[10], 0);
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    cout << "seed=" << parameters.seed_ << endl;
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
    const Problem::hash_t<state_t> *hash = results.empty() ? 0 : results[0].hash_;
    evaluate_policy(policy, problem, hash, heuristic);
    //evaluate_all_policies(problem, hash, heuristic);

    // free resources
    for( unsigned i = 0; i < results.size(); ++i ) {
        delete results[i].hash_;
    }
    delete heuristic;

    exit(0);
}

