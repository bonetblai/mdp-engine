#include <cassert>
#include <iostream>
#include <math.h>

#include "wet.h"

#include "../evaluation.h"

using namespace std;

void usage(ostream &os) {
    os << "usage: wet [-a <n>] [-b <n>] [-e <f>] [-g <f>] [-h <n>] [-p <f>] [-s <n>] [-X] [-Y|-Z] <size>"
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

    if( argc == 10 ) {
        size = strtoul(*argv, 0, 0);
        policy = strtoul(argv[1], 0, 0);
        rollout_width = strtoul(argv[2], 0, 0);
        rollout_depth = strtoul(argv[3], 0, 0);
        rollout_nesting = strtoul(argv[4], 0, 0);
        uct_width = strtoul(argv[5], 0, 0);
        uct_depth = strtoul(argv[6], 0, 0);
        uct_parameter = strtod(argv[7], 0);
        ao_width = strtoul(argv[8], 0, 0);
        ao_depth = strtoul(argv[9], 0, 0);
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    cout << "seed=" << parameters.seed_ << endl;
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

