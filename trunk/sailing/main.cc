#include <iostream>
#include <iomanip>
#include <strings.h>

#include "sailing.h"
#include "ao.h"
#include "ao2.h"

using namespace std;

unsigned evaluation_trials = 200;
unsigned evaluation_depth = 70;

unsigned rollout_width = 50;
unsigned rollout_depth = 50;
unsigned rollout_nesting = 3;

unsigned uct_width = 32;
unsigned uct_depth = 50;
float uct_parameter = -0.15;

unsigned ao_width = 32;
unsigned ao_depth = 50;

void evaluate_policy(const Policy::policy_t<state_t> &policy) {
    float start_time = Utils::read_time_in_seconds();
    cout << setprecision(5);
    cout << Policy::evaluation(policy, policy.problem().init(), evaluation_trials, evaluation_depth);
    cout << setprecision(2);
    cout << " (" << Utils::read_time_in_seconds() - start_time << " secs)" << endl;
}
 
void evaluate_hash_policy(const Problem::hash_t<state_t> *hash, const char *name) {
    if( hash == 0 ) {
        cout << "  " << name << "=<not-available>" << std::endl;
    } else {
        cout << "  " << name << "=";
        Policy::hash_policy_t<state_t> policy(*hash);
        evaluate_policy(policy);
    }
}

void evaluate_rollout_policy(const Policy::policy_t<state_t> &base, const char *name) {
    Policy::nested_rollout_t<state_t> policy(base, rollout_width, rollout_depth, rollout_nesting);
    cout << "  nrollout(" << name << ", nesting=" << rollout_nesting << ")=";
    evaluate_policy(policy);
}

void evaluate_uct_policy(const Policy::policy_t<state_t> &base, const char *name) {
    Policy::mcts_t<state_t> policy(base, uct_width, uct_depth, uct_parameter); 
    cout << "  uct(" << name << ", width=" << uct_width << ", depth=" << uct_depth << ", p=" << uct_parameter << ")=";
    evaluate_policy(policy);
}

void evaluate_ao_policy(const Policy::policy_t<state_t> &base, const char *name) {
    Policy::ao2_t<state_t> policy(base, ao_width, ao_depth); 
    cout << "  ao2(" << name << ", width=" << ao_width << ", depth=" << ao_depth << ")=";
    evaluate_policy(policy);
}

void evaluate_policies(const Problem::problem_t<state_t> &problem, const Problem::hash_t<state_t> *hash, const Heuristic::heuristic_t<state_t> *heuristic) {

    cout << "evaluation of policies:" << endl;

    // Optimal policy (if available)
    evaluate_hash_policy(hash, "optimal");

    // Greedy policy wrt heuristic (if available)
    if( heuristic != 0 ) {
        Policy::greedy_t<state_t> greedy_policy(problem, *heuristic);

        // Greedy policy
        cout << "  greedy=";
        evaluate_policy(greedy_policy);

        // Rollouts
        evaluate_rollout_policy(greedy_policy, "greedy");

        // UCT
        evaluate_uct_policy(greedy_policy, "greedy");

        // AO
        evaluate_ao_policy(greedy_policy, "greedy");
    }

    // Random policy
    Policy::random_t<state_t> random_policy(problem);

    cout << "  random=";
    evaluate_policy(random_policy);

    // Rollouts
    evaluate_rollout_policy(random_policy, "random");

    // UCT
    evaluate_uct_policy(random_policy, "random");

    // AO
    evaluate_ao_policy(random_policy, "random");
}

void usage(ostream &os) {
    os << "usage: sailing [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-s <n>] <rows> <cols>"
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
        if( formatted ) Dispatcher::print_result<state_t>(cout, 0);
        for( unsigned i = 0; i < results.size(); ++i ) {
            Dispatcher::print_result(cout, &results[i]);
        }
    }

    // evaluate policies
    const Problem::hash_t<state_t> *hash = results.empty() ? 0 : results[0].hash_;
    evaluate_policies(problem, hash, heuristic);

    // free resources
    for( unsigned i = 0; i < results.size(); ++i ) {
        delete results[i].hash_;
    }
    delete heuristic;

    exit(0);
}

