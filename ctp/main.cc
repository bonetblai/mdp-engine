#include <iostream>
#include <fstream>
#include <vector>

#include "ctp.h"

#include "../evaluation.h"

using namespace std;

void usage(ostream &os) {
    os << "usage: ctp [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-s <n>] <file>"
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
       << "  <file>    Racetrack file."
       << endl << endl;
}

int main(int argc, const char **argv) {
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

    CTP::graph_t graph;
    if( (argc == 11) || (argc == 12) ) {
        ifstream is(argv[0], ifstream::in);
        if( !graph.parse(is) ) exit(-1);
        is.close();
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
        if( argc == 12 ) ao_expansions_per_iteration = strtod(argv[11], 0);
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    cout << "seed=" << parameters.seed_ << endl;
    Random::seeds(parameters.seed_);
    problem_t problem(graph);
    cout << "P(bad weather)=" << probability_bad_weather(graph, 1e6) << endl;

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

    problem_with_hidden_state_t ph(graph);
    Policy::random_t<state_t> random(problem);
    Policy::greedy_t<state_t> greedy(problem, *heuristic);
    Policy::ao4_t<state_t> pol(greedy, ao_width, ao_depth, ao_parameter, false, ao_expansions_per_iteration); 

    float start_time = Utils::read_time_in_seconds();
    float value = 0;
    for( unsigned i = 0; i < evaluation_trials; ++i ) {
        vector<int> distances;
        state_t hidden = ph.sample_weather();
        hidden.compute_distances(graph, distances);
        while( distances[graph.num_nodes_ - 1] == numeric_limits<int>::max() ) {
            state_t hidden = ph.sample_weather();
            hidden.compute_distances(graph, distances);
        }
        cout << "dist=" << distances[graph.num_nodes_ - 1] << endl;
        hidden.set(graph.num_edges_ - 1, 0);
        ph.set_hidden(hidden);

        state_t state = ph.init();
        //cout << "init=" << state << endl;
        size_t steps = 0;
        float cost = 0;
        float discount = 1;
        while( (steps < evaluation_depth) && !ph.terminal(state) ) {
            Problem::action_t action = pol(state);
            assert(action != Problem::noop);
            assert(policy.problem().applicable(state, action));
            std::pair<state_t, bool> p = ph.sample(state, action);
            if( ph.cost(state, action) > 200 ) cout << "large cost" << endl;
            cost += discount * ph.cost(state, action);
            discount *= DISCOUNT;
            state = p.first;
            ++steps;
        }

        cout << "steps=" << steps << ", cost=" << cost << endl;
        value += cost;
    }
    value /= evaluation_trials;

    cout << setprecision(5);
    cout << value;
    cout << setprecision(2);
    cout << " ( " << Utils::read_time_in_seconds() - start_time << " secs)" << endl;

    // free resources
    for( unsigned i = 0; i < results.size(); ++i ) {
        delete results[i].hash_;
    }
    delete heuristic;

    exit(0);
}

