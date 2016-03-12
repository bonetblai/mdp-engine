#include <iostream>
#include <fstream>
#include <strings.h>
#include <vector>
#include <string>

#include <dispatcher.h>
#include "ctp3.h"

namespace Algorithm {
  unsigned g_seed = 0;
};

namespace Online {
  unsigned g_seed = 0;
};

namespace Utils {
  bool g_use_colors = true;
};

using namespace std;

void usage(ostream &os) {
    os << "usage: sailing [{-r | --request} <request>]* [{-s | --seed} <default-seed>] [{-t | --trials} <num-trials>] [{-c | --shortcut-cost} <cost>] [{-d | --dead-end-value} <value>] [{-f | --calculate-features} <nsamples>] <file>" << endl;
}

int main(int argc, const char **argv) {
    int cache_capacity = (int)5e5;
    int shortcut_cost = (int)5e3;
    float dead_end_value = 0;
    unsigned num_trials = 1;

    int calculate_features = 0;
    int calculate_nsamples = 0;

    vector<string> requests;

    float start_time = Utils::read_time_in_seconds();
    cout << fixed;

    // parse arguments
    for( ++argv, --argc; (argc > 1) && (**argv == '-'); ++argv, --argc ) {
        if( ((*argv)[1] == 'r') || (string(*argv) == "--request") ) {
            requests.push_back(argv[1]);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 's') || (string(*argv) == "--seed") ) {
            Algorithm::g_seed = strtoul(argv[1], 0, 0);
            Online::g_seed = Algorithm::g_seed;
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 't') || (string(*argv) == "--trials") ) {
            num_trials = strtoul(argv[1], 0, 0);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 'c') || (string(*argv) == "--shortcut-cost") ) {
            shortcut_cost = strtoul(argv[1], 0, 0);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 'd') || (string(*argv) == "--dead-end-value") ) {
            dead_end_value = strtod(argv[1], 0);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 'f') || (string(*argv) == "--calculate-features") ) {
            calculate_nsamples = strtoul(argv[1], 0, 0);
            calculate_features = true;
            ++argv;
            --argc;
        } else {
            usage(cout);
            exit(-1);
        }
    }

    // read problem parameters
    CTP::graph_t graph(false, shortcut_cost);
    if( argc >= 1 ) {
        ifstream is(argv[0], ifstream::in);
        if( !graph.parse(is) ) exit(-1);
        is.close();
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    cout << "main: seed=" << Algorithm::g_seed << endl;
    Random::set_seed(Algorithm::g_seed);
    state_t::initialize(graph, false, cache_capacity);
    problem_t problem(graph, dead_end_value, false, cache_capacity);

    if( calculate_features ) {
        float probability = probability_bad_weather(graph, calculate_nsamples);
        cout << "P(bad weather)=" << probability << endl;
        pair<float, float> p = branching_factor(problem, calculate_nsamples);
        cout << "avg-branching=" << p.first << endl;
        cout << "max-branching=" << p.second << endl;
    }

    // build requests
    vector<pair<string, Online::Policy::policy_t<state_t>*> > policies;
    vector<pair<string, Algorithm::algorithm_t<state_t>*> > algorithms;
    Dispatcher::dispatcher_t<state_t> dispatcher;
    dispatcher.insert_heuristic("ctp-min-min()", new ctp_min_min_t(problem));
    dispatcher.insert_policy("ctp-optimistic()", new ctp_optimistic_policy_t(problem, graph, 1.0));
    for( int i = 0; i < int(requests.size()); ++i ) {
        const string &request_str = requests[i];
        std::multimap<std::string, std::string> request;
        Utils::tokenize(request_str, request);
        for( std::multimap<std::string, std::string>::const_iterator it = request.begin(); it != request.end(); ++it ) {
            dispatcher.create_request(problem, it->first, it->second);
            if( it->first == "algorithm" ) {
                Algorithm::algorithm_t<state_t> *algorithm = dispatcher.fetch_algorithm(it->second);
                if( algorithm != 0 ) algorithms.push_back(make_pair(it->second, algorithm));
            } else if( it->first == "policy" ) {
                Online::Policy::policy_t<state_t> *policy = dispatcher.fetch_policy(it->second);
                if( policy != 0 ) policies.push_back(make_pair(it->second, policy));
            }
        }
    }

    // solve problems with requested algorithms
    vector<Dispatcher::dispatcher_t<state_t>::solve_result_t> solve_results;
    for( int i = 0; i < int(algorithms.size()); ++i ) {
        const string &request = algorithms[i].first;
        const Algorithm::algorithm_t<state_t> &algorithm = *algorithms[i].second;
        Dispatcher::dispatcher_t<state_t>::solve_result_t result;
        dispatcher.solve(request, algorithm, problem.init(), result);
        solve_results.push_back(result);
    }

    cout << Utils::warning() << "the following stats may aggregate figures when elements are shared among algorithms" << endl;
    if( !solve_results.empty() ) {
        for( int i = 0; i < int(solve_results.size()); ++i ) {
            dispatcher.print_stats(cout, solve_results[i]);
        }
    }

    // evaluate requested policies
    vector<Dispatcher::dispatcher_t<state_t>::evaluate_result_t> evaluate_results;
    for( int i = 0; i < int(policies.size()); ++i ) {
        const string &request = policies[i].first;
        const Online::Policy::policy_t<state_t> &policy = *policies[i].second;
        Dispatcher::dispatcher_t<state_t>::evaluate_result_t result;
        dispatcher.evaluate(request, policy, problem.init(), result, num_trials, 100, true);
        evaluate_results.push_back(result);
    }

    cout << Utils::warning() << "the following stats may aggregate figures when elements are shared among policies" << endl;
    if( !evaluate_results.empty() ) {
        for( int i = 0; i < int(evaluate_results.size()); ++i ) {
            dispatcher.print_stats(cout, evaluate_results[i]);
        }
    }

    cout << "main: total-time=" << Utils::read_time_in_seconds() - start_time << endl;
    return 0;
}

