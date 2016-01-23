#include <iostream>
#include <strings.h>
#include <vector>

#include <dispatcher.h>
#include "sailing.h"

namespace Algorithm {
  unsigned g_seed = 0;
};

namespace Online {
  unsigned g_seed = 0;
};

using namespace std;

void usage(ostream &os) {
    os << "usage: sailing [{-r | --request} <request>]* [{-s | --seed} <default-seed>] [{-t | --trials} <num-trials>]  <x-dim> <y-dim>" << endl;
}

int main(int argc, const char **argv) {
    unsigned xdim = 0;
    unsigned ydim = 0;
    unsigned num_trials = 1;

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
        } else {
            usage(cout);
            exit(-1);
        }
    }

    // read dimensions
    if( argc == 2 ) {
        xdim = strtoul(argv[0], 0, 0);
        ydim = strtoul(argv[1], 0, 0);
    } else {
        usage(cout);
        exit(-1);
    }

    // build problem instances
    cout << "main: seed= " << Algorithm::g_seed << endl;
    Random::set_seed(Algorithm::g_seed);
    problem_t problem(xdim, ydim);

    // build requests
    vector<pair<string, Online::Policy::policy_t<state_t>*> > policies;
    vector<pair<string, Algorithm::algorithm_t<state_t>*> > algorithms;
    Dispatcher::dispatcher_t<state_t> dispatcher;
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
        Algorithm::algorithm_t<state_t> *algorithm = algorithms[i].second;
        Dispatcher::dispatcher_t<state_t>::solve_result_t result;
        dispatcher.solve(request, *algorithm, problem.init(), result);
        solve_results.push_back(result);
    }

    if( !solve_results.empty() ) {
        for( int i = 0; i < int(solve_results.size()); ++i )
            dispatcher.print(cout, solve_results[i]);
    }

    // evaluate requested policies
    vector<Dispatcher::dispatcher_t<state_t>::evaluate_result_t> evaluate_results;
    for( int i = 0; i < int(policies.size()); ++i ) {
        const string &request = policies[i].first;
        Online::Policy::policy_t<state_t> *policy = policies[i].second;
        Dispatcher::dispatcher_t<state_t>::evaluate_result_t result;
        dispatcher.evaluate(request, *policy, problem.init(), result, num_trials, 100, true);
        evaluate_results.push_back(result);
    }

    if( !evaluate_results.empty() ) {
        for( int i = 0; i < int(evaluate_results.size()); ++i )
            dispatcher.print(cout, evaluate_results[i]);
    }


    cout << "main: total-time= " << Utils::read_time_in_seconds() - start_time << endl;
    return 0;
}

