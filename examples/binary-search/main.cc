#include <cassert>
#include <iostream>
#include <strings.h>
#include <vector>
#include <string>

#include <dispatcher.h>
#include "bs.h"

namespace Algorithm {
    unsigned g_seed = 0;
};

namespace Online {
    unsigned g_seed = 0;
};

namespace Utils {
    bool g_use_colors = true;
};

int beam_t::dim_ = 0;
unsigned belief_state_t::bitmap_mask_ = 0;
std::vector<unsigned> belief_state_t::action_masks_;

using namespace std;

void usage(ostream &os) {
    os << "usage: bs [--no-colors] [{-r | --request} <request>]* [{-s | --seed} <default-seed>] [{-t | --trials} <num-trials>] <dim>" << endl;
}

int main(int argc, const char **argv) {
    unsigned dim = 0;
    unsigned num_trials = 1;

    vector<string> requests;

    float start_time = Utils::read_time_in_seconds();
    cout << fixed;

    // parse arguments
    for( ++argv, --argc; (argc > 1) && (**argv == '-'); ++argv, --argc ) {
        if( string(*argv) == "--no-colors" ) {
            Utils::g_use_colors = false;
        } else if( ((*argv)[1] == 'r') || (string(*argv) == "--request") ) {
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

    // read pomdp parameters
    if( argc == 1 ) {
        dim = strtoul(argv[0], 0, 0);
    } else {
        usage(cout);
        exit(-1);
    }

    // build pomdp instance
    cout << "main: seed=" << Algorithm::g_seed << endl;
    Random::set_seed(Algorithm::g_seed);
    beam_t::set_dimension(dim);
    belief_state_t::set_bitmap_mask(dim);
    pomdp_t pomdp(dim);

    belief_state_t ibel = pomdp.init();
    cout << "initial bel = " << ibel << endl;
    vector<pair<belief_state_t, float> > outcomes;
    for( int a = 0; a <= dim; ++a ) {
        pomdp.next(ibel, a, outcomes);
        cout << "action a=" << a << ":" << endl;
        cout << "  p=" << outcomes[0].second << " --> " << outcomes[0].first << " {";
        for( beam_t::const_iterator it = outcomes[0].first.beam(0).begin(); it != outcomes[0].first.beam(0).end(); ++it )
            cout << it.value() << ",";
        cout << "}" << endl;
        cout << "  p=" << outcomes[1].second << " --> " << outcomes[1].first << " {";
        for( beam_t::const_iterator it = outcomes[1].first.beam(0).begin(); it != outcomes[1].first.beam(0).end(); ++it )
            cout << it.value() << ",";
        cout << "}" << endl;
    }

    // build requests
    vector<pair<string, Online::Policy::policy_t<belief_state_t>*> > policies;
    vector<pair<string, Algorithm::algorithm_t<belief_state_t>*> > algorithms;
    Dispatcher::dispatcher_t<belief_state_t> dispatcher;
    for( int i = 0; i < int(requests.size()); ++i ) {
        const string &request_str = requests[i];
        std::multimap<std::string, std::string> request;
        Utils::tokenize(request_str, request);
        for( std::multimap<std::string, std::string>::const_iterator it = request.begin(); it != request.end(); ++it ) {
            dispatcher.create_request(pomdp, it->first, it->second);
            if( it->first == "algorithm" ) {
                Algorithm::algorithm_t<belief_state_t> *algorithm = dispatcher.fetch_algorithm(it->second);
                if( algorithm != 0 ) algorithms.push_back(make_pair(it->second, algorithm));
            } else if( it->first == "policy" ) {
                Online::Policy::policy_t<belief_state_t> *policy = dispatcher.fetch_policy(it->second);
                if( policy != 0 ) policies.push_back(make_pair(it->second, policy));
            }
        }
    }

    // solve pomdp with requested algorithms
    vector<Dispatcher::dispatcher_t<belief_state_t>::solve_result_t> solve_results;
    for( int i = 0; i < int(algorithms.size()); ++i ) {
        const string &request = algorithms[i].first;
        const Algorithm::algorithm_t<belief_state_t> &algorithm = *algorithms[i].second;
        Dispatcher::dispatcher_t<belief_state_t>::solve_result_t result;
        dispatcher.solve(request, algorithm, pomdp.init(), result);
        solve_results.push_back(result);
    }

    cout << Utils::warning() << "the following stats may aggregate figures when elements are shared among algorithms" << endl;
    if( !solve_results.empty() ) {
        for( int i = 0; i < int(solve_results.size()); ++i ) {
            dispatcher.print_stats(cout, solve_results[i]);
        }
    }

    // evaluate requested policies
    vector<Dispatcher::dispatcher_t<belief_state_t>::evaluate_result_t> evaluate_results;
    for( int i = 0; i < int(policies.size()); ++i ) {
        const string &request = policies[i].first;
        const Online::Policy::policy_t<belief_state_t> &policy = *policies[i].second;
        Dispatcher::dispatcher_t<belief_state_t>::evaluate_result_t result;
        dispatcher.evaluate(request, policy, pomdp.init(), result, num_trials, 100, true);
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

