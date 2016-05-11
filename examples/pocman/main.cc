#include <cassert>
#include <iostream>
#include <strings.h>
#include <vector>
#include <string>

#include <dispatcher.h>
#include "pocman.h"

namespace Algorithm {
  unsigned g_seed = 0;
};

namespace Online {
  unsigned g_seed = 0;
};

namespace Utils {
  bool g_use_colors = true;
};

#if 0
int Bitmap::bitmap_t::dim_ = 0;
int Bitmap::bitmap_t::dim_in_words_ = 0;
int Bitmap::bitmap_t::bits_in_last_word_ = 0;
unsigned Bitmap::bitmap_t::last_word_mask_ = 0;

const pomdp_t *arc_consistency_t::pomdp_ = 0;
const pomdp_t *belief_state_t::pomdp_ = 0;
#endif

using namespace std;

void usage(ostream &os) {
    os << "usage: pocman [--no-colors] [{-r | --request} <request>]* [{-s | --seed} <default-seed>] [{-t | --trials} <num-trials>] <PARAMETERS>" << endl;
}

int main(int argc, const char **argv) {
#if 0
    int xdim = 0;
    int ydim = 0;
    int number_rocks = 0;
    int max_antenna_height = 0;
#endif

    unsigned num_trials = 1;
    unsigned max_steps = 100;
    vector<string> requests;

    float start_time = Utils::read_time_in_seconds();
    cout << fixed;

    // parse arguments
    for( ++argv, --argc; (argc > 1) && (**argv == '-'); ++argv, --argc ) {
        if( string(*argv) == "--max-steps" ) {
            max_steps = strtol(argv[1], 0, 0);
            ++argv;
            --argc;
        } else if( string(*argv) == "--no-colors" ) {
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
        } else if( string(*argv, 3) == "--X" ) {
            // we use --X<something else> to ignode <something else> without raising usage() error
            // do nothing
        } else {
            usage(cout);
            exit(-1);
        }
    }

    //requests.push_back("policy=iw-bel2(determinization=most-likely,dp=5,stop-criterion=reward,max-expansions=30,random-ties=true,prune-threshold=2)");

    // read parameters
    if( argc == 0 ) {
#if 0
        xdim = strtoul(argv[0], 0, 0);
        ydim = strtoul(argv[1], 0, 0);
        number_rocks = strtoul(argv[2], 0, 0);
        max_antenna_height = strtoul(argv[3], 0, 0);
#endif
    } else {
        usage(cout);
        exit(-1);
    }

    // build instance
    cout << "main: seed=" << Algorithm::g_seed << endl;
    Random::set_seed(Algorithm::g_seed);

    state_t state;
    cout << state << endl;

#if 0
    Bitmap::bitmap_t::set_dimension(number_rocks);
    pomdp_t pomdp(xdim, ydim, number_rocks, max_antenna_height, true);
    arc_consistency_t::set_static_members(&pomdp);
    belief_state_t::set_static_members(&pomdp);
#endif

#if 0
    // build requests
    vector<pair<string, Online::Policy::policy_t<belief_state_t>*> > policies;
    vector<pair<string, Algorithm::algorithm_t<belief_state_t>*> > algorithms;
    Dispatcher::dispatcher_t<belief_state_t> dispatcher;
    for( int i = 0; i < int(requests.size()); ++i ) {
        const string &request_str = requests[i];
        multimap<string, string> request;
        if( !Utils::tokenize(request_str, request) ) continue;
        for( multimap<string, string>::const_iterator it = request.begin(); it != request.end(); ++it ) {
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
        dispatcher.solve(request, algorithm, result);
        solve_results.push_back(result);
    }

    if( !solve_results.empty() ) {
        cout << Utils::warning() << "the following stats may aggregate figures when elements are shared among algorithms" << endl;
        for( int i = 0; i < int(solve_results.size()); ++i )
            dispatcher.print_stats(cout, solve_results[i]);
    }

    // evaluate requested policies
    vector<Dispatcher::dispatcher_t<belief_state_t>::evaluate_result_t> evaluate_results;
    for( int i = 0; i < int(policies.size()); ++i ) {
        const string &request = policies[i].first;
        const Online::Policy::policy_t<belief_state_t> &policy = *policies[i].second;
        Dispatcher::dispatcher_t<belief_state_t>::evaluate_result_t result;
        dispatcher.evaluate(request, policy, result, num_trials, max_steps, true);
        evaluate_results.push_back(result);
    }

    if( !evaluate_results.empty() ) {
        cout << Utils::warning() << "the following stats may aggregate figures when elements are shared among policies" << endl;
        for( int i = 0; i < int(evaluate_results.size()); ++i )
            dispatcher.print_stats(cout, evaluate_results[i]);
    }
#endif

    cout << "main: total-time=" << Utils::read_time_in_seconds() - start_time << endl;
    return 0;
}

