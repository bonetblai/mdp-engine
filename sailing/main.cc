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
    os << "usage: sailing [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-s <n>] <dim>"
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
       << "  <dim>     Dimension for rows ans cols <= 2^16."
       << endl << endl;

    os << "usage: sailing [-r <request>]* [-f | --formatted] [{-s | --seed} <seed>] [{-t | --trials} <ntrials>]  <x-dim> <y-dim>" << endl;
}

/*
NEED to pass: heuristic, base policie, constructed policy
e.g.    pac(tree=tree,heuristic=min-min,base=greedy(heuristic=min-min),other-parameters)
e.g.    aot(...)
e.g.    value-iteration()
e.g.    slrtdp()
*/


int main(int argc, const char **argv) {
    unsigned xdim = 0;
    unsigned ydim = 0;
    bool formatted = false;
    unsigned ntrials = 1;

    vector<string> requests;

    //string base_name;
    //string policy_type;
    //Online::Evaluation::parameters_t eval_pars;

    cout << fixed;
    //Algorithm::parameters_t alg_pars;

    // parse arguments
    for( ++argv, --argc; (argc > 1) && (**argv == '-'); ++argv, --argc ) {
        if( (*argv)[1] == 'r' ) {
            requests.push_back(argv[1]);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 'f') || (string(*argv) == "--formatted") ) {
            formatted = true;
        } else if( ((*argv)[1] == 's') || (string(*argv) == "--seed") ) {
            Algorithm::g_seed = strtoul(argv[1], 0, 0);
            Online::g_seed = Algorithm::g_seed;
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 't') || (string(*argv) == "--trials") ) {
            ntrials = strtoul(argv[1], 0, 0);
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
    cout << "seed=" << Algorithm::g_seed << endl;
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
        //if( formatted ) dispatcher.print_result(cout, 0);
        for( int i = 0; i < int(solve_results.size()); ++i )
            dispatcher.print(cout, solve_results[i]);
    }

    // evaluate requested policies
    vector<Dispatcher::dispatcher_t<state_t>::evaluate_result_t> evaluate_results;
    for( int i = 0; i < int(policies.size()); ++i ) {
        const string &request = policies[i].first;
        Online::Policy::policy_t<state_t> *policy = policies[i].second;
        Dispatcher::dispatcher_t<state_t>::evaluate_result_t result;
        dispatcher.evaluate(request, *policy, problem.init(), result);
        evaluate_results.push_back(result);
    }

#if 0
    vector<pair<const Heuristic::heuristic_t<state_t>*, string> > heuristics;
    heuristics.push_back(make_pair(new zero_heuristic_t, "zero"));
    heuristics.push_back(make_pair(new Heuristic::min_min_heuristic_t<state_t>(problem), "min-min"));
    heuristics.push_back(make_pair(heuristics.back().first, "random"));
    heuristics.push_back(make_pair(heuristics.back().first, "greedy"));
    heuristics.push_back(make_pair(heuristics.back().first, "optimal"));
    heuristics.push_back(make_pair(new scaled_heuristic_t(new Heuristic::min_min_heuristic_t<state_t>(problem), 0.5), "min-min-scaled"));

    Heuristic::heuristic_t<state_t> *heuristic = 0;
    if( h == 0 ) {
        heuristic = new zero_heuristic_t;
    } else if( h == 1 ) {
        heuristic = new Heuristic::min_min_heuristic_t<state_t>(problem);
    } else if( h == 2 ) {
        Heuristic::heuristic_t<state_t> *base = new Heuristic::min_min_heuristic_t<state_t>(problem);
        heuristic = new scaled_heuristic_t(base, 0.5);
    }

    // solve problem with algorithms
    vector<Dispatcher::result_t<state_t> > results;
    Dispatcher::solve(problem, heuristic, problem.init(), bitmap, alg_pars, results);

    // print results
    if( !results.empty() ) {
        if( formatted ) Dispatcher::print_result<state_t>(cout, 0);
        for( unsigned i = 0; i < results.size(); ++i ) {
            Dispatcher::print_result(cout, &results[i]);
        }
    }


    // evaluate policies
    vector<pair<const Online::Policy::policy_t<state_t>*, string> > bases;

    // fill base policies
    const Problem::hash_t<state_t> *hash = results.empty() ? 0 : results[0].hash_;
    if( hash != 0 ) {
        Online::Policy::hash_policy_t<state_t> optimal(*hash);
        bases.push_back(make_pair(optimal.clone(), "optimal"));
    }
    if( heuristic != 0 ) {
        Online::Policy::greedy_t<state_t> greedy(problem, *heuristic, false);
        bases.push_back(make_pair(greedy.clone(), "greedy"));
cout << "EVALUATE (in main.cc): greedy=" << Online::Evaluation::evaluate_policy(greedy, eval_pars).first.first << endl;
    }
    if( heuristic != 0 ) {
        Online::Policy::greedy_t<state_t> greedy(problem, *heuristic, false);
        bases.push_back(make_pair(greedy.clone(), "greedy-scaled"));
    }
    Online::Policy::random_t<state_t> random(problem);
    bases.push_back(make_pair(&random, "random"));
cout << "EVALUATE (in main.cc): random=" << Online::Evaluation::evaluate_policy(random, eval_pars).first.first << endl;

    // evaluate
cout << "BASE: " << base_name << " " << policy_type << endl;
    pair<const Online::Policy::policy_t<state_t>*, std::string> policy =
      Online::Evaluation::select_policy(problem, base_name, policy_type, bases, heuristics, eval_pars);
    if( policy.first != 0 ) {
        pair<pair<float, float>, float> eval = Online::Evaluation::evaluate_policy(*policy.first, eval_pars, true);
        cout << policy.second
             << "= " << setprecision(5) << eval.first.first
             << " " << eval.first.second
             << setprecision(2) << " ( " << eval.second << " secs " << policy.first->decisions() << " decisions)" << endl;
        policy.first->print_stats(cout);
    } else {
        cout << "error: " << policy.second << endl;
    }

    // free resources
    delete policy.first;
    for( unsigned i = 0; i < results.size(); ++i ) {
        delete results[i].hash_;
    }
    delete heuristic;
#endif

    exit(0);
}

