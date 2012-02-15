
#include "agent.h"
#include <iostream>
#include <set>

using namespace std;

void place_random_objects(vector<int> &state, int rows, int cols, int nobjs, set<int> &forbidden) {
    state = vector<int>(rows * cols, 0);
    for( int i = 0; i < nobjs; ++i ) {
        int cell = lrand48() % (rows * cols);
        while( (state[cell] == 1) || (forbidden.find(cell) != forbidden.end()) )
            cell = lrand48() % (rows * cols);
        state[cell] = 1;
    }
}

class hidden_state_t : public state_t {
    std::vector<int> pits_;
    std::vector<int> wumpus_;

  public:
    hidden_state_t(int rows, int cols, int npits, int nwumpus, int narrows)
      : state_t(rows, cols, npits, nwumpus, narrows) { }
    ~hidden_state_t() { }

    void sample() {
        alive_ = true;
        pos_ = 0;
        heading_ = NORTH;
        gold_ = lrand48() % (rows_ * cols_);

        set<int> forbidden;
        forbidden.insert(pos_);
        forbidden.insert(gold_);
        place_random_objects(pits_, rows_, cols_, npits_, forbidden);
        place_random_objects(wumpus_, rows_, cols_, nwumpus_, forbidden);
    }

    int get_obs() {
        int breeze = num_surrounding_objs(pits_);
        int stench = num_surrounding_objs(wumpus_);
        int glitter = gold_ == pos_ ? 1 : 0;
        int obs = 0;
        obs += glitter > 0 ? GLITTER : 0;
        obs += breeze > 0 ? BREEZE : 0;
        obs += stench > 0 ? STENCH : 0;
        alive_ = (breeze != 9) && (stench != 9);
        return obs;
    }

    int apply_action_and_get_obs(int action) {
        apply(action);
        return get_obs();
    }

    int num_surrounding_objs(std::vector<int> &objs) const {
        if( objs[pos_] == 1 ) return 9;

        int row = pos_ / cols_;
        int col = pos_ % cols_;

        int num = 0;
        for( int drow = -1; drow < 2; ++drow ) {
            int nrow = row + drow;
            if( (nrow < 0) || (nrow >= rows_) ) continue;
            for( int dcol = -1; dcol < 2; ++dcol ) {
                if( (drow != 0) && (dcol != 0) ) continue;
                int ncol = col + dcol;
                if( (ncol < 0) || (ncol >= cols_) ) continue;
                num += objs[nrow * cols_ + ncol];
            }
        }
        assert((0 <= num) && (num < 9));
        return num;
    }
};

void usage(ostream &os) {
    os << "wumpus [-s <seed>] [-t <ntrials>] <nrows> <ncols> <npits> <nwumpus> <narrows>"
       << endl;
}

int main(int argc, const char **argv) {
    int rows = 0;
    int cols = 0;
    int npits = 0;
    int nwumpus = 0;
    int narrows = 0;

    int seed = 0;
    int ntrials = 1;

    cout << fixed;

    // parse arguments
    ++argv;
    --argc;
    while( argc > 1 ) {
        if( **argv != '-' ) break;
        switch( (*argv)[1] ) {
            case 's':
                seed = strtoul(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            case 't':
                ntrials = strtoul(argv[1], 0, 0);
                argv += 2;
                argc -= 2;
                break;
            default:
                usage(cout);
                exit(-1);
        }
    }

    cout << "seed=" << seed << endl;
    unsigned short useed[3];
    useed[0] = useed[1] = useed[2] = seed;
    srand48((long int)seed);
    seed48(useed);

    if( argc >= 5 ) {
        rows = strtoul(*argv++, 0, 0);
        cols = strtoul(*argv++, 0, 0);
        npits = strtoul(*argv++, 0, 0);
        nwumpus = strtoul(*argv++, 0, 0);
        narrows = strtoul(*argv++, 0, 0);
        argc -= 5;
    } else {
        usage(cout);
        exit(-1);
    }

    // initialize and construct base policy
    wumpus_belief_t::initialize(rows, cols);
    base_policy_t policy(rows, cols);
    hidden_state_t hidden_state(rows, cols, npits, nwumpus, narrows);

    int nalive = 0, ngold = 0, nsteps = 0;

    cout << "trials: ";
    for( int trial = 0; trial < ntrials; ++trial ) {
        cout << "." << flush;

        // sample hidden state
        hidden_state.sample();

        // set initial belief state
        state_t state(rows, cols, npits, nwumpus, narrows);
        state.set_as_unknown();

        // filter with initial obs
        state.update(hidden_state.get_obs());

        // main loop
        assert(hidden_state.in_cave());
        //set<int> visited;
        //visited.insert(0);
        while( hidden_state.in_cave() ) {
            ++nsteps;
            assert(hidden_state.pos() == state.pos());
            //if( state.inconsistent() ) { cout << "inconsistent: "; state.print(cout); }
            assert(!state.inconsistent());
            int action = policy(state);
            assert(state.applicable(action));
            assert(hidden_state.applicable(action));
            int obs = hidden_state.apply_action_and_get_obs(action);
            if( hidden_state.dead() ) {
                cout << "dead!" << endl;
                //cout << "state:  "; state.print(cout);
                //cout << "hidden: "; hidden_state.print(cout);
                break;
            } else {
                state.apply_action_and_update(action, obs);
            }
#if 0
            if( visited.find(state.pos()) == visited.end() ) {
                cout << "action=" << action << ", obs=" << obs << endl;
                cout << "state:  "; state.print(cout);
                //cout << "hidden: "; hidden_state.print(cout);
                if( visited.size() == 12 ) exit(0);
                visited.insert(state.pos());
            }
#endif
        }

        if( hidden_state.alive() ) {
            ++nalive;
            ngold += hidden_state.have_gold() ? 1 : 0;
        }

    }
    cout << endl;

    float alive_avg = (float)nalive / (float)ntrials;
    float gold_avg = (float)ngold / (float)ntrials;
    float steps_avg = (float)nsteps / (float)ntrials;
    cout << "alive avg. = " << alive_avg << endl
         << "gold avg. = " << gold_avg << endl
         << "steps avg. = " << steps_avg << endl;

    exit(0);
}

