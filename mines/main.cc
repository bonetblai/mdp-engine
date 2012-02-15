
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

class hidden_state_t {
    int rows_;
    int cols_;
    int nmines_;
    int first_cell_;
    std::vector<int> mines_;

  public:
    hidden_state_t(int rows, int cols, int nmines)
      : rows_(rows), cols_(cols), nmines_(nmines) { }
    ~hidden_state_t() { }

    void sample() {
        set<int> forbidden;
        first_cell_ = lrand48() % (rows_ * cols_);
        forbidden.insert(first_cell_);
        place_random_objects(mines_, rows_, cols_, nmines_, forbidden);
    }

    int get_obs(int action) const {
        bool flag = action < rows_ * cols_;
        int cell = flag ? action : action - rows_ * cols_;
        int obs = num_surrounding_mines(cell);
        return flag ? -1 : obs;
    }

    int get_first_action() const {
        return (rows_ * cols_) + first_cell_;
    }
    int get_first_obs() const {
        return get_obs(get_first_action());
    }

    int num_surrounding_mines(int cell) const {
        if( mines_[cell] == 1 ) return 9;

        int row = cell / cols_;
        int col = cell % cols_;

        int num = 0;
        for( int drow = -1; drow < 2; ++drow ) {
            int nrow = row + drow;
            if( (nrow < 0) || (nrow >= rows_) ) continue;
            for( int dcol = -1; dcol < 2; ++dcol ) {
                int ncol = col + dcol;
                if( (ncol < 0) || (ncol >= cols_) ) continue;
                num += mines_[nrow * cols_ + ncol];
            }
        }
        assert((0 <= num) && (num < 9));
        return num;
    }
};

void usage(ostream &os) {
    os << "mines [-s <seed>] [-t <ntrials>] <nrows> <ncols> <nmines>"
       << endl;
}

int main(int argc, const char **argv) {
    int rows = 0;
    int cols = 0;
    int nmines = 0;
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

    if( argc >= 3 ) {
        rows = strtoul(*argv++, 0, 0);
        cols = strtoul(*argv++, 0, 0);
        nmines = strtoul(*argv++, 0, 0);
        argc -= 3;
    } else {
        usage(cout);
        exit(-1);
    }

    // initialize and construct base policy
    mines_belief_t::initialize(rows, cols);
    base_policy_t policy;
    hidden_state_t hidden_state(rows, cols, nmines);

    int nsuccess = 0;

    cout << "trials: ";
    for( int trial = 0; trial < ntrials; ++trial ) {
        cout << "." << flush;

        // sample hidden state
        hidden_state.sample();

        // set initial belief state
        state_t state(rows, cols, nmines);
        state.set_as_unknown();

        // make first move
        state.apply_action_and_update(hidden_state.get_first_action(),
                                      hidden_state.get_first_obs());

        // main loop
        while( state.nflags() != nmines ) {
            assert(!state.inconsistent());
            int action = policy(state);
            int obs = hidden_state.get_obs(action);
            if( obs == 9 ) {
                //cout << "Died when opening cell=("
                //     << (cell % cols) << "," << (cell / cols) << ")"
                //     << endl;
                break;
            } else {
                state.apply_action_and_update(action, obs);
            }
        }
        nsuccess += state.nflags() == nmines ? 1 : 0;

    }
    cout << endl;

    float avg = (float)nsuccess / (float)ntrials;
    cout << "succ. avg. = " << avg << endl;

    exit(0);
}

