
extern "C" {
#include "c_api.h"
};

#include "agent.h"
#include <iostream>

using namespace std;

state_t *agent_state = 0;
base_policy_t *agent_policy = 0;

void agent_initialize(int rows, int cols, int nmines) {
    cout << "agent: initialization: rows=" << rows
         << ", cols=" << cols
         << ", nmines=" << nmines
         << endl;

    mines_belief_t::initialize(rows, cols);

    agent_state = new state_t(rows, cols, nmines);
    agent_state->set_as_unknown();
    agent_policy = new base_policy_t;
}

int agent_get_action() {
    int action = (*agent_policy)(*agent_state);
    return action;
}

int agent_is_flag_action(int action) {
    return action < agent_state->ncells() ? 1 : 0;
}

int agent_get_cell(int action) {
    bool flag = action < agent_state->ncells();
    return flag ? action : action - agent_state->ncells();
}

void agent_update_state(int flag, int cell, int obs) {
    agent_state->apply(flag == 1, cell);
    agent_state->update(flag == 1, cell, obs);
}

