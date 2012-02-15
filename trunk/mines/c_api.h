void agent_initialize(int rows, int cols, int nmines);
int agent_get_action();
int agent_is_flag_action(int action);
int agent_get_cell(int action);
void agent_update_state(int flag, int cell, int obs);

