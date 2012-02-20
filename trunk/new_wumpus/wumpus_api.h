#ifndef WUMPUS_API_
#define WUMPUS_API_

#include <iostream>
#include <vector>

#include "wumpus.h"
#include "base_policy.h"

#define EXPERIMENT

#include "../evaluation.h"
#include "dispatcher.h"

namespace Policy {
  namespace AOT {
    const Heuristic::heuristic_t<state_t> *global_heuristic = 0;
  };
  namespace AOT2 {
    const Heuristic::heuristic_t<state_t> *global_heuristic = 0;
  };
};

struct wumpus_api_t {
    int rows_;
    int cols_;
    int npits_;
    int nwumpus_;
    int narrows_;
    bool compass_;

    problem_t *problem_;
    state_t *state_;

    std::vector<std::pair<const Policy::policy_t<state_t>*, std::string> > bases_;
    Heuristic::heuristic_t<state_t> *heuristic_;

    Evaluation::parameters_t eval_pars_;
    const Policy::policy_t<state_t> *policy_;
    std::string policy_name_;

  public:
    wumpus_api_t(int rows, int cols, int npits, int nwumpus, int narrows, bool compass)
      : rows_(rows), cols_(cols), npits_(npits),
        nwumpus_(nwumpus), narrows_(narrows), compass_(compass),
        problem_(0), state_(0), heuristic_(0), policy_(0) {
        state_t::initialize(rows_, cols_, compass_);
        wumpus_belief_t::initialize(rows_, cols_);
        problem_ = new problem_t(rows_, cols_, npits_, nwumpus_, narrows_, 1e5);
        heuristic_ = new shortest_distance_to_unvisited_cell_t(*problem_, compass_);
        Policy::AOT::global_heuristic = heuristic_;
        Policy::AOT2::global_heuristic = heuristic_;

        // set base policies
        Policy::greedy_t<state_t> greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(greedy.clone(), "greedy"));
        Policy::random_t<state_t> random(*problem_);
        bases_.push_back(std::make_pair(random.clone(), "random"));
        wumpus_base_policy_t wumpus(*problem_, rows_, cols_, compass_);
        bases_.push_back(std::make_pair(wumpus.clone(), "wumpus_base"));
    }
    ~wumpus_api_t() {
        delete heuristic_;
        delete problem_;
    }

    void set_seed(int seed) {
        std::cout << "seed=" << seed << std::endl;
        Random::seeds(seed);
    }

    void set_policy_parameters(int width, int depth, float par1, float par2) {
        eval_pars_.width_ = width;
        eval_pars_.depth_ = depth;
        eval_pars_.par1_ = par1;
        eval_pars_.par2_ = par2;
    }
    void select_policy(const std::string &base_name, const std::string &policy_type) {
        std::pair<const Policy::policy_t<state_t>*, std::string> p = Evaluation::select_policy(base_name, policy_type, bases_, eval_pars_);
        policy_ = p.first;
        policy_name_ = p.second;
        std::cout << "wumpus_api_t::policy=" << policy_name_ << std::endl;
    }

    void prepare_new_trial() {
        if( state_ ) delete state_;
        state_ = new state_t(0, East);
        state_->set_narrows(narrows_);
        state_->set_as_unknown();
    }

    int select_action() const {
        Problem::action_t action = (*policy_)(*state_);
        assert(action != Problem::noop);
        assert(state_->applicable(action));
        //std::cout << "pos=(" << (state_->pos() % cols_) << ","
        //          << (state_->pos() / cols_) << ")"
        //          << ", heading=" << heading_name(state_->heading())
        //          << std::endl;
        return action;
    }

    void update(int obs) {
        state_->update(obs);
    }
    void apply_action_and_update(int action, int obs) {
        state_->apply_action_and_update(action, obs);
    }

};

#endif

