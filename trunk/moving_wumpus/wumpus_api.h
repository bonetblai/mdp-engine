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
    //const Heuristic::heuristic_t<state_t> *global_heuristic = 0;
    //const Heuristic::heuristic_t<m_state_t> *global_heuristic = 0;
    void *global_heuristic = 0;
  };
  namespace AOT2 {
    //const Heuristic::heuristic_t<state_t> *global_heuristic = 0;
    //const Heuristic::heuristic_t<m_state_t> *global_heuristic = 0;
    void *global_heuristic = 0;
  };
};

struct abstract_api_t {
    int nrows_;
    int ncols_;
    int npits_;
    int nwumpus_;
    int narrows_;
    bool compass_;
    int seed_;

    std::string policy_name_;
    Evaluation::parameters_t eval_pars_;

  public:
    abstract_api_t(int nrows, int ncols, int npits, int nwumpus, int narrows, bool compass)
      : nrows_(nrows), ncols_(ncols), npits_(npits), nwumpus_(nwumpus), narrows_(narrows),
        compass_(compass), seed_(0) {
        Random::seeds(seed_);
    }
    virtual ~abstract_api_t() { }

    void set_seed(int seed) {
        seed_ = seed;
        Random::seeds(seed_);
        std::cout << "seed=" << seed_ << std::endl;
    }

    void set_policy_parameters(int width, int depth, float par1, float par2) {
        eval_pars_.width_ = width;
        eval_pars_.depth_ = depth;
        eval_pars_.par1_ = par1;
        eval_pars_.par2_ = par2;
    }

    virtual void select_policy(const std::string &base_name, const std::string &policy_type) = 0;
    virtual void prepare_new_trial(int heading) = 0;
    virtual int select_action() const = 0;
    virtual void update(int obs) = 0;
    virtual void apply_action_and_update(int action, int obs) = 0;
    virtual void print(std::ostream &os) const = 0;
};

template<typename T> struct template_wumpus_api_t : public abstract_api_t {
    template_problem_t<T> *problem_;
    T *state_;

    std::vector<std::pair<const Policy::policy_t<T>*, std::string> > bases_;
    Heuristic::heuristic_t<T> *heuristic_;
    const Policy::policy_t<T> *policy_;

  public:
    template_wumpus_api_t(int nrows, int ncols, int npits, int nwumpus, int narrows, bool compass)
      : abstract_api_t(nrows, ncols, npits, nwumpus, narrows, compass),
        problem_(0), state_(0), heuristic_(0), policy_(0) {
        T::initialize(nrows_, ncols_);
        wumpus_belief_t::initialize(nrows_, ncols_, npits_, nwumpus_);
        m_wumpus_belief_t::initialize(nrows_, ncols_, npits_, nwumpus_);
        problem_ = new template_problem_t<T>(nrows_, ncols_, npits_, nwumpus_, narrows_, GOAL_IS_HAVE_GOLD, 1e5);
        heuristic_ = new shortest_distance_to_unvisited_cell_t<T>(*problem_, compass_);
        Policy::AOT::global_heuristic = heuristic_;
        Policy::AOT2::global_heuristic = heuristic_;

        // set base policies
        Policy::greedy_t<T> greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(greedy.clone(), "greedy"));
        Policy::random_greedy_t<T> random_greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(random_greedy.clone(), "random-greedy"));
        Policy::optimistic_greedy_t<T> optimistic_greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(optimistic_greedy.clone(), "optimistic-greedy"));
        Policy::random_t<T> random(*problem_);
        bases_.push_back(std::make_pair(random.clone(), "random"));
        wumpus_base_policy_t<T> wumpus(*problem_, nrows_, ncols_, compass_);
        bases_.push_back(std::make_pair(wumpus.clone(), "wumpus_base"));
    }
    virtual ~template_wumpus_api_t() {
        delete heuristic_;
        delete problem_;
    }

    virtual void select_policy(const std::string &base_name, const std::string &policy_type) {
        std::pair<const Policy::policy_t<T>*, std::string> p = Evaluation::select_policy(base_name, policy_type, bases_, eval_pars_);
        policy_ = p.first;
        policy_name_ = p.second;
        std::cout << "template_wumpus_api_t::policy=" << policy_name_ << std::endl;
    }

    virtual void prepare_new_trial(int heading) {
        if( state_ ) delete state_;
        state_ = new T(0, heading);
        //state_->set_narrows(narrows_);
        state_->set_as_unknown();
    }

    virtual int select_action() const {
        Problem::action_t action = (*policy_)(*state_);
        assert(action != Problem::noop);
        assert(state_->applicable(action));
        //std::cout << "pos=(" << (state_->pos() % ncols_) << ","
        //          << (state_->pos() / ncols_) << ")"
        //          << ", heading=" << heading_name(state_->heading())
        //          << std::endl;
        return action;
    }

    virtual void update(int obs) {
        state_->update(-1, obs);
        //std::cout << *state_;
    }
    virtual void apply_action_and_update(int action, int obs) {
        state_->apply_action_and_update(action, obs);
        //std::cout << *state_;
    }

    virtual void print(std::ostream &os) const {
        //os << *state_;
        //os << "heuristic=" << heuristic_->value(*state_) << std::endl;
    }
};

typedef template_wumpus_api_t<state_t> wumpus_api_t;
typedef template_wumpus_api_t<m_state_t> m_wumpus_api_t;

#if 0
struct wumpus_api_t {
    int nrows_;
    int ncols_;
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
    wumpus_api_t(int nrows, int ncols, int npits, int nwumpus, int narrows, bool compass)
      : nrows_(nrows), ncols_(ncols), npits_(npits),
        nwumpus_(nwumpus), narrows_(narrows), compass_(compass),
        problem_(0), state_(0), heuristic_(0), policy_(0) {
        state_t::initialize(nrows_, ncols_);
        wumpus_belief_t::initialize(nrows_, ncols_, npits_, nwumpus_);
        problem_ = new problem_t(nrows_, ncols_, npits_, nwumpus_, narrows_, GOAL_IS_HAVE_GOLD, 1e5);
        heuristic_ = new shortest_distance_to_unvisited_cell_t<state_t>(*problem_, compass_);
        Policy::AOT::global_heuristic = heuristic_;
        Policy::AOT2::global_heuristic = heuristic_;

        // set base policies
        Policy::greedy_t<state_t> greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(greedy.clone(), "greedy"));
        Policy::random_greedy_t<state_t> random_greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(random_greedy.clone(), "random-greedy"));
        Policy::optimistic_greedy_t<state_t> optimistic_greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(optimistic_greedy.clone(), "optimistic-greedy"));
        Policy::random_t<state_t> random(*problem_);
        bases_.push_back(std::make_pair(random.clone(), "random"));
        wumpus_base_policy_t<state_t> wumpus(*problem_, nrows_, ncols_, compass_);
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

    void prepare_new_trial(int heading = North) {
        if( state_ ) delete state_;
        state_ = new state_t(0, heading);
        //state_->set_narrows(narrows_);
        state_->set_as_unknown();
    }

    int select_action() const {
        Problem::action_t action = (*policy_)(*state_);
        assert(action != Problem::noop);
        assert(state_->applicable(action));
        //std::cout << "pos=(" << (state_->pos() % ncols_) << ","
        //          << (state_->pos() / ncols_) << ")"
        //          << ", heading=" << heading_name(state_->heading())
        //          << std::endl;
        return action;
    }

    void update(int obs) { state_->update(-1, obs); }
    void apply_action_and_update(int action, int obs) {
        state_->apply_action_and_update(action, obs);
    }

    void print(std::ostream &os) const {
        //os << *state_;
        //os << "heuristic=" << heuristic_->value(*state_) << std::endl;
    }
};
#endif

#if 0
struct m_wumpus_api_t {
    int nrows_;
    int ncols_;
    int npits_;
    int nwumpus_;
    int narrows_;
    bool compass_;

    m_problem_t *problem_;
    m_state_t *state_;

    std::vector<std::pair<const Policy::policy_t<m_state_t>*, std::string> > bases_;
    Heuristic::heuristic_t<m_state_t> *heuristic_;

    Evaluation::parameters_t eval_pars_;
    const Policy::policy_t<m_state_t> *policy_;
    std::string policy_name_;

  public:
    m_wumpus_api_t(int nrows, int ncols, int npits, int nwumpus, int narrows, bool compass)
      : nrows_(nrows), ncols_(ncols), npits_(npits),
        nwumpus_(nwumpus), narrows_(narrows), compass_(compass),
        problem_(0), state_(0), heuristic_(0), policy_(0) {
        m_state_t::initialize(nrows_, ncols_);
        m_wumpus_belief_t::initialize(nrows_, ncols_, npits_, nwumpus_);
        problem_ = new m_problem_t(nrows_, ncols_, npits_, nwumpus_, narrows_, GOAL_IS_HAVE_GOLD, 1e5);
        heuristic_ = new shortest_distance_to_unvisited_cell_t<m_state_t>(*problem_, compass_);
        Policy::AOT::global_heuristic = heuristic_;
        Policy::AOT2::global_heuristic = heuristic_;

        // set base policies
        Policy::greedy_t<m_state_t> greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(greedy.clone(), "greedy"));
        Policy::random_greedy_t<m_state_t> random_greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(random_greedy.clone(), "random-greedy"));
        Policy::optimistic_greedy_t<m_state_t> optimistic_greedy(*problem_, *heuristic_);
        bases_.push_back(std::make_pair(optimistic_greedy.clone(), "optimistic-greedy"));
        Policy::random_t<m_state_t> random(*problem_);
        bases_.push_back(std::make_pair(random.clone(), "random"));
        wumpus_base_policy_t<m_state_t> wumpus(*problem_, nrows_, ncols_, compass_);
        bases_.push_back(std::make_pair(wumpus.clone(), "wumpus_base"));
    }
    ~m_wumpus_api_t() {
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
        std::pair<const Policy::policy_t<m_state_t>*, std::string> p = Evaluation::select_policy(base_name, policy_type, bases_, eval_pars_);
        policy_ = p.first;
        policy_name_ = p.second;
        std::cout << "wumpus_api_t::policy=" << policy_name_ << std::endl;
    }

    void prepare_new_trial(int heading = North) {
        if( state_ ) delete state_;
        state_ = new m_state_t(0, heading);
        //state_->set_narrows(narrows_);
        state_->set_as_unknown();
    }

    int select_action() const {
        Problem::action_t action = (*policy_)(*state_);
        assert(action != Problem::noop);
        assert(state_->applicable(action));
        //std::cout << "pos=(" << (state_->pos() % ncols_) << ","
        //          << (state_->pos() / ncols_) << ")"
        //          << ", heading=" << heading_name(state_->heading())
        //          << std::endl;
        return action;
    }

    void update(int obs) { state_->update(-1, obs); }
    void apply_action_and_update(int action, int obs) {
        state_->apply_action_and_update(action, obs);
    }

    void print(std::ostream &os) const {
        os << *state_;
        //os << "heuristic=" << heuristic_->value(*state_) << std::endl;
    }
};
#endif


#endif

