#ifndef AGENT_H
#define AGENT_H

#include "wumpus_belief.h"
#include "defs.h"

#include <cassert>
#include <iostream>

template<typename T> class template_state_t {
  protected:
    T *belief_;

    static int nrows_;
    static int ncols_;
    static int ncells_;

  public:
    template_state_t(int pos = 0, int heading = North, int narrows = 0) {
        belief_ = T::allocate();
        belief_->set_position(pos);
        belief_->set_heading(heading);
    }
    template_state_t(const template_state_t &state) {
        belief_ = T::allocate();
        *belief_ = *state.belief_;
    }
    template_state_t(template_state_t &&state) : belief_(state.belief_) {
        state.belief_ = 0;
    }
    ~template_state_t() {
        T::deallocate(belief_);
    }

    static void initialize(int nrows, int ncols) {
        nrows_ = nrows;
        ncols_ = ncols;
        ncells_ = nrows_ * ncols_;
        std::cout << "template_state_t::initialize:"
                  << " nrows=" << nrows_
                  << ", ncols=" << ncols_
                  << ", ncells=" << ncells_
                  << std::endl;
    }

    size_t hash() const { return belief_->hash(); }

    int position() const { return belief_->position(); }
    int heading() const { return belief_->heading(); }
    bool have_gold() const { return belief_->have_gold(); }
    bool dead() const { return belief_->dead(); }
    bool inconsistent() const { return !belief_->consistent(); }
    bool in_gold_cell() const { return belief_->in_gold_cell(); }
    bool possible_gold_at(int cell) const { return belief_->possible_gold_at(cell); }
    bool in_cave() const { return belief_->position() != OutsideCave; }

    void set_as_unknown() { belief_->set_as_unknown(); }
    bool applicable(int action) const { return belief_->applicable(action); }
    void apply(int action) { belief_->progress(action); }
    void update(int action, int obs) { belief_->filter(action, obs); }
    void apply_action_and_update(int action, int obs) { belief_->progress_and_filter(action, obs); }
    bool possible_obs(int action, int obs) { return belief_->possible_obs(action, obs); }

    bool hazard_at(int cell) const { return belief_->hazard_at(cell); }
    bool no_hazard_at(int cell) const { return belief_->no_hazard_at(cell); }
    int target_cell(int action) const { return belief_->target_cell(action); }

    void print(std::ostream &os) const {
        os << *belief_;
    }

    const template_state_t& operator=(const template_state_t &s) {
        *belief_ = *s.belief_;
        return *this;
    }
    bool operator==(const template_state_t &s) const {
        return *belief_ == *s.belief_;
    }
    bool operator<(const template_state_t &s) const {
        assert(0); return false;
    }
};

template<typename T> int template_state_t<T>::nrows_ = 0;
template<typename T> int template_state_t<T>::ncols_ = 0;
template<typename T> int template_state_t<T>::ncells_ = 0;

template<typename T> inline std::ostream& operator<<(std::ostream &os, const template_state_t<T> &state) {
    state.print(os);
    return os;
}


// template instantiation
typedef template_state_t<wumpus_belief_t> state_t;
typedef template_state_t<m_wumpus_belief_t> m_state_t;

#endif

