#include <iostream>
#include <iomanip>
#include <strings.h>

#include "../binary-search/bitmap.h"

#define DISCOUNT            1
#define OBS_NOT_GOOD        0
#define OBS_GOOD            1

//#define DEBUG_CTOR_DTOR
//#define DEBUG

struct loc_t {
    int r_, c_;

    loc_t(int r, int c) : r_(r), c_(c) { }
    loc_t(int loc, int xdim, int ydim)
      : r_(loc / xdim), c_(loc % xdim) {
    }

    void move_north(int xdim, int ydim) {
        if( r_ + 1 < ydim ) ++r_;
    }
    void move_east(int xdim, int ydim) {
        if( c_ + 1 < xdim ) ++c_;
    }
    void move_south(int xdim, int ydim) {
        if( r_ > 0 ) --r_;
    }
    void move_west(int xdim, int ydim) {
        if( c_ > 0 ) --c_;
    }

    float euclidean_distance(const loc_t loc) const {
        int d = abs(r_ - loc.r_) * abs(r_ - loc.r_) + abs(c_ - loc.c_) * abs(c_ - loc.c_);
        return sqrtf(d);
    }

    bool operator==(const loc_t &loc) const {
        return (r_ == loc.r_) && (c_ == loc.c_);
    }
    bool operator<(const loc_t &loc) const {
        return (r_ < loc.r_) || ((r_ == loc.r_) && (c_ < loc.c_));
    }

    int as_integer(int xdim, int ydim) const {
        return r_ * xdim + c_;
    }

    void print(std::ostream &os) const {
        os << "(" << c_ << "," << r_ << ")" << std::flush;
    }
};

inline std::ostream& operator<<(std::ostream &os, const loc_t &loc) {
    loc.print(os);
    return os;
}

class pomdp_t; // forward reference

// known vars: loc, antenna-height, sampled rocks, skipped rocks
// unknown vars: rock status
class belief_state_t {
  protected:
    loc_t loc_;
    float *beams_;
    Bitmap::bitmap_t sampled_;
    Bitmap::bitmap_t hidden_;

    static const pomdp_t *pomdp_;

  public:
    belief_state_t(int loc = 0)
      : loc_(loc, xdim(), ydim()),
        sampled_(0),
        hidden_(0) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: ctor called" << std::endl;
#endif
        beams_ = new float[number_rocks()];
        set_initial_beams();
    }
    belief_state_t(int loc, const Bitmap::bitmap_t &hidden)
      : loc_(loc, xdim(), ydim()),
        sampled_(0),
        hidden_(hidden) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: ctor w/ hidden called" << std::endl;
#endif
        beams_ = new float[number_rocks()];
        set_initial_beams();
    }
    belief_state_t(const belief_state_t &bel)
      : loc_(bel.loc_),
        sampled_(bel.sampled_),
        hidden_(bel.hidden_) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: copy ctor called" << std::endl;
#endif
        beams_ = new float[number_rocks()];
        copy_beams(bel);
    }
    belief_state_t(belief_state_t &&bel)
      : loc_(bel.loc_),
        beams_(bel.beams_),
        sampled_(std::move(bel.sampled_)),
        hidden_(std::move(bel.hidden_)) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: move ctor called" << std::endl;
#endif
        bel.beams_ = 0;
    }
    virtual ~belief_state_t() {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: dtor called" << std::endl;
#endif
        delete[] beams_;
    }

    static void set_static_members(const pomdp_t *pomdp) {
        pomdp_ = pomdp;
        std::cout << "pomdp: xdim=" << xdim() << ", ydim=" << ydim() << std::endl;
    }

    static int xdim();                         // defined below
    static int ydim();                         // defined below
    static int number_rocks();                 // defined below
    static const loc_t& rock_location(int r);  // defined below

    size_t hash() const {
        std::cout << "warning: hash value is zero!" << std::endl;
        return 0; // CHECK
        //return Utils::jenkins_one_at_a_time_hash(beams_);
    }

    void copy_beams(const belief_state_t &bel) {
        assert((beams_ != 0) && (bel.beams_ != 0));
        for( int r = 0; r < number_rocks(); ++r )
            beams_[r] = bel.beams_[r];
    }
    void set_initial_beams() {
        assert(beams_ != 0);
        for( int r = 0; r < number_rocks(); ++r )
            beams_[r] = 0.5;
    }
    int compare_beams(const belief_state_t &bel) const {
        assert((beams_ != 0) && (bel.beams_ != 0));
        for( int r = 0; r < number_rocks(); ++r ) {
            if( beams_[r] < bel.beams_[r] )
                return -1;
            else if( beams_[r] > bel.beams_[r] )
                return 1;
        }
        return 0;
    }

    const belief_state_t& operator=(const belief_state_t &bel) {
        loc_ = bel.loc_;
        copy_beams(bel);
        sampled_ = bel.sampled_;
        hidden_ = bel.hidden_;
        return *this;
    }
    bool operator==(const belief_state_t &bel) const {
        return (loc_ == bel.loc_) && (compare_beams(bel) == 0) && (sampled_ == bel.sampled_) && (hidden_ == bel.hidden_);
    }
    bool operator!=(const belief_state_t &bel) const {
        return !(*this == bel);
    }
    bool operator<(const belief_state_t &bel) const {
        return (loc_ < bel.loc_) ||
          ((loc_ == bel.loc_) && (sampled_ < bel.sampled_)) ||
          ((loc_ == bel.loc_) && (sampled_ == bel.sampled_) && (compare_beams(bel) < 0));
    }

    int value(int vid) const {
        if( vid == 0 ) {
            return loc_.as_integer(xdim(), ydim());
        } else if( vid <= number_rocks() ) {
            int r = vid - 1;
            assert((r >= 0) && (r < number_rocks()));
            return sampled_.bit(r);
        } else {
            int r = vid - 1 - number_rocks();
            assert((r >= 0) && (r < number_rocks()));
            if( beams_[r] == 1 )
                return 1;
            else if( beams_[r] == 0 )
                return 0;
            else
                return -1;
        }
    }
    void fill_values_for_variable(int vid, std::vector<float> &probabilities) const {
        if( vid == 0 ) {
            probabilities = std::vector<float>(xdim() * ydim(), 0);
            probabilities[loc_.as_integer(xdim(), ydim())] = 1;
        } else if( vid <= number_rocks() ) {
            int r = vid - 1;
            assert((r >= 0) && (r < number_rocks()));
            probabilities = std::vector<float>(2, 0);
            probabilities[sampled_.bit(r)] = 1;
        } else {
            int r = vid - 1 - number_rocks();
            assert((r >= 0) && (r < number_rocks()));
            probabilities = std::vector<float>(2, 0);
            probabilities[0] = 1 - beams_[r];
            probabilities[1] = beams_[r];
        }
    }
    void fill_values_for_variable(int vid, std::vector<std::pair<int, float> > &values) const {
        if( vid == 0 ) {
            values.push_back(std::make_pair(loc_.as_integer(xdim(), ydim()), 1));
        } else if( vid <= number_rocks() ) {
            int r = vid - 1;
            assert((r >= 0) && (r < number_rocks()));
            values.push_back(std::make_pair(sampled_.bit(r), 1));
        } else {
            int r = vid - 1 - number_rocks();
            assert((r >= 0) && (r < number_rocks()));
            if( beams_[r] != 1 )
                values.push_back(std::make_pair(0, 1 - beams_[r]));
            if( beams_[r] != 0 )
                values.push_back(std::make_pair(1, beams_[r]));
        }
    }

    const loc_t& loc() const {
        return loc_;
    }

    void sample(int r) {
        // need to set rock at current location as bad
        sampled_.set_bit(r, 1);
    }

    void apply_sense(int obs, int r, float alpha) {
        assert((r >= 0) && (r < number_rocks()));
        assert(sampled_.bit(r) == 0);
        const loc_t &rloc = rock_location(r);
        float d = rloc.euclidean_distance(loc_);
        float efficiency = expf(-d * alpha);
        assert((efficiency > 0) && (efficiency <= 1));
        float probability_correct_reading = efficiency + (1 - efficiency) * 0.5;

        float p_good = beams_[r];
        float p_not_good = 1 - beams_[r];
        if( obs == OBS_GOOD ) {
            p_not_good *= (1 - probability_correct_reading);
            p_good *= probability_correct_reading;
        } else {
            p_not_good *= probability_correct_reading;
            p_good *= (1 - probability_correct_reading);
        }

        // normalize
        beams_[r] = p_good / (p_good + p_not_good);
    }

    std::pair<float, float> probability_sense(int r, float alpha) const { // pair is (P(good), P(!good))
        assert((r >= 0) && (r < number_rocks()));
        assert(sampled_.bit(r) == 0);
        std::pair<float, float> p(0, 0);
        if( sampled_.bit(r) ) {
            p.second = 1;
        } else {
            const loc_t &rloc = rock_location(r);
            float d = rloc.euclidean_distance(loc_);
            float efficiency = expf(-d * alpha);
            assert((efficiency > 0) && (efficiency <= 1));
            float probability_correct_reading = efficiency + (1 - efficiency) * 0.5;

            // obs = good
            p.first = beams_[r] * probability_correct_reading;
            p.first += (1 - beams_[r]) * (1 - probability_correct_reading);

            // obs = not good
            p.second = beams_[r] * (1 - probability_correct_reading);
            p.second += (1 - beams_[r]) * probability_correct_reading;

            // normalize
            float m = p.first + p.second;
            p.first /= m;
            p.second /= m;
        }
        assert(fabs(1 - p.first - p.second) < 1e-6);
        return p;
    }

    void print(std::ostream &os) const {
        os << "[loc=" << loc_
           << ",beams=[";
        for( int r = 0; r < number_rocks(); ++r ) {
            os << "{r=" << r
               << ",dist={" << std::setprecision(2) << 1 - beams_[r]
               << "," << std::setprecision(2) << beams_[r]
               << "}}";
            if( 1 + r < number_rocks() ) os << ",";
        }
        os << "],sampled=" << sampled_ << ",hidden=" << hidden_ << "]" << std::flush;
    }

    friend class pomdp_t;
};

inline std::ostream& operator<<(std::ostream &os, const belief_state_t &bel) {
    bel.print(os);
    return os;
}

class pomdp_t : public POMDP::pomdp_t<belief_state_t> {
  protected:
    const int xdim_;
    const int ydim_;
    const int number_rocks_;
    const float alpha_;  // sensor efficiency
    const bool sample_rock_locations_with_init_;

    // There is 1 binary (hidden) variable for status of each rock. There are known
    // variables for location of agent and whether a rock has been sampled.
    int number_variables_; // 2 for each rock, 1 for location
    int number_actions_;   // 4 for movements, 1 for sample, 1 for sense each rock
    int number_beams_;     // 1 for each rock

    // The following are set when creating a new initial belief state.
    // At that time, rocks are placed in the grid, status of rocks are
    // determined, and the constraint graph and beams are constructed.
    // These elements are only modified in init() that creates initial
    // belief state
    mutable belief_state_t *init_;
    mutable std::vector<loc_t> rock_locations_;
    mutable std::map<int, int> rock_locations_map_;

    enum { GoNorth, GoEast, GoSouth, GoWest, Sample, Sense };

  public:
    pomdp_t(int xdim, int ydim, int number_rocks, float alpha, bool sample_rock_locations_with_init)
      : POMDP::pomdp_t<belief_state_t>(DISCOUNT),
        xdim_(xdim),
        ydim_(ydim),
        number_rocks_(number_rocks),
        alpha_(alpha),
        sample_rock_locations_with_init_(sample_rock_locations_with_init),
        init_(0) {
        number_variables_ = 1 + 2 * number_rocks_;
        number_actions_ = 5 + number_rocks_;
        number_beams_ = number_rocks_;
        if( !sample_rock_locations_with_init_ )
            sample_rock_locations();
    }
    virtual ~pomdp_t() {
        delete init_;
    }

    void sample_rock_locations() const {
        std::vector<int> locations(xdim_ * ydim_);
        for( int i = 0; i < xdim_ * ydim_; ++i )
            locations[i] = i;

        rock_locations_.clear();
        rock_locations_map_.clear();
        rock_locations_.reserve(number_rocks_);
        for( int r = 0; r < number_rocks_; ++r ) {
            int i = Random::random(0, locations.size());
            int loc = locations[i];
            locations[i] = locations.back();
            locations.pop_back();
            rock_locations_.push_back(loc_t(loc, xdim_, ydim_));
            rock_locations_map_.insert(std::make_pair(rock_locations_.back().as_integer(xdim_, ydim_), rock_locations_map_.size()));
#if 1//def DEBUG
            std::cout << "rock: r=" << r << " --> loc=" << loc_t(loc, xdim_, ydim_) << std::endl;
#endif
        }
    }
 
    bool is_sense_action(Problem::action_t action) const {
        return (action > Sample) && (action < Sense + number_rocks_);
    }
    int sensed_rock(Problem::action_t action) const {
        assert(is_sense_action(action));
        return action - Sense;
    }

    virtual Problem::action_t number_actions(const belief_state_t &bel) const {
        return number_actions_;
    }
    virtual const belief_state_t& init() const {
        // if requested, we first sample different locations for
        // the rocks. These are sampled but remain fixed and thus
        // don't require variable/beams for them.
        if( sample_rock_locations_with_init_ )
            sample_rock_locations();

        // we now create a new belief state and
        // sample status for each rock
        delete init_;
        Bitmap::bitmap_t rocks;
        for( int r = 0; r < number_rocks_; ++r ) {
            int status = Random::random(0, 2);
            rocks.set_bit(r, status);
        }
#if 1//def DEBUG
        std::cout << "rock status: " << rocks << std::endl;
#endif

        // we must create constraint graph before any belief state is created
        init_ = new belief_state_t(0, rocks);
        return *init_;
    }
    virtual bool terminal(const belief_state_t &bel) const {
        bool is_terminal = true;
        for( int r = 0; is_terminal && (r < number_rocks_); ++r ) {
            if( (bel.sampled_.bit(r) == 0) && (bel.beams_[r] > 0) )
                is_terminal = false;
        }
        return is_terminal;
    }
    virtual bool dead_end(const belief_state_t &bel) const {
        return false;
    }
    virtual bool applicable(const belief_state_t &bel, ::Problem::action_t a) const {
        if( a == GoNorth ) {
            return 1 + bel.loc_.r_ < ydim_;
        } else if( a == GoEast ) {
            return 1 + bel.loc_.c_ < xdim_;
        } else if( a == GoSouth ) {
            return bel.loc_.r_ > 0;
        } else if( a == GoWest ) {
            return bel.loc_.c_ > 0;
        } else if( a == Sample ) {
            return rock_locations_map_.find(bel.loc_.as_integer(xdim_, ydim_)) != rock_locations_map_.end();
        } else {
            assert(is_sense_action(a));
            int r = sensed_rock(a);
            return bel.sampled_.bit(r) == 0;
        }
    }
    virtual float min_absolute_cost() const { return 1; }
    virtual float max_absolute_cost() const { return 10; }
    virtual float cost(const belief_state_t &bel, Problem::action_t a) const {
        if( a == Sample ) {
            assert(rock_locations_map_.find(bel.loc_.as_integer(xdim_, ydim_)) != rock_locations_map_.end());
            int r = rock_locations_map_.at(bel.loc_.as_integer(xdim_, ydim_));
            return bel.sampled_.bit(r) ? 10 : 10 * (1 - bel.beams_[r] - bel.beams_[r]);
        } else {
            return 1;
        }
    }
    virtual int max_action_branching() const {
        return number_actions_;
    }
    virtual int max_state_branching() const {
        return 2;
    }
    virtual void next(const belief_state_t &bel, Problem::action_t a, std::vector<std::pair<belief_state_t, float> > &outcomes) const {
        ++expansions_;
        outcomes.clear();

#ifdef DEBUG
        std::cout << "next: bel=" << bel << ", action=" << action_name(a) << std::endl;
#endif

        if( !is_sense_action(a) ) {
            outcomes.push_back(std::make_pair(belief_state_t(bel), 1.0));
            if( a == GoNorth ) {
                outcomes.back().first.loc_.move_north(xdim_, ydim_);
            } else if( a == GoEast ) {
                outcomes.back().first.loc_.move_east(xdim_, ydim_);
            } else if( a == GoSouth ) {
                outcomes.back().first.loc_.move_south(xdim_, ydim_);
            } else if( a == GoWest ) {
                outcomes.back().first.loc_.move_west(xdim_, ydim_);
            } else if( a == Sample ) {
                assert(rock_locations_map_.find(bel.loc_.as_integer(xdim_, ydim_)) != rock_locations_map_.end());
                int r = rock_locations_map_.at(bel.loc_.as_integer(xdim_, ydim_));
                outcomes.back().first.sample(r);
            } else {
                std::cout << Utils::error() << "unknown action a=" << a << std::endl;
                exit(-1);
            }
#ifdef DEBUG
            std::cout << "    " << outcomes.size() << ". next-bel=" << outcomes.back().first << std::endl;
#endif
        } else {
            int r = sensed_rock(a);
            outcomes.reserve(2);
            //std::cout << "PROBABILITY-SENSE.0: loc=" << bel.loc_ << ", a=" << a << ", r=" << r << std::endl;
            std::pair<float, float> p = bel.probability_sense(r, alpha_); // pair is (P(good), P(!good))
            //std::cout << "PROBABILITY-SENSE.1: p: " << p.first << " " << p.second << std::endl;
            if( p.first > 0 ) {
                outcomes.push_back(std::make_pair(belief_state_t(bel), p.first));
                outcomes.back().first.apply_sense(OBS_GOOD, r, alpha_);
#ifdef DEBUG
                std::cout << "    " << outcomes.size() << ". next-bel=" << outcomes.back().first << std::endl;
#endif
            }
            if( p.second > 0 ) {
                outcomes.push_back(std::make_pair(belief_state_t(bel), p.second));
                outcomes.back().first.apply_sense(OBS_NOT_GOOD, r, alpha_);
#ifdef DEBUG
                std::cout << "    " << outcomes.size() << ". next-bel=" << outcomes.back().first << std::endl;
#endif
            }
        }
    }

    // POMDP virtual methods
    virtual int number_beams() const {
        return number_beams_;
    }
    virtual int number_variables() const {
        return number_variables_;
    }
    virtual int number_determined_variables() const {
        return 1 + number_rocks_;
    }

    virtual bool determined(int vid) const {
        return vid <= number_rocks_;
    }
    virtual int domain_size(int vid) const {
        return vid == 0 ? xdim_ * ydim_ : 2;
    }
    virtual int value(const belief_state_t &belief_state, int vid) const {
        return belief_state.value(vid);
    }
    virtual void fill_values_for_variable(const belief_state_t &belief, int vid, std::vector<float> &probabilities) const {
        belief.fill_values_for_variable(vid, probabilities);
    }
    virtual void fill_values_for_variable(const belief_state_t &belief, int vid, std::vector<std::pair<int, float> > &values) const {
        belief.fill_values_for_variable(vid, values);
    }

    virtual void apply_action(belief_state_t &bel_a, Problem::action_t a) const {
        if( !is_sense_action(a) ) {
            if( a == GoNorth ) {
                bel_a.loc_.move_north(xdim_, ydim_);
            } else if( a == GoEast ) {
                bel_a.loc_.move_east(xdim_, ydim_);
            } else if( a == GoSouth ) {
                bel_a.loc_.move_south(xdim_, ydim_);
            } else if( a == GoWest ) {
                bel_a.loc_.move_west(xdim_, ydim_);
            } else if( a == Sample ) {
                assert(rock_locations_map_.find(bel_a.loc_.as_integer(xdim_, ydim_)) != rock_locations_map_.end());
                int r = rock_locations_map_.at(bel_a.loc_.as_integer(xdim_, ydim_));
                bel_a.sample(r);
            } else {
                std::cout << Utils::error() << "unknown action a=" << a << std::endl;
                exit(-1);
            }
        } else {
            /* Sense action has no effect on states, it only returns information */
        }
    }
    virtual void apply_obs(belief_state_t &bel_ao, Problem::action_t a, POMDP::observation_t obs) const {
        if( is_sense_action(a) ) {
            int r = sensed_rock(a);
            bel_ao.apply_sense(obs, r, alpha_);
        }
    }
    virtual POMDP::observation_t sample_observation_using_hidden_state(const belief_state_t &bel, const belief_state_t &bel_a, Problem::action_t a) const {
        POMDP::observation_t obs = OBS_NOT_GOOD; // default sensing
        if( is_sense_action(a) ) {
            int r = sensed_rock(a);
            assert(bel.sampled_.bit(r) == 0);
            const loc_t &rloc = rock_locations_[r];
            float d = rloc.euclidean_distance(bel.loc_);
            float efficiency = expf(-d * alpha_);
            assert((efficiency > 0) && (efficiency <= 1));
            float probability_correct_reading = efficiency + (1 - efficiency) * 0.5;
            if( Random::uniform() < probability_correct_reading )
                obs = bel.hidden_.bit(r) == 1 ? OBS_GOOD : OBS_NOT_GOOD;
            else
                obs = bel.hidden_.bit(r) == 1 ? OBS_NOT_GOOD : OBS_GOOD;
        }
        return obs;
    }

    virtual std::string action_name(Problem::action_t a) const {
        if( a == GoNorth )
            return std::string("go-north()");
        else if( a == GoEast )
            return std::string("go-east()");
        else if( a == GoSouth )
            return std::string("go-south()");
        else if( a == GoWest )
            return std::string("go-west()");
        else if( a == Sample )
            return std::string("sample()");
        else if( is_sense_action(a) )
            return std::string("sense(r=") + std::to_string(sensed_rock(a)) + std::string(")");
        else
            return std::string("unknown");
    }

    virtual void print(std::ostream &os) const {
        os << Utils::error() << "not implemented yet" << std::endl;
        assert(0); // CHECK
    }

    friend class belief_state_t;
};

inline std::ostream& operator<<(std::ostream &os, const pomdp_t &p) {
    p.print(os);
    return os;
}

inline int belief_state_t::xdim() {
    assert(pomdp_ != 0);
    return pomdp_->xdim_;
}

inline int belief_state_t::ydim() {
    assert(pomdp_ != 0);
    return pomdp_->ydim_;
}

inline int belief_state_t::number_rocks() {
    assert(pomdp_ != 0);
    return pomdp_->number_rocks_;
}

inline const loc_t& belief_state_t::rock_location(int r) {
    assert(pomdp_ != 0);
    assert(r < int(pomdp_->rock_locations_.size()));
    return pomdp_->rock_locations_[r];
}

#undef DEBUG

