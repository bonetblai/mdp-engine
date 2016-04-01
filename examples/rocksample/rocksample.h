#include <iostream>
#include <iomanip>
#include <strings.h>

#include "../binary-search/bitmap.h"

#define DISCOUNT            1

//#define DEBUG

/*
 * Parameters
 *
 * xdim, ydim -- grid dimensions
 * h1, h2, ... -- heights
 *
 * Sensor detects good rocks at euclidean distance <= n when antenna is at height h_n
 *
 * Actions
 *
 *  (:action move
 *      :parameters (?p ?q - location)
 *      :precondition (and (agent-at ?p) (adj ?p ?q) (antenna-height h0) (not (need-start)))
 *      :effect (and (agent-at ?q) (not (agent-at ?p)))
 *  )
 *
 *
 *  (:action sample
 *      :parameters (?r - rock ?p - location)
 *      :precondition (and (agent-at ?p) (rock-at ?r ?p) (good ?r) (not (need-start)))
 *      :effect (finished ?r)
 *  )
 *
 *  (:action skip
 *      :parameters (?r - rock)
 *      :precondition (and (not (good ?r)) (not (need-start)))
 *      :effect (finished ?r)
 *  )
 *
 *  (:action raise-antenna
 *      :precondition (not (need-start))
 *      :effect
 *          (forall (?h1 ?h2 - height)
 *              (when (and (antenna-height ?h1) (next-height ?h1 ?h2)) (and (not (antenna-height ?h1)) (antenna-height ?h2)))
 *          )
 *  )
 *
 *  (:action lower-antenna
 *      :precondition (not (need-start))
 *      :effect
 *          (forall (?h1 ?h2 - height)
 *              (when (and (antenna-height ?h1) (next-height ?h2 ?h1)) (and (not (antenna-height ?h1)) (antenna-height ?h2)))
 *          )
 *  )
 *
 *  (:action activate-sensor
 *      :parameters (?p - location ?h - height)
 *      :precondition (and (agent-at ?p) (antenna-height ?h) (not (need-start)))
 *      :sensing
 *          (model-for (rocks-in-range ?h ?p)
 *                     (good-rocks-in-sensing-range ?h ?p)
 *                     (exists (?r - rock ?q - location) (and (rock-at ?r ?q) (in-sensing-range ?h ?p ?q) (not (finished ?r)) (good ?r)))
 *          )
 *          (model-for (rocks-in-range ?h ?p)
 *                     (not (good-rocks-in-sensing-range ?h ?p))
 *                     (forall (?r - rock ?q - location)
 *                         (or (not (rock-at ?r ?q)) (not (in-sensing-range ?h ?p ?q)) (finished ?r) (not (good ?r)))
 *                     )
 *          )
 *  )
 *
 */

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

    bool operator==(const loc_t &loc) const {
        return (r_ == loc.r_) && (c_ == loc.c_);
    }
    bool operator<(const loc_t &loc) const {
        return (r_ < loc.r_) || ((r_ == loc.r_) && (c_ < loc.c_));
    }

    void print(std::ostream &os) const {
        os << "(" << c_ << "," << r_ << ")" << std::flush;
    }
};

inline std::ostream& operator<<(std::ostream &os, const loc_t &loc) {
    loc.print(os);
    return os;
}

struct beam_t {
    int value_; // 0=contains only BAD, 1=contains only GOOD, 2=empty, 3=contains both

    struct const_iterator {
        int marker_;

        const_iterator(int marker) : marker_(marker) { }

        bool operator==(const const_iterator &it) const {
            return marker_ == it.marker_;
        }
        bool operator!=(const const_iterator &it) const {
            return !(*this == it);
        }
        const const_iterator& operator++() {
            ++marker_;
            return *this;
        }

        int value() const {
            return marker_;
        }
    }; // const_iterator

    beam_t(int value = 3) : value_(value) { }
    ~beam_t() { }

    const beam_t& operator=(const beam_t &beam) {
        value_ = beam.value_;
        return *this;
    }
    bool operator==(const beam_t &beam) const {
        return value_ == beam.value_;
    }
    bool operator!=(const beam_t &beam) const {
        return !(*this == beam);
    }
    bool operator<(const beam_t &beam) const {
        return value_ < beam.value_;
    }

    int cardinality() const {
        return value_ == 3 ? 2 : (value_ == 2 ? 0 : 1);

    }
    unsigned hash() const {
        return value_;
    }

    virtual const_iterator begin() const {
        return const_iterator(0);
    }
    virtual const_iterator end() const {
        return const_iterator(value_ == 3 ? 3 : (value_ == 2 ? 0 : 2));
    }

    void print(std::ostream &os) const {
        if( value_ == 0 )
            os << "{bad}";
        else if( value_ == 1 )
            os << "{good}";
        else if( value_ == 2 )
            os << "{}";
        else
            os << "{bad,good}";
    }
};

inline std::ostream& operator<<(std::ostream &os, const beam_t &beam) {
    beam.print(os);
    return os;
}

class belief_state_t {
  protected:
    loc_t loc_;
    int antenna_height_;
    std::vector<beam_t> beams_;
    Bitmap::bitmap_t sampled_;
    Bitmap::bitmap_t skipped_;
    Bitmap::bitmap_t hidden_;

    static int xdim_;
    static int ydim_;
    static int number_rocks_;

  public:
    belief_state_t(int loc = 0, int antenna_height = 0)
      : loc_(loc, xdim_, ydim_),
        antenna_height_(antenna_height),
        sampled_(0),
        skipped_(0) {
    }
    belief_state_t(int loc, int antenna_height, const Bitmap::bitmap_t &hidden)
      : loc_(loc, xdim_, ydim_),
        antenna_height_(antenna_height),
        sampled_(0),
        skipped_(0),
        hidden_(hidden) {
        beams_ = std::vector<beam_t>(number_rocks_, beam_t());
    }
    belief_state_t(const belief_state_t &bel)
      : loc_(bel.loc_),
        antenna_height_(bel.antenna_height_),
        beams_(bel.beams_),
        sampled_(bel.sampled_),
        skipped_(bel.skipped_),
        hidden_(bel.hidden_) {
    }
    belief_state_t(belief_state_t &&bel)
      : loc_(bel.loc_),
        antenna_height_(bel.antenna_height_),
        beams_(std::move(bel.beams_)),
        sampled_(std::move(bel.sampled_)),
        skipped_(std::move(bel.skipped_)),
        hidden_(bel.hidden_) {
    }
    ~belief_state_t() { }

    static void set_static_members(int xdim, int ydim, int number_rocks) {
        xdim_ = xdim;
        ydim_ = ydim;
        number_rocks_ = number_rocks;
    }

    size_t hash() const {
        //return beam_.hash(); // CHECK: use something for strings, like jenkins
    }

    const belief_state_t& operator=( const belief_state_t &bel) {
        loc_ = bel.loc_;
        antenna_height_ = bel.antenna_height_;
        beams_ = bel.beams_;
        sampled_ = bel.sampled_;
        skipped_ = bel.skipped_;
        hidden_ = bel.hidden_;
        return *this;
    }
    bool operator==(const belief_state_t &bel) const {
        return (loc_ == bel.loc_) &&
          (antenna_height_ == bel.antenna_height_) &&
          (beams_ == bel.beams_) &&
          (sampled_ == bel.sampled_) &&
          (skipped_ == bel.skipped_) &&
          (hidden_ == bel.hidden_);
    }
    bool operator!=(const belief_state_t &bel) const {
        return !(*this == bel);
    }
    bool operator<(const belief_state_t &bel) const {
        return (loc_ < bel.loc_) ||
          ((loc_ == bel.loc_) && (antenna_height_ < bel.antenna_height_)) ||
          ((loc_ == bel.loc_) && (antenna_height_ == bel.antenna_height_) && (beams_ < bel.beams_)) ||
          ((loc_ == bel.loc_) && (antenna_height_ == bel.antenna_height_) && (beams_ == bel.beams_) && (sampled_ < bel.sampled_)) ||
          ((loc_ == bel.loc_) && (antenna_height_ == bel.antenna_height_) && (beams_ == bel.beams_) && (sampled_ == bel.sampled_) && (skipped_ < bel.skipped_)) ||
          ((loc_ == bel.loc_) && (antenna_height_ == bel.antenna_height_) && (beams_ == bel.beams_) && (sampled_ == bel.sampled_) && (skipped_ == bel.skipped_) && (hidden_ < bel.hidden_));
    }

    const beam_t& beam(int bid) const {
        assert(bid < beams_.size());
        return beams_[bid];
    }

    int cardinality() const {
        //return beam_.cardinality(); // CHECK
        assert(0); // CHECK
    }

    void print(std::ostream &os) const {
        os << "[loc=" << loc_
           << ", height=" << antenna_height_
           << ", beams=[";
        for( int i = 0; i < int(beams_.size()); ++i ) {
            os << beams_[i];
            if( i + 1 < int(beams_.size()) ) os << ",";
        }
        os << "], hidden=" << hidden_ << "]" << std::flush;
    }
    friend class pomdp_t;
};

inline std::ostream& operator<<(std::ostream &os, const belief_state_t &bel) {
    bel.print(os);
    return os;
}

struct feature_t : public POMDP::feature_t<belief_state_t> {
    feature_t(const belief_state_t &bel) {
#if 0 // CHECK
#ifdef DEBUG
        std::cout << "bel=" << bel << std::endl;
        std::cout << "marginal:";
#endif
        marginals_ = std::vector<std::vector<float> >(1);
        marginals_[0] = std::vector<float>(bitmap_t::dim_, 0);
        float p = 1.0 / bel.cardinality();
        for( beam_t::const_iterator it = bel.beam(0).begin(); it != bel.beam(0).end(); ++it ) {
            assert((it.value() >= 0) && (it.value() < bitmap_t::dim_));
            marginals_[0][it.value()] = p;
#  ifdef DEBUG
            std::cout << " " << p << "@" << it.value() << std::flush;
#  endif
        }
#ifdef DEBUG
        std::cout << std::endl;
#endif
#endif
        assert(0); // CHECK
    }
    virtual ~feature_t() { }
};

class pomdp_t : public POMDP::pomdp_t<belief_state_t> {
  protected:
    int xdim_;
    int ydim_;
    int number_rocks_;
    int max_antenna_height_;

    std::vector<int> rock_locations_;

    // There is 1 binary (hidden) variable for status of each rock. There are known
    // variables for location of agent and height of antenna.
    int number_variables_; // 1 for each rock, 1 for location, 1 for antenna
    int number_actions_;   // 4 for movements, 2 for antenna, 1 for sensor, 1 for sample, 1 for skip
    int number_beams_;     // 1 for each rock, each containing gthe status of the rock

    std::vector<POMDP::pomdp_t<belief_state_t>::varset_t> varsets_;

    mutable belief_state_t init_tmp_;

    enum { MoveNorth, MoveEast, MoveSouth, MoveWest, RaiseAntenna, LowerAntenna, Sense };

  public:
    pomdp_t(int xdim, int ydim, int number_rocks, int max_antenna_height)
      : POMDP::pomdp_t<belief_state_t>(DISCOUNT),
        xdim_(xdim),
        ydim_(ydim),
        number_rocks_(number_rocks),
        max_antenna_height_(max_antenna_height) {
        number_variables_ = 2 + number_rocks_;
        number_actions_ = 4 + 2 + 3;
        number_beams_ = number_rocks_;

        // we must different locations for the rocks. These are sampled
        // but fixed and thus don't require variables/beams for them.
        std::vector<int> locations(xdim_ * ydim_);
        for( int i = 0; i < xdim_ * ydim_; ++i )
            locations[i] = i;

        rock_locations_.reserve(number_rocks_);
        for( int r = 0; r < number_rocks_; ++r ) {
            int i = Random::random(0, locations.size());
            int loc = locations[i];
            locations[i] = locations.back();
            locations.pop_back();
            rock_locations_.push_back(loc);
            std::cout << "rock: r=" << r << " --> loc=" << loc_t(loc, xdim_, ydim_) << std::endl;
        }
    }
    virtual ~pomdp_t() { }

    virtual Problem::action_t number_actions(const belief_state_t &bel) const {
        return number_actions_;
    }
    virtual const belief_state_t& init() const {
        Bitmap::bitmap_t rocks;
        for( int r = 0; r < number_rocks_; ++r ) {
            int status = Random::random(0, 2);
            rocks.set_bit(r, status);
        }
        init_tmp_ = belief_state_t(0, 0, rocks);
        return init_tmp_;
    }
    virtual bool terminal(const belief_state_t &bel) const {
        return bel.sampled_.popcount() + bel.skipped_.popcount() == number_rocks_;
    }
    virtual bool dead_end(const belief_state_t &bel) const {
        return false;
    }
    virtual bool applicable(const belief_state_t &bel, ::Problem::action_t a) const {
        if( (a >= MoveNorth) && (a <= MoveWest) ) {
            return bel.antenna_height_ == 0;
        } else if( a == RaiseAntenna ) {
            return bel.antenna_height_ < max_antenna_height_;
        } else if( a == LowerAntenna ) {
            return bel.antenna_height_ > 0;
        } else if( a == Sense ) {
            return true; // sensor can be activated at any time
        } else if( (a > Sense) && (a <= number_rocks_ + Sense) ) { // Sample actions, 1 for each rock
            // rock must not be sampled/skipped and it must be known to be a good rock
            int rock = a - Sense - 1;
            assert((rock >= 0) && (rock < number_rocks_));
            return bel.beams_[rock].value_ == 1;
        } else if( (a > number_rocks_ + Sense) && (a <= 2 * number_rocks_ + Sense) ) { // Skip actions, 1 for each rock
            // rock must not be sampled/skipped and it must be known to be a bad rock
            int rock = a - Sense - number_rocks_ - 1;
            assert((rock >= 0) && (rock < number_rocks_));
            return bel.beams_[rock].value_ == 0;
        } else {
            std::cout << Utils::error() << "unknown action a=" << a << std::endl;
            return false;
        }
    }
    virtual float min_absolute_cost() const { return 1; } // CHECK
    virtual float max_absolute_cost() const { return 1; } // CHECK
    virtual float cost(const belief_state_t &bel, Problem::action_t a) const {
        return 1; // CHECK
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
        outcomes.reserve(1);

        if( a == MoveNorth ) {
            belief_state_t next_bel(bel);
            next_bel.loc_.move_north(xdim_, ydim_);
            outcomes.push_back(std::make_pair(next_bel, 1.0));
        } else if( a == MoveEast ) {
            belief_state_t next_bel(bel);
            next_bel.loc_.move_east(xdim_, ydim_);
            outcomes.push_back(std::make_pair(next_bel, 1.0));
        } else if( a == MoveSouth ) {
            belief_state_t next_bel(bel);
            next_bel.loc_.move_south(xdim_, ydim_);
            outcomes.push_back(std::make_pair(next_bel, 1.0));
        } else if( a == MoveWest ) {
            belief_state_t next_bel(bel);
            next_bel.loc_.move_west(xdim_, ydim_);
            outcomes.push_back(std::make_pair(next_bel, 1.0));
        } else if( a == RaiseAntenna ) {
            belief_state_t next_bel(bel);
            ++next_bel.antenna_height_;
            outcomes.push_back(std::make_pair(next_bel, 1.0));
        } else if( a == LowerAntenna ) {
            belief_state_t next_bel(bel);
            --next_bel.antenna_height_;
            outcomes.push_back(std::make_pair(next_bel, 1.0));
        } else if( a == Sense ) {
            outcomes.reserve(2);
            // CHECK: assume two outputs and filter beams accordingly. Use auxiliary function...
        } else if( (a > Sense) && (a <= number_rocks_ + Sense) ) { // Sample actions, 1 for each rock
            int rock = a - Sense - 1;
            assert((rock >= 0) && (rock < number_rocks_));
            belief_state_t next_bel(bel);
            assert((next_bel.sampled_.bit(rock) == 0) && (next_bel.skipped_.bit(rock) == 0));
            next_bel.sampled_.set_bit(rock, 1);
            outcomes.push_back(std::make_pair(next_bel, 1.0));
        } else if( (a > number_rocks_ + static_cast<int>(Sense)) && (a <= 2 * number_rocks_ + static_cast<int>(Sense)) ) { // Skip actions, 1 for each rock
            int rock = a - Sense - number_rocks_ - 1;
            assert((rock >= 0) && (rock < number_rocks_));
            belief_state_t next_bel(bel);
            assert((next_bel.sampled_.bit(rock) == 0) && (next_bel.skipped_.bit(rock) == 0));
            next_bel.skipped_.set_bit(rock, 1);
            outcomes.push_back(std::make_pair(next_bel, 1.0));
        } else {
            std::cout << Utils::error() << "unknown action a=" << a << std::endl;
            exit(-1);
        }
    }

    // POMDP virtual methods
    virtual int num_variables() const {
        return number_variables_;
    }
    virtual int num_beams() const {
        return number_beams_;
    }
    virtual const POMDP::pomdp_t<belief_state_t>::varset_t& varset(int bid) const {
        //return varsets_[0]; // CHECK
        assert(0); // CHECK
    }
    virtual int cardinality(const belief_state_t &bel) const {
        return bel.cardinality();
    }
    virtual POMDP::feature_t<belief_state_t> *get_feature(const belief_state_t &bel) const {
        return new feature_t(bel);
    }
    virtual void clean_feature(const POMDP::feature_t<belief_state_t> *feature) const {
        delete feature;
    }

    virtual void apply_action(belief_state_t &bel_a, Problem::action_t a) const {
#if 0
        /* real work is done below */
#endif
        assert(0); // CHECK
    }
    virtual void apply_obs(belief_state_t &bel_ao, Problem::action_t a, POMDP::observation_t obs) const {
#if 0
        belief_state_t nbel = bel_ao.apply(a, obs);
        bel_ao = nbel;
#endif
        assert(0); // CHECK
    }
    virtual POMDP::observation_t sample_observation_using_hidden_state(const belief_state_t &bel, const belief_state_t &bel_a, Problem::action_t a) const {
#if 0
        return bel.hidden_ < a ? 0 : 1;
#endif
        assert(0); // CHECK
    }

    virtual void print(std::ostream &os) const {
        os << Utils::error() << "not implemented yet" << std::endl;
        assert(0);
    }
};

inline std::ostream& operator<<(std::ostream &os, const pomdp_t &p) {
    p.print(os);
    return os;
}

#undef DEBUG

