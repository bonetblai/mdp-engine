#include <iostream>
#include <iomanip>
#include <strings.h>

#include "../binary-search/bitmap.h"

#define DISCOUNT            1
#define OBS_NOT_GOOD        0
#define OBS_GOOD            1

//#define DEBUG

/*
 * Parameters
 *
 * xdim, ydim, max-antenna-height, number-rocks -- grid dimensions
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
 *  (:action sample
 *      :parameters (?r - rock ?p - location)
 *      :precondition (and (agent-at ?p) (rock-at ?r ?p) (good ?r) (not (need-start)))
 *      :effect (finished ?r)
 *  )
 *  (:action skip
 *      :parameters (?r - rock)
 *      :precondition (and (not (good ?r)) (not (need-start)))
 *      :effect (finished ?r)
 *  )
 *  (:action raise-antenna
 *      :precondition (not (need-start))
 *      :effect
 *          (forall (?h1 ?h2 - height)
 *              (when (and (antenna-height ?h1) (next-height ?h1 ?h2)) (and (not (antenna-height ?h1)) (antenna-height ?h2)))
 *          )
 *  )
 *  (:action lower-antenna
 *      :precondition (not (need-start))
 *      :effect
 *          (forall (?h1 ?h2 - height)
 *              (when (and (antenna-height ?h1) (next-height ?h2 ?h1)) (and (not (antenna-height ?h1)) (antenna-height ?h2)))
 *          )
 *  )
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

    void print(std::ostream &os) const {
        os << "(" << c_ << "," << r_ << ")" << std::flush;
    }
};

inline std::ostream& operator<<(std::ostream &os, const loc_t &loc) {
    loc.print(os);
    return os;
}

struct beam_t {
    typedef enum { bad = 0, good = 1, empty = 2, both = 3 } value_t;
    value_t value_; // 0=contains only BAD, 1=contains only GOOD, 2=empty, 3=contains both

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

    beam_t(value_t value = both) : value_(value) { }
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

    unsigned hash() const {
        return value_;
    }

    virtual const_iterator begin() const {
        return const_iterator(value_ == both ? bad : (value_ == empty ? bad : value_));
    }
    virtual const_iterator end() const {
        return const_iterator(value_ == both ? empty : (value_ == empty ? bad : 1 + value_));
    }

    void print(std::ostream &os) const {
        if( value_ == bad )
            os << "{bad}";
        else if( value_ == good )
            os << "{good}";
        else if( value_ == empty )
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
    static int max_antenna_height_;

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

    static void set_static_members(int xdim, int ydim, int number_rocks, int max_antenna_height) {
        xdim_ = xdim;
        ydim_ = ydim;
        number_rocks_ = number_rocks;
        max_antenna_height_ = max_antenna_height;
    }

    static int xdim() { return xdim_; }
    static int ydim() { return ydim_; }
    static int number_rocks() { return number_rocks_; }
    static int max_antenna_height() { return max_antenna_height_; }

    size_t hash() const {
        return Utils::jenkins_one_at_a_time_hash(beams_);
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

    const loc_t& loc() const { return loc_; }
    int antenna_height() const { return antenna_height_; }

    const beam_t& beam(int bid) const {
        assert(bid < beams_.size());
        return beams_[bid];
    }

    void raise_antenna() {
        assert(antenna_height_ < max_antenna_height_);
        ++antenna_height_;
    }
    void lower_antenna() {
        assert(antenna_height_ > 0);
        --antenna_height_;
    }
    void sample_rock(int r) {
        assert((r >= 0) && (r < number_rocks_));
        assert((sampled_.bit(r) == 0) && (skipped_.bit(r) == 0));
        sampled_.set_bit(r, 1);
    }
    void skip_rock(int r) {
        assert((r >= 0) && (r < number_rocks_));
        assert((sampled_.bit(r) == 0) && (skipped_.bit(r) == 0));
        skipped_.set_bit(r, 1);
    }

    void apply_sense(int obs, const std::vector<int> &rock_locations) {
        int number_good_rocks_in_sensing_range = 0;
        int number_unknown_rocks_in_sensing_range = 0;
        for( int r = 0; r < number_rocks_; ++r ) {
            assert(beams_[r].value_ != beam_t::empty);
            loc_t rloc(rock_locations[r], xdim_, ydim_);
            if( loc_.euclidean_distance(rloc) <= float(antenna_height_) ) {
                if( beams_[r].value_ == beam_t::good )
                    ++number_good_rocks_in_sensing_range;
                else if( beams_[r].value_ == beam_t::both )
                    ++number_unknown_rocks_in_sensing_range;
            }
        }

        if( obs == OBS_NOT_GOOD ) { // no good rocks in sensing range sensed
            assert(number_good_rocks_in_sensing_range == 0);
            if( number_unknown_rocks_in_sensing_range > 0 ) {
                for( int r = 0; r < number_rocks_; ++r ) {
                    loc_t rloc(rock_locations[r], xdim_, ydim_);
                    if( loc_.euclidean_distance(rloc) > float(antenna_height_) ) continue;
                    if( beams_[r].value_ == beam_t::both ) // unknown rock in sensing range, mark it as bad rock
                        beams_[r].value_ = beam_t::bad;
                }
            }
        } else { // a good rock in sensing range was sensed
            if( (number_good_rocks_in_sensing_range == 0) && (number_unknown_rocks_in_sensing_range == 1) ) {
                for( int r = 0; r < number_rocks_; ++r ) {
                    loc_t rloc(rock_locations[r], xdim_, ydim_);
                    if( loc_.euclidean_distance(rloc) > float(antenna_height_) ) continue;
                    if( beams_[r].value_ == beam_t::both ) { // unknown rock in sensing range, mark it as good rock and finish
                        beams_[r].value_ = beam_t::good;
                        break;
                    }
                }
            }
        }
    }
    std::pair<float, float> probability_sense(const std::vector<int> &rock_locations) const { // pair is (P(good), P(!good))
        int number_good_rocks = 0;
        int number_unknown_rocks = 0;
        for( int r = 0; r < number_rocks_; ++r ) {
            assert(beams_[r].value_ != beam_t::empty);
            loc_t rloc(rock_locations[r], xdim_, ydim_);
            if( loc_.euclidean_distance(rloc) <= float(antenna_height_) ) {
                if( beams_[r].value_ == beam_t::good ) { // good rock
                    ++number_good_rocks;
                    break;
                } else if( beams_[r].value_ == beam_t::both ) { // unknown rock
                    ++number_unknown_rocks;
                }
            }
        }
        float p_not_good = number_good_rocks > 0 ? 0.0 : (number_unknown_rocks > 0 ? powf(0.5, number_unknown_rocks) : 1.0);
        return std::make_pair(1 - p_not_good, p_not_good);
    }

    void print(std::ostream &os) const {
        os << "[loc=" << loc_
           << ",height=" << antenna_height_
           << ",beams=[";
        for( int i = 0; i < int(beams_.size()); ++i ) {
            os << beams_[i];
            if( i + 1 < int(beams_.size()) ) os << ",";
        }
        os << "],sampled=" << sampled_ << ",skipped=" << skipped_ << ",hidden=" << hidden_ << "]" << std::flush;
    }
    friend class pomdp_t;
    friend struct feature_t;
};

inline std::ostream& operator<<(std::ostream &os, const belief_state_t &bel) {
    bel.print(os);
    return os;
}

struct feature_t : public POMDP::feature_t<belief_state_t> {
    feature_t(const belief_state_t &bel) {
        number_marginals_ = belief_state_t::number_rocks();
        std::vector<float> *marginals = new std::vector<float>[belief_state_t::number_rocks()];
        for( int r = 0; r < belief_state_t::number_rocks(); ++r ) {
            assert(bel.beam(r).value_ != beam_t::empty);
            marginals[r] = std::vector<float>(2, 0);
            if( bel.beam(r).value_ == beam_t::bad ) {
                marginals[r][beam_t::bad] = 1;
            } else if( bel.beam(r).value_ == beam_t::good ) {
                marginals[r][beam_t::good] = 1;
            } else if( bel.beam(r).value_ == beam_t::both ) {
                marginals[r][beam_t::bad] = 0.5;
                marginals[r][beam_t::good] = 0.5;
            }
        }
        marginals_ = marginals;

        number_fixed_tuples_ = 2;
        std::vector<int> *fixed_tuples = new std::vector<int>[2];
        fixed_tuples[0].reserve(belief_state_t::number_rocks());
        for( int r = 0; r < belief_state_t::number_rocks(); ++r )
            fixed_tuples[0].push_back(bel.sampled_.bit(r));
        fixed_tuples[1].reserve(belief_state_t::number_rocks());
        for( int r = 0; r < belief_state_t::number_rocks(); ++r )
            fixed_tuples[1].push_back(bel.skipped_.bit(r));
        fixed_tuples_ = fixed_tuples;
    }
    virtual ~feature_t() {
        assert(number_marginals_ == belief_state_t::number_rocks());
        assert(number_fixed_tuples_ == 2);
        assert(number_other_tuples_ == 0);
        delete[] fixed_tuples_;
        delete[] marginals_;
    }
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

    enum { GoNorth, GoEast, GoSouth, GoWest, RaiseAntenna, LowerAntenna, Sense };

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

    bool is_sample_action(Problem::action_t action) const {
        return (action > Sense) && (action <= number_rocks_ + Sense);
    }
    bool is_skip_action(Problem::action_t action) const {
        return (action > number_rocks_ + Sense) && (action <= 2 * number_rocks_ + Sense);
    }
    int sampled_rock(Problem::action_t action) const {
        assert(is_sample_action(action));
        return action - Sense - 1;
    }
    int skipped_rock(Problem::action_t action) const {
        assert(is_skip_action(action));
        return action - Sense - number_rocks_ - 1;
    }

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
        if( (a >= GoNorth) && (a <= GoWest) ) {
            return bel.antenna_height_ == 0;
        } else if( a == RaiseAntenna ) {
            return bel.antenna_height_ < max_antenna_height_;
        } else if( a == LowerAntenna ) {
            return bel.antenna_height_ > 0;
        } else if( a == Sense ) {
            return true; // sensor can be activated at any time
        } else if( is_sample_action(a) ) {
            // rock must not be sampled/skipped, it must be known to be a good rock, and agent must be at rock's location
            int r = sampled_rock(a);
            assert((r >= 0) && (r < number_rocks_));
            assert((bel.beams_[r].value_ != beam_t::good) || (bel.skipped_.bit(r) == 0));
            return (bel.loc_ == loc_t(rock_locations_[r], xdim_, ydim_)) && (bel.beams_[r].value_ == beam_t::good) && (bel.sampled_.bit(r) == 0);
        } else if( is_skip_action(a) ) {
            // rock must not be sampled/skipped, it must be known to be a bad rock, and agent must be at rock's location
            int r = skipped_rock(a);
            assert((r >= 0) && (r < number_rocks_));
            assert((bel.beams_[r].value_ != beam_t::bad) || (bel.sampled_.bit(r) == 0));
            return (bel.loc_ == loc_t(rock_locations_[r], xdim_, ydim_)) && (bel.beams_[r].value_ == beam_t::bad) && (bel.skipped_.bit(r) == 0);
        } else {
            std::cout << Utils::error() << "unknown action a=" << a << std::endl;
            return false;
        }
    }
    virtual float min_absolute_cost() const { return 1; }
    virtual float max_absolute_cost() const { return 10; }
    virtual float cost(const belief_state_t &bel, Problem::action_t a) const {
        if( is_sample_action(a) || is_skip_action(a) )
            return -10;
        else
            return 1;
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
        if( a != Sense ) {
            outcomes.reserve(1);
            belief_state_t next_bel(bel);
            if( a == GoNorth ) {
                next_bel.loc_.move_north(xdim_, ydim_);
            } else if( a == GoEast ) {
                next_bel.loc_.move_east(xdim_, ydim_);
            } else if( a == GoSouth ) {
                next_bel.loc_.move_south(xdim_, ydim_);
            } else if( a == GoWest ) {
                next_bel.loc_.move_west(xdim_, ydim_);
            } else if( a == RaiseAntenna ) {
                next_bel.raise_antenna();
            } else if( a == LowerAntenna ) {
                next_bel.lower_antenna();
            } else if( is_sample_action(a) ) {
                int r = sampled_rock(a);
                next_bel.sample_rock(r);
            } else if( is_skip_action(a) ) {
                int r = skipped_rock(a);
                next_bel.skip_rock(r);
            } else {
                std::cout << Utils::error() << "unknown action a=" << a << std::endl;
                exit(-1);
            }
            outcomes.push_back(std::make_pair(next_bel, 1.0));
#ifdef DEBUG
            std::cout << "    " << outcomes.size() << ". next-bel=" << next_bel << std::endl;
#endif
        } else {
            outcomes.reserve(2);
            belief_state_t next_bel_0(bel);
            std::pair<float, float> p = bel.probability_sense(rock_locations_); // pair is (P(good), P(!good))
            if( p.first > 0 ) {
                belief_state_t next_bel(bel);
                next_bel.apply_sense(OBS_GOOD, rock_locations_);
                outcomes.push_back(std::make_pair(next_bel, p.first));
#ifdef DEBUG
                std::cout << "    " << outcomes.size() << ". next-bel=" << next_bel << std::endl;
#endif
            }
            if( p.second > 0 ) {
                belief_state_t next_bel(bel);
                next_bel.apply_sense(OBS_NOT_GOOD, rock_locations_);
                outcomes.push_back(std::make_pair(next_bel, p.second));
#ifdef DEBUG
                std::cout << "    " << outcomes.size() << ". next-bel=" << next_bel << std::endl;
#endif
            }
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
        assert(0); // CHECK
    }
    virtual POMDP::feature_t<belief_state_t> *get_feature(const belief_state_t &bel) const {
        POMDP::feature_t<belief_state_t> *feature = new feature_t(bel);
        //std::cout << feature << " GET" << std::endl;
        return feature;
    }
    virtual void remove_feature(const POMDP::feature_t<belief_state_t> *feature) const {
        //std::cout << feature << " DEL" << std::endl;
        delete feature;
    }

    virtual void apply_action(belief_state_t &bel_a, Problem::action_t a) const {
        if( a != Sense ) {
            if( a == GoNorth ) {
                bel_a.loc_.move_north(xdim_, ydim_);
            } else if( a == GoEast ) {
                bel_a.loc_.move_east(xdim_, ydim_);
            } else if( a == GoSouth ) {
                bel_a.loc_.move_south(xdim_, ydim_);
            } else if( a == GoWest ) {
                bel_a.loc_.move_west(xdim_, ydim_);
            } else if( a == RaiseAntenna ) {
                bel_a.raise_antenna();
            } else if( a == LowerAntenna ) {
                bel_a.lower_antenna();
            } else if( is_sample_action(a) ) {
                int r = sampled_rock(a);
                bel_a.sample_rock(r);
            } else if( is_skip_action(a) ) {
                int r = skipped_rock(a);
                bel_a.skip_rock(r);
            } else {
                std::cout << Utils::error() << "unknown action a=" << a << std::endl;
                exit(-1);
            }
        } else {
            /* Sense action has no effect on states, it only returns information */
        }
    }
    virtual void apply_obs(belief_state_t &bel_ao, Problem::action_t a, POMDP::observation_t obs) const {
        if( a == Sense )
            bel_ao.apply_sense(obs, rock_locations_);
    }
    virtual POMDP::observation_t sample_observation_using_hidden_state(const belief_state_t &bel, const belief_state_t &bel_a, Problem::action_t a) const {
        if( a == Sense ) {
            for( int r = 0; r < number_rocks_; ++r ) {
                if( (bel.hidden_.bit(r) == 1) && (bel.loc().euclidean_distance(loc_t(rock_locations_[r], xdim_, ydim_)) <= float(bel.antenna_height())) )
                    return OBS_GOOD;
            }
            return OBS_NOT_GOOD;
        } else {
            return OBS_NOT_GOOD; // fixed (dummy) observation
        }
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
        else if( a == RaiseAntenna )
            return std::string("raise-antenna()");
        else if( a == LowerAntenna )
            return std::string("lower-antenna()");
        else if( a == Sense )
            return std::string("sense()");
        else if( is_sample_action(a) )
            return std::string("sample(r=") + std::to_string(sampled_rock(a)) + std::string(")");
        else if( is_skip_action(a) )
            return std::string("skip(r=") + std::to_string(skipped_rock(a)) + std::string(")");
        else
            return std::string("unknown");
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

