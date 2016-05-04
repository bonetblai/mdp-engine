#include <iostream>
#include <iomanip>
#include <strings.h>

#include "../binary-search/bitmap.h"

#include <arc_consistency.h>

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

struct beam_t {
    const loc_t loc_;
    const int range_;
    std::map<int, int> rocks_;
    std::vector<int> values_;

    struct const_iterator {
        int i_;
        const std::vector<int> &values_;

        const_iterator(int i, const std::vector<int> &values) : i_(i), values_(values) { }

        bool operator==(const const_iterator &it) const {
            return (i_ == it.i_) && (&values_ == &it.values_);
        }
        bool operator!=(const const_iterator &it) const {
            return !(*this == it);
        }
        const const_iterator& operator++() {
            ++i_;
            return *this;
        }
        int operator*() const {
            return value();
        }

        int index() const {
            return i_;
        }
        int value() const {
            assert(i_ < int(values_.size()));
            return values_[i_];
        }
    }; // const_iterator

    beam_t(loc_t loc, int range) : loc_(loc), range_(range) { }
    beam_t(const beam_t &beam)
      : loc_(beam.loc_),
        range_(beam.range_),
        rocks_(beam.rocks_),
        values_(beam.values_) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "beam_t: copy ctor called" << std::endl;
#endif
    }
    beam_t(beam_t &&beam)
      : loc_(beam.loc_),
        range_(beam.range_),
        rocks_(std::move(beam.rocks_)),
        values_(std::move(beam.values_)) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "beam_t: move ctor called" << std::endl;
#endif
    }
    virtual ~beam_t() {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "beam_t: dtor called" << std::endl;
#endif
    }

    void push_rock(int r) {
        rocks_.insert(std::make_pair(r, rocks_.size()));
    }
    void set_initial_values() {
        values_.clear();
        values_.reserve(1 << rocks_.size());
        for( int i = 0; i < (1 << rocks_.size()); ++i )
            values_.push_back(i);
    }

    static int rock_value(int rock_index, int valuation) {
        return (valuation >> rock_index) & 0x1;
    }
    bool is_value_for_rock(int r, int value) const {
        assert(rocks_.find(r) != rocks_.end());
        int rock_index = rocks_.at(r);
        for( int i = 0; i < int(values_.size()); ++i ) {
            if( beam_t::rock_value(rock_index, values_[i]) != value )
                return false;
        }
        return true;
    }
    bool is_good_rock(int r) const {
        return is_value_for_rock(r, 1);
    }
    bool is_bad_rock(int r) const {
        return is_value_for_rock(r, 0);
    }

    bool empty() const {
        return values_.empty();
    }
    int size() const {
        return values_.size();
    }
    void clear() {
        values_.clear();
    }

    void erase_ordered_indices(const std::vector<int> &indices) {
        if( !indices.empty() ) {
            int indices_sz = indices.size();
            int k = indices[0];
            for( int i = 0, j = k; j < int(values_.size()); ) {
                while( (i < indices_sz) && (j == indices[i]) ) { ++j; ++i; }
                while( (j < int(values_.size())) && ((i >= indices_sz) || (j < indices[i])) ) {
                    values_[k++] = values_[j++];
                }
                assert((j == int(size())) || (i < indices_sz));
                assert((j == int(size())) || (j == indices[i]));
            }
            assert(k == int(size()) - indices_sz);
            while( k < int(size()) ) values_.pop_back();
        }
    }

    const beam_t& operator=(const beam_t &beam) {
        assert(loc_ == beam.loc_);
        rocks_ = beam.rocks_;
        values_ = beam.values_;
        return *this;
    }
    bool operator==(const beam_t &beam) const {
        return (loc_ == beam.loc_) && (rocks_ == beam.rocks_) && (values_ == beam.values_);
    }
    bool operator!=(const beam_t &beam) const {
        return !(*this == beam);
    }
    bool operator<(const beam_t &beam) const {
        return (loc_ < beam.loc_) ||
          ((loc_ == beam.loc_) && (rocks_ < beam.rocks_)) ||
          ((loc_ == beam.loc_) && (rocks_ == beam.rocks_) && (values_ < beam.values_));
    }

    unsigned hash() const {
        return Utils::jenkins_one_at_a_time_hash(values_);
    }

    virtual const_iterator begin() const {
        return const_iterator(0, values_);
    }
    virtual const_iterator end() const {
        return const_iterator(values_.size(), values_);
    }

    void print(std::ostream &os) const {
        os << "[loc=" << loc_ << ",range=" << range_ << ",rocks={";
        for( std::map<int, int>::const_iterator it = rocks_.begin(); it != rocks_.end(); ++it )
            os << it->first << ",";
        os << "},values={";
        for( int i = 0; i < int(values_.size()); ++i ) {
            os << values_[i];
            if( 1 + i < int(values_.size()) ) os << ",";
        }
        os << "}]" << std::flush;
    }
};

inline std::ostream& operator<<(std::ostream &os, const beam_t &beam) {
    beam.print(os);
    return os;
}

class pomdp_t; // forward reference

class arc_consistency_t : public CSP::arc_consistency_t<beam_t> {
  protected:
    mutable const std::vector<std::pair<int, std::pair<int, int> > > *common_rocks_;
    mutable std::vector<int> rock_values_;

    static const pomdp_t *pomdp_;

  public:
    arc_consistency_t(const CSP::constraint_digraph_t &digraph)
      : CSP::arc_consistency_t<beam_t>(digraph), common_rocks_(0) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "arc_consistency_t: ctor called" << std::endl;
#endif
    }
    arc_consistency_t(const arc_consistency_t &ac)
      : CSP::arc_consistency_t<beam_t>(ac), common_rocks_(0) {
        for( int i = 0; i < nvars_; ++i )
            set_domain(i, new beam_t(*ac.domain(i)));
#ifdef DEBUG_CTOR_DTOR
        std::cout << "arc_consistency_t: copy ctor called" << std::endl;
#endif
    }
    arc_consistency_t(arc_consistency_t &&ac)
      : CSP::arc_consistency_t<beam_t>(std::move(ac)) {
        ac.clear();
#ifdef DEBUG_CTOR_DTOR
        std::cout << "arc_consistency_t: move ctor called" << std::endl;
#endif
    }
    virtual ~arc_consistency_t() {
        delete_domains_and_clear();
#ifdef DEBUG_CTOR_DTOR
        std::cout << "arc_consistency_t: dtor called" << std::endl;
#endif
    }

    static void set_static_members(const pomdp_t *pomdp) {
        pomdp_ = pomdp;
    }

    const arc_consistency_t& operator=(const arc_consistency_t &ac) {
        delete_domains_and_clear();
        static_cast<CSP::arc_consistency_t<beam_t>&>(*this) = ac;
        for( int i = 0; i < nvars(); ++i )
            set_domain(i, new beam_t(*ac.domain(i)));
        return *this;
    }
    bool operator==(const arc_consistency_t &ac) const {
        if( nvars() != ac.nvars() ) return false;
        for( int i = 0; i < nvars(); ++i ) {
            if( (domain(i) == 0) && (ac.domain(i) == 0) )
                continue;
            if( (domain(i) == 0) || (ac.domain(i) == 0) )
                return false;
            if( *domain(i) != *ac.domain(i) )
                return false;
        }
        return true;
    }

    virtual void arc_reduce_preprocessing_0(int var_x, int var_y) const; // defined below
    virtual void arc_reduce_preprocessing_1(int var_x, int val_x) const {
        assert(common_rocks_ != 0);
        rock_values_.clear();
        for( int i = 0; i < int(common_rocks_->size()); ++i ) {
            const std::pair<int, int> &p = (*common_rocks_)[i].second;
            rock_values_.push_back(beam_t::rock_value(p.first, val_x));
        }
    }
    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const {
        //std::cout << "consistent(var_x=" << var_x << ",var_y=" << var_y << ",val_x=" << val_x << ",val_y=" << val_y << ")=" << std::flush;
        assert(common_rocks_ != 0);
        for( int i = 0; i < int(common_rocks_->size()); ++i ) {
            const std::pair<int, int> &p = (*common_rocks_)[i].second;
            if( rock_values_[i] != beam_t::rock_value(p.second, val_y) ) {
                //std::cout << "false" << std::endl;
                return false;
            }
        }
        //std::cout << "true" << std::endl;
        return true;
    }
    virtual void arc_reduce_postprocessing(int var_x, int var_y) const {
        common_rocks_ = 0;
    }
};

// known vars: loc, antenna-height, sampled rocks, skipped rocks
// unknown vars: rock status
class belief_state_t {
  protected:
    loc_t loc_;
    int antenna_height_;
    Bitmap::bitmap_t sampled_;
    Bitmap::bitmap_t skipped_;
    Bitmap::bitmap_t hidden_;

    arc_consistency_t csp_;

    static const pomdp_t *pomdp_;

  public:
    belief_state_t(int loc = 0, int antenna_height = 0)
      : loc_(loc, xdim(), ydim()),
        antenna_height_(antenna_height),
        sampled_(0),
        skipped_(0),
        csp_(constraint_digraph()) {
    }
    belief_state_t(int loc, int antenna_height, const Bitmap::bitmap_t &hidden)
      : loc_(loc, xdim(), ydim()),
        antenna_height_(antenna_height),
        sampled_(0),
        skipped_(0),
        hidden_(hidden),
        csp_(constraint_digraph()) {
        assert(csp_.nvars() == xdim() * ydim());
        for( int l = 0; l < xdim() * ydim(); ++l ) {
            loc_t bloc(l, xdim(), ydim());
            int d = max_antenna_height();
            beam_t *beam = new beam_t(bloc, d);
            csp_.set_domain(l, beam);
            for( int r = 0; r < number_rocks(); ++r ) {
                if( bloc.euclidean_distance(rock_location(r)) <= d )
                    beam->push_rock(r);
            }
            beam->set_initial_values();
#ifdef DEBUG
            std::cout << "beam(loc=" << bloc << ",d=" << d << ")=" << *beam << std::endl;
#endif
        }
    }
    belief_state_t(const belief_state_t &bel)
      : loc_(bel.loc_),
        antenna_height_(bel.antenna_height_),
        sampled_(bel.sampled_),
        skipped_(bel.skipped_),
        hidden_(bel.hidden_),
        csp_(bel.csp_) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: copy ctor called" << std::endl;
#endif
    }
    belief_state_t(belief_state_t &&bel)
      : loc_(bel.loc_),
        antenna_height_(bel.antenna_height_),
        sampled_(std::move(bel.sampled_)),
        skipped_(std::move(bel.skipped_)),
        hidden_(std::move(bel.hidden_)),
        csp_(std::move(bel.csp_)) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: move ctor called" << std::endl;
#endif
    }
    virtual ~belief_state_t() {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: dtor called" << std::endl;
#endif
    }

    static void set_static_members(const pomdp_t *pomdp) {
        pomdp_ = pomdp;
        std::cout << "pomdp: xdim=" << xdim() << ", ydim=" << ydim() << ", max-antenna-height=" << max_antenna_height() << std::endl;
    }

    static int xdim();                         // defined below
    static int ydim();                         // defined below
    static int number_variables();             // defined below
    static int number_rocks();                 // defined below
    static int max_antenna_height();           // defined below
    static const loc_t& rock_location(int r);  // defined below
    static const CSP::constraint_digraph_t& constraint_digraph();  // defined below

    size_t hash() const {
        std::cout << "warning: hash value is zero!" << std::endl;
        return 0; // CHECK
        //return Utils::jenkins_one_at_a_time_hash(beams_);
    }

    const belief_state_t& operator=(const belief_state_t &bel) {
        loc_ = bel.loc_;
        antenna_height_ = bel.antenna_height_;
        sampled_ = bel.sampled_;
        skipped_ = bel.skipped_;
        hidden_ = bel.hidden_;
        csp_ = bel.csp_;
        return *this;
    }
    bool operator==(const belief_state_t &bel) const {
        return (loc_ == bel.loc_) &&
          (antenna_height_ == bel.antenna_height_) &&
          (sampled_ == bel.sampled_) &&
          (skipped_ == bel.skipped_) &&
          (hidden_ == bel.hidden_) &&
          (csp_ == bel.csp_);
    }
    bool operator!=(const belief_state_t &bel) const {
        return !(*this == bel);
    }
    bool operator<(const belief_state_t &bel) const {
        assert(0); // CHECK
#if 0
        return (loc_ < bel.loc_) ||
          ((loc_ == bel.loc_) && (antenna_height_ < bel.antenna_height_)) ||
          ((loc_ == bel.loc_) && (antenna_height_ == bel.antenna_height_) && (beams_ < bel.beams_)) ||
          ((loc_ == bel.loc_) && (antenna_height_ == bel.antenna_height_) && (beams_ == bel.beams_) && (sampled_ < bel.sampled_)) ||
          ((loc_ == bel.loc_) && (antenna_height_ == bel.antenna_height_) && (beams_ == bel.beams_) && (sampled_ == bel.sampled_) && (skipped_ < bel.skipped_)) ||
          ((loc_ == bel.loc_) && (antenna_height_ == bel.antenna_height_) && (beams_ == bel.beams_) && (sampled_ == bel.sampled_) && (skipped_ == bel.skipped_) && (hidden_ < bel.hidden_));
#endif
    }

    int value(int vid) const {
        assert((vid >= 0) && (vid < number_variables()));
        if( vid == 0 ) {
            return loc_.as_integer(xdim(), ydim());
        } else if( vid == 1 ) {
            return antenna_height_;
        } else if( vid - 2 < number_rocks() ) {
            int r = vid - 2;
            return sampled_.bit(r);
        } else if( vid - 2 - number_rocks() < number_rocks() ) {
            int r = vid - 2 - number_rocks();
            return skipped_.bit(r);
        } else {
            assert(0); // CHECK
            int index = vid - 2 - 2 * number_rocks();
            const beam_t &beam = get_beam(index);
            assert(!beam.values_.empty());
            return beam.values_.size() > 1 ? -1 : beam.values_[0];
        }
    }
    void fill_values_for_variable(int vid, std::vector<float> &probabilities) const {
        assert((vid >= 0) && (vid < number_variables()));
        if( vid == 0 ) {
            probabilities = std::vector<float>(xdim() * ydim(), 0);
            int value = loc_.as_integer(xdim(), ydim());
            probabilities[value] = 1;
        } else if( vid == 1 ) {
            probabilities = std::vector<float>(1 + max_antenna_height(), 0);
            probabilities[antenna_height_] = 1;
        } else if( vid - 2 < number_rocks() ) {
            probabilities = std::vector<float>(2, 0);
            int r = vid - 2;
            probabilities[sampled_.bit(r)] = 1;
        } else if( vid - 2 - number_rocks() < number_rocks() ) {
            probabilities = std::vector<float>(2, 0);
            int r = vid - 2 - number_rocks();
            probabilities[skipped_.bit(r)] = 1;
        } else {
            probabilities = std::vector<float>(2, 0);
            int r = vid - 2 - 2 * number_rocks();
            assert((r >= 0) && (r < number_rocks()));
            int beam_index = rock_location(r).as_integer(xdim(), ydim());;
            const beam_t &beam = get_beam(beam_index);
            assert(beam.loc_ == rock_location(r));
            int rock_index = beam.rocks_.at(r);
            for( int i = 0; i < beam.values_.size(); ++i ) {
                int rvalue = beam_t::rock_value(rock_index, beam.values_[i]);
                assert((rvalue == 0) || (rvalue == 1));
                ++probabilities[rvalue];
            }
            probabilities[0] /= float(beam.values_.size());
            probabilities[1] /= float(beam.values_.size());
        }
    }
    void fill_values_for_variable(int vid, std::vector<std::pair<int, float> > &values) const {
        std::vector<float> probabilities;
        fill_values_for_variable(vid, probabilities);
        values.reserve(probabilities.size());
        for( int i = 0; i < int(probabilities.size()); ++i ) {
            if( probabilities[i] != 0 )
                values.push_back(std::make_pair(i, probabilities[i]));
        }
    }

    const loc_t& loc() const { return loc_; }
    int antenna_height() const { return antenna_height_; }

    beam_t& get_beam(int bid) {
        assert((bid >= 0) && (bid < csp_.nvars()));
        assert(csp_.domain(bid) != 0);
        return *csp_.domain(bid);
    }
    const beam_t& get_beam(int bid) const {
        assert((bid >= 0) && (bid < csp_.nvars()));
        assert(csp_.domain(bid) != 0);
        return *csp_.domain(bid);
    }

    void raise_antenna() {
        assert(antenna_height_ < max_antenna_height());
        ++antenna_height_;
    }
    void lower_antenna() {
        assert(antenna_height_ > 0);
        --antenna_height_;
    }
    void sample_rock(int r) {
        assert((r >= 0) && (r < number_rocks()));
        assert((sampled_.bit(r) == 0) && (skipped_.bit(r) == 0));
        sampled_.set_bit(r, 1);
    }
    void skip_rock(int r) {
        assert((r >= 0) && (r < number_rocks()));
        assert((sampled_.bit(r) == 0) && (skipped_.bit(r) == 0));
        skipped_.set_bit(r, 1);
    }

    bool apply_sense(int obs, const std::vector<loc_t> &rock_locations) {
        //std::cout << "apply_sense: obs=" << obs << ", bel="; print(std::cout); std::cout << std::endl;
        int index = loc_.as_integer(xdim(), ydim());
        beam_t &beam = get_beam(index);
        assert(loc_ == beam.loc_);

        // if beam has no rocks, obs must be OBS_NOT_GOOD and there is nothing to do
        if( beam.rocks_.empty() ) {
            //std::cout << "apply_sense(EXIT0): nothing done!" << std::endl;
            assert(obs == OBS_NOT_GOOD);
            return false;
        }

        // calculate rocks within sensor range
        std::vector<int> indices_for_rocks_within_sensor_range;
        for( std::map<int, int>::const_iterator it = beam.rocks_.begin(); it != beam.rocks_.end(); ++it ) {
            if( loc_.euclidean_distance(rock_location(it->first)) <= antenna_height_ )
                indices_for_rocks_within_sensor_range.push_back(it->second);
        }

        // remove all valuations not compatible with observation and restore consistency across beams using AC3
        std::vector<int> indices_to_erase;
        for( int i = 0; i < int(beam.values_.size()); ++i ) {
            int value = beam.values_[i];
            if( obs == OBS_GOOD ) {
                bool all_rocks_within_sensor_range_are_bad = true;
                for( int j = 0; j < int(indices_for_rocks_within_sensor_range.size()); ++j ) {
                    int rock_index = indices_for_rocks_within_sensor_range[j];
                    if( beam_t::rock_value(rock_index, value) == 1 ) {
                        all_rocks_within_sensor_range_are_bad = false;
                        break;
                    }
                }
                if( all_rocks_within_sensor_range_are_bad )
                    indices_to_erase.push_back(i);
            } else {
                for( int j = 0; j < int(indices_for_rocks_within_sensor_range.size()); ++j ) {
                    int rock_index = indices_for_rocks_within_sensor_range[j];
                    if( beam_t::rock_value(rock_index, value) == 1 ) {
                        indices_to_erase.push_back(i);
                        break;
                    }
                }
            }
        }

        // erase indices
        bool something_removed = !indices_to_erase.empty();
        beam.erase_ordered_indices(indices_to_erase);

        // restore consistency across beams
        if( something_removed ) restore_consistency(index);

        //std::cout << "apply_sense(EXIT): obs=" << obs << ", bel="; print(std::cout); std::cout << std::endl;
        return something_removed;
    }

    std::pair<float, float> probability_sense(const std::vector<loc_t> &rock_locations) const { // pair is (P(good), P(!good))
        int index = loc_.as_integer(xdim(), ydim());
        const beam_t &beam = get_beam(index);
        assert(loc_ == beam.loc_);

        float p_good = 0;
        if( beam.values_.empty() ) {
            p_good = 0;
        } else {
            // calculate rocks within sensor range
            std::vector<int> indices_for_rocks_within_sensor_range;
            for( std::map<int, int>::const_iterator it = beam.rocks_.begin(); it != beam.rocks_.end(); ++it ) {
                if( loc_.euclidean_distance(rock_location(it->first)) <= antenna_height_ )
                    indices_for_rocks_within_sensor_range.push_back(it->second);
            }

            // calculate # valuations that contain rocks within sensor range that are good
            int number_valuations_with_good_rocks_within_sensor_range = 0;
            for( int i = 0; i < beam.values_.size(); ++i ) {
                int value = beam.values_[i];
                for( int j = 0; j < int(indices_for_rocks_within_sensor_range.size()); ++j ) {
                    int rock_index = indices_for_rocks_within_sensor_range[j];
                    if( beam_t::rock_value(rock_index, value) == 1 )
                        ++number_valuations_with_good_rocks_within_sensor_range;
                }
            }

            // P(good) = # valuations that contain good rocks within sensor range / total # valuations
            p_good = float(number_valuations_with_good_rocks_within_sensor_range) / float(beam.values_.size());
        } 
        return std::make_pair(p_good, 1 - p_good);
    }
    bool restore_consistency(int seed_var) {
        // restore consistency using AC3
        //std::cout << "=====================================================" << std::endl;
        //std::cout << "BELIEF BEFORE AC3: seed_loc=" << loc_t(seed_var, xdim(), ydim()) << std::endl;
        //print(std::cout);
        //std::cout << std::endl;

        std::vector<int> revised_vars;
        csp_.add_to_worklist(seed_var);
        bool something_removed = csp_.ac3(revised_vars); // CHECK: inverse_check is off

        //std::cout << "=====================================================" << std::endl;
        //std::cout << "BELIEF AFTER AC3: seed_loc=" << loc_t(seed_var, xdim(), ydim()) << std::endl;
        //print(std::cout);
        //std::cout << std::endl;
        //std::cout << "=====================================================" << std::endl;

        return something_removed;
    }

    void print(std::ostream &os) const {
        os << "[loc=" << loc_
           << ",height=" << antenna_height_
           << ",beams=[";
        for( int i = 0; i < csp_.nvars(); ++i ) {
            const beam_t &beam = get_beam(i);
            os << beam;
            if( 1 + i < int(csp_.nvars()) ) os << ",";
        }
        os << "],sampled=" << sampled_ << ",skipped=" << skipped_ << ",hidden=" << hidden_ << "]" << std::flush;
    }

    friend class pomdp_t;
};

inline std::ostream& operator<<(std::ostream &os, const belief_state_t &bel) {
    bel.print(os);
    return os;
}

class pomdp_t : public POMDP::pomdp_t<belief_state_t> {
  protected:
    int xdim_;
    int ydim_;
    int number_rocks_;
    int max_antenna_height_;
    bool sample_rock_locations_with_init_;

    // There is 1 binary (hidden) variable for status of each rock. There are known
    // variables for location of agent and height of antenna.
    int number_variables_; // 1 for each rock, 1 for location, 1 for antenna
    int number_actions_;   // 4 for movements, 2 for antenna, 1 for sense, nrocks for sample, nrocks for skip
    int number_beams_;     // 1 for each rock, each containing gthe status of the rock

    // The following are set when creating a new initial belief state.
    // At that time, rocks are placed in the grid, status of rocks are
    // determined, and the constraint graph and beams are constructed.
    // These elements are only modified in init() that creates initial
    // belief state
    mutable belief_state_t *init_;
    mutable std::vector<loc_t> rock_locations_;
    mutable CSP::constraint_digraph_t constraint_digraph_;
    mutable std::vector<std::vector<int> > beams_for_rocks_;
    mutable std::vector<std::vector<std::pair<int, std::pair<int, int> > > > common_rocks_;

    enum { GoNorth, GoEast, GoSouth, GoWest, RaiseAntenna, LowerAntenna, Sense };

  public:
    pomdp_t(int xdim, int ydim, int number_rocks, int max_antenna_height, bool sample_rock_locations_with_init)
      : POMDP::pomdp_t<belief_state_t>(DISCOUNT),
        xdim_(xdim),
        ydim_(ydim),
        number_rocks_(number_rocks),
        max_antenna_height_(max_antenna_height),
        sample_rock_locations_with_init_(sample_rock_locations_with_init),
        init_(0) {
        number_variables_ = 2 + 3 * number_rocks_; // vars: loc (1), antenna (1), sampled (#rocks), skipped (#rocks), rock status (#rock)
        number_actions_ = 7 + 2 * number_rocks_;
        number_beams_ = number_rocks_;
        if( !sample_rock_locations_with_init_ )
            sample_rock_locations();
    }
    virtual ~pomdp_t() {
        delete init_;
    }

    void set_constraint_digraph() const {
        // there is one beam for each cell which is associated with maximum antenna height
        constraint_digraph_.create_empty_graph(xdim_ * ydim_);

        // constraint graph must have edge (loc1) -- (loc2) iff there is rock r
        // such that dist(loc1,loc(r)) <= d and dist(loc2,loc(r)) <= d where d
        // is maximum antenna height

        // for each rock r, calculate beams (loc,d) that contain r
        beams_for_rocks_ = std::vector<std::vector<int> >(number_rocks_);
        for( int r = 0; r < number_rocks_; ++r ) {
            const loc_t &rloc = rock_locations_[r];
            for( int l = 0; l < xdim_ * ydim_; ++l ) {
                loc_t bloc(l, xdim_, ydim_);
                int d = max_antenna_height_;
                if( bloc.euclidean_distance(rloc) <= d )
                    beams_for_rocks_[r].push_back(l);
            }
        }

        // for each rock r, and pair of beam locs l1 and l2 where r belongs,
        // add edge l1 -- l2 if edge has not been created yet
        std::set<std::pair<int, int> > edges;
        for( int r = 0; r < number_rocks_; ++r ) {
            const std::vector<int> &beams = beams_for_rocks_[r];
            for( int i = 0; i < int(beams.size()); ++i ) {
                int l1 = beams[i];
                for( int j = i + 1; j < int(beams.size()); ++j ) {
                    int l2 = beams[j];
                    assert(l1 < l2);
                    std::pair<int, int> edge(l1, l1);
                    if( edges.find(edge) == edges.end() ) {
                        constraint_digraph_.add_edge(l1, l2);
                        constraint_digraph_.add_edge(l2, l1);
                        edges.insert(std::make_pair(l1, l2));
#ifdef DEBUG
                        std::cout << "constraint digraph: add edge "
                                  << loc_t(l1, xdim_, ydim_) << " <--> " << loc_t(l2, xdim_, ydim_)
                                  << " because of rock r=" << r
                                  << std::endl;
#endif
                    }
                }
            }
        }
    }

    void sample_rock_locations() const {
        std::vector<int> locations(xdim_ * ydim_);
        for( int i = 0; i < xdim_ * ydim_; ++i )
            locations[i] = i;

        rock_locations_.reserve(number_rocks_);
        for( int r = 0; r < number_rocks_; ++r ) {
            int i = Random::random(0, locations.size());
            int loc = locations[i];
            locations[i] = locations.back();
            locations.pop_back();
            rock_locations_.push_back(loc_t(loc, xdim_, ydim_));
#if 1//def DEBUG
            std::cout << "rock: r=" << r << " --> loc=" << loc_t(loc, xdim_, ydim_) << std::endl;
#endif
        }
    }
 
    void populate_common_rocks_between_beams(const belief_state_t *belief) const {
        assert(belief != 0);
        common_rocks_ = std::vector<std::vector<std::pair<int, std::pair<int, int> > > >(xdim_ * ydim_ * xdim_ * ydim_);
        for( int r = 0; r < number_rocks_; ++r ) {
            const std::vector<int> &beams = beams_for_rocks_[r];
            for( int i = 0; i < int(beams.size()); ++i ) {
                int l1 = beams[i];
                const beam_t &beam1 = belief->get_beam(l1);
                int index1 = beam1.rocks_.at(r);
                for( int j = i + 1; j < int(beams.size()); ++j ) {
                    int l2 = beams[j];
                    assert(l1 < l2);
                    const beam_t &beam2 = belief->get_beam(l2);
                    int index2 = beam2.rocks_.at(r);
                    common_rocks_[l1 * xdim_ * ydim_ + l2].push_back(std::make_pair(r, std::make_pair(index1, index2)));
                    common_rocks_[l2 * xdim_ * ydim_ + l1].push_back(std::make_pair(r, std::make_pair(index2, index1)));
#ifdef DEBUG
                    std::cout << "rock " << r << " is common to beams ["
                              << l1 << "=" << loc_t(l1, xdim_, ydim_) << "," << l2 << "=" << loc_t(l2, xdim_, ydim_) << "]"
                              << "=(" << index1 << "," << index2 << ")"
                              << std::endl;
#endif
                }
            }
        }
    }

    const std::vector<std::pair<int, std::pair<int, int> > >& common_rocks(int beam_index1, int beam_index2) const {
        assert((beam_index1 >= 0) && (beam_index2 >= 0));
        assert((beam_index1 < xdim_ * ydim_) && (beam_index2 < xdim_ * ydim_));
        return common_rocks_[beam_index1 * xdim_ * ydim_ + beam_index2];
    }

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

        // we must create constraint graph before any belief state is created
        set_constraint_digraph();
        init_ = new belief_state_t(0, 0, rocks);

        // after creating initial belief, where all rocks are placed, calculate
        // for each pair of beams the common rocks to them, together with their
        // indices (this is used when filtering)
        populate_common_rocks_between_beams(init_);
        return *init_;
    }
    virtual bool terminal(const belief_state_t &bel) const {
        return bel.sampled_.popcount() + bel.skipped_.popcount() == number_rocks_;
    }
    virtual bool dead_end(const belief_state_t &bel) const {
        return false;
    }
    virtual bool applicable(const belief_state_t &bel, ::Problem::action_t a) const {
        if( a == GoNorth ) {
            return (bel.antenna_height_ == 0) && (1 + bel.loc_.r_ < ydim_);
        } else if( a == GoEast ) {
            return (bel.antenna_height_ == 0) && (1 + bel.loc_.c_ < xdim_);
        } else if( a == GoSouth ) {
            return (bel.antenna_height_ == 0) && (bel.loc_.r_ > 0);
        } else if( a == GoWest ) {
            return (bel.antenna_height_ == 0) && (bel.loc_.c_ > 0);
        } else if( a == RaiseAntenna ) {
            return bel.antenna_height_ < max_antenna_height_;
        } else if( a == LowerAntenna ) {
            return bel.antenna_height_ > 0;
        } else if( a == Sense ) {
            return true; // sensor can be activated at any time
        } else if( is_sample_action(a) ) {
            // rock must not be sampled/skipped, it must be known to be good, and agent must be at rock's location
            int r = sampled_rock(a);
            assert((r >= 0) && (r < number_rocks_));
            int vid = rock_locations_[r].as_integer(xdim_, ydim_);
            const beam_t &beam = *bel.csp_.domain(vid);
            return (bel.loc_ == rock_locations_[r]) && (bel.sampled_.bit(r) == 0) && (bel.skipped_.bit(r) == 0) && beam.is_good_rock(r);
        } else if( is_skip_action(a) ) {
            // rock must not be sampled/skipped, and it must be known to be bad
            int r = skipped_rock(a);
            assert((r >= 0) && (r < number_rocks_));
            int vid = rock_locations_[r].as_integer(xdim_, ydim_);
            const beam_t &beam = *bel.csp_.domain(vid);
            return (bel.sampled_.bit(r) == 0) && (bel.skipped_.bit(r) == 0) && beam.is_bad_rock(r);
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
            outcomes.push_back(std::make_pair(belief_state_t(bel), 1.0));
            if( a == GoNorth ) {
                outcomes.back().first.loc_.move_north(xdim_, ydim_);
            } else if( a == GoEast ) {
                outcomes.back().first.loc_.move_east(xdim_, ydim_);
            } else if( a == GoSouth ) {
                outcomes.back().first.loc_.move_south(xdim_, ydim_);
            } else if( a == GoWest ) {
                outcomes.back().first.loc_.move_west(xdim_, ydim_);
            } else if( a == RaiseAntenna ) {
                outcomes.back().first.raise_antenna();
            } else if( a == LowerAntenna ) {
                outcomes.back().first.lower_antenna();
            } else if( is_sample_action(a) ) {
                int r = sampled_rock(a);
                assert((r >= 0) && (r < number_rocks_));
                outcomes.back().first.sample_rock(r);
            } else if( is_skip_action(a) ) {
                int r = skipped_rock(a);
                assert((r >= 0) && (r < number_rocks_));
                outcomes.back().first.skip_rock(r);
            } else {
                std::cout << Utils::error() << "unknown action a=" << a << std::endl;
                exit(-1);
            }
#ifdef DEBUG
            std::cout << "    " << outcomes.size() << ". next-bel=" << outcomes.back().first << std::endl;
#endif
        } else {
            outcomes.reserve(2);
            //std::cout << "PROBABILITY-SENSE.0: loc=" << bel.loc_ << ", antenna-height=" << bel.antenna_height_ << ", a=" << a << std::endl;
            std::pair<float, float> p = bel.probability_sense(rock_locations_); // pair is (P(good), P(!good))
            //std::cout << "PROBABILITY-SENSE.1: p: " << p.first << " " << p.second << std::endl;
            if( p.first > 0 ) {
                outcomes.push_back(std::make_pair(belief_state_t(bel), p.first));
                outcomes.back().first.apply_sense(OBS_GOOD, rock_locations_);
#ifdef DEBUG
                std::cout << "    " << outcomes.size() << ". next-bel=" << outcomes.back().first << std::endl;
#endif
            }
            if( p.second > 0 ) {
                outcomes.push_back(std::make_pair(belief_state_t(bel), p.second));
                outcomes.back().first.apply_sense(OBS_NOT_GOOD, rock_locations_);
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
        return 2 + 2 * number_rocks_;
    }

    virtual bool determined(int vid) const {
        return vid < 2; // CHECK: there are more determined variables
    }
    virtual int domain_size(int vid) const {
        if( vid == 0 ) {
            return xdim_ * ydim_;
        } else if( vid == 1 ) {
            return 1 + max_antenna_height_;
        } else {
            return 2;
        }
    }
    virtual int value(const belief_state_t &belief_state, int vid) const {
        assert(0); // CHECK
    }
    virtual void fill_values_for_variable(const belief_state_t &belief, int vid, std::vector<float> &probabilities) const {
        belief.fill_values_for_variable(vid, probabilities);
    }
    virtual void fill_values_for_variable(const belief_state_t &belief, int vid, std::vector<std::pair<int, float> > &values) const {
        belief.fill_values_for_variable(vid, values);
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
        POMDP::observation_t obs = OBS_NOT_GOOD; // default sensing
        if( a == Sense ) {
            for( int r = 0; r < number_rocks_; ++r ) {
                if( (bel.hidden_.bit(r) == 1) && (bel.loc().euclidean_distance(rock_locations_[r]) <= float(bel.antenna_height())) ) {
                    obs = OBS_GOOD;
                    break;
                }
            }
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
        assert(0); // CHECK
    }

    friend class belief_state_t;
};

inline std::ostream& operator<<(std::ostream &os, const pomdp_t &p) {
    p.print(os);
    return os;
}

inline void arc_consistency_t::arc_reduce_preprocessing_0(int var_x, int var_y) const {
    common_rocks_ = &pomdp_->common_rocks(var_x, var_y);
}

inline int belief_state_t::xdim() {
    assert(pomdp_ != 0);
    return pomdp_->xdim_;
}

inline int belief_state_t::ydim() {
    assert(pomdp_ != 0);
    return pomdp_->ydim_;
}

inline int belief_state_t::number_variables() {
    assert(pomdp_ != 0);
    return pomdp_->number_variables();
}

inline int belief_state_t::number_rocks() {
    assert(pomdp_ != 0);
    return pomdp_->number_rocks_;
}

inline int belief_state_t::max_antenna_height() {
    assert(pomdp_ != 0);
    return pomdp_->max_antenna_height_;
}

inline const loc_t& belief_state_t::rock_location(int r) {
    assert(pomdp_ != 0);
    assert(r < int(pomdp_->rock_locations_.size()));
    return pomdp_->rock_locations_[r];
}

inline const CSP::constraint_digraph_t& belief_state_t::constraint_digraph() {
    assert(pomdp_ != 0);
    return pomdp_->constraint_digraph_;
}

#undef DEBUG

