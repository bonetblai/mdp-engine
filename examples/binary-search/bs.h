#include <iostream>
#include <iomanip>
#include <strings.h>

#define DISCOUNT 1

//#define DEBUG

struct beam_t {
    unsigned bitmap_;

    struct const_iterator {
        unsigned bitmap_;
        int pos_;
        int size_;
        float p_;

        enum { Begin, End }; // iterator type

        const_iterator(unsigned bitmap, int type, int size)
          : bitmap_(bitmap), size_(size), p_(1.0 / float(__builtin_popcount(bitmap_))) {
            if( type == Begin ) {
                pos_ = -1;
                ++(*this);
            } else {
                pos_ = size_;
            }
        }

        bool operator==(const const_iterator &it) const {
            return (bitmap_ == it.bitmap_) && (pos_ == it.pos_) && (size_ == it.size_);
        }
        bool operator!=(const const_iterator &it) const {
            return (bitmap_ != it.bitmap_) || (pos_ != it.pos_) || (size_ != it.size_);
        }
        const const_iterator& operator++() {
            if( pos_ < size_ ) {
                for( ++pos_; (pos_ < size_) && (((bitmap_ >> pos_) & 1) == 0); ++pos_ );
            }
            return *this;
        }

        int value() const { return pos_; }
        float prob() const { return p_; }
    }; // const_iterator

    static int dim_;

    beam_t(unsigned bitmap) : bitmap_(bitmap) { }
    virtual ~beam_t() { }

    static void set_dimension(int dim) {
        dim_ = dim;
    }

    const beam_t& operator=(const beam_t &beam) {
        bitmap_ = beam.bitmap_;
        return *this;
    }
    bool operator==(const beam_t &beam) const {
        return bitmap_ == beam.bitmap_;
    }
    bool operator!=(const beam_t &beam) const {
        return bitmap_ != beam.bitmap_;
    }
    bool operator<(const beam_t &beam) const {
        return bitmap_ < beam.bitmap_;
    }

    int cardinality() const {
        return __builtin_popcount(bitmap_);
    }

    virtual const_iterator begin() const {
        return const_iterator(bitmap_, const_iterator::Begin, dim_);
    }
    virtual const_iterator end() const {
        return const_iterator(bitmap_, const_iterator::End, dim_);
    }

    void print(std::ostream &os) const {
        Utils::print_bits(os, bitmap_, beam_t::dim_);
    }
};

inline std::ostream& operator<<(std::ostream &os, const beam_t &beam) {
    beam.print(os);
    return os;
}

class belief_state_t {
  protected:
    beam_t beam_;
    int hidden_;

    static int dim_;
    static unsigned bitmap_mask_;
    static std::vector<unsigned> action_masks_;

  public:
    belief_state_t(int bitmap = unsigned(-1), int hidden = 0) : beam_(bitmap & bitmap_mask_), hidden_(hidden) { }
    belief_state_t(const belief_state_t &bel) : beam_(bel.beam_), hidden_(bel.hidden_) { }
    ~belief_state_t() { }

    static void set_bitmap_mask(int dim) {
        assert(dim <= 8 * sizeof(unsigned));
        dim_ = dim;
        bitmap_mask_ = unsigned(-1);
        bitmap_mask_ = bitmap_mask_ << (8 * sizeof(unsigned) - dim_);
        bitmap_mask_ = bitmap_mask_ >> (8 * sizeof(unsigned) - dim_);

        action_masks_.reserve(2 * (1 + dim_));
        for( int i = 0; i <= dim_; ++i ) {
            unsigned base = unsigned(-1);
            unsigned lower = i == 0 ? 0 : (base << (8 * sizeof(unsigned) - i)) >> (8 * sizeof(unsigned) - i);
            action_masks_.push_back(lower);
            unsigned upper = (base >> i) << i;
            action_masks_.push_back(upper);
#ifdef DEBUG
            std::cout << "action_mask[a=" << i << ",lower]="; Utils::print_bits(std::cout, lower, dim_); std::cout << std::endl;
            std::cout << "action_mask[a=" << i << ",upper]="; Utils::print_bits(std::cout, upper, dim_); std::cout << std::endl;
#endif
        }
    }

    size_t hash() const {
        return beam_.bitmap_;
    }

    belief_state_t apply(Problem::action_t a, int side) const {
        return belief_state_t(beam_.bitmap_ & action_masks_[2 * a + side], hidden_);
    }

    const belief_state_t& operator=( const belief_state_t &bel) {
        beam_ = bel.beam_;
        hidden_ = bel.hidden_;
        return *this;
    }
    bool operator==(const belief_state_t &bel) const {
        return (beam_ == bel.beam_) && (hidden_ == bel.hidden_);
    }
    bool operator!=(const belief_state_t &bel) const {
        return (beam_ != bel.beam_) || (hidden_ != bel.hidden_);
    }
    bool operator<(const belief_state_t &bel) const {
        return (beam_ < bel.beam_) || ((beam_ == bel.beam_) && (hidden_ < bel.hidden_));
    }

    const beam_t& beam(int bid) const {
        assert(bid == 0);
        return beam_;
    }

    int cardinality() const {
        return beam_.cardinality();
    }

    void print(std::ostream &os) const {
        os << "[beam=" << beam_ << ", hidden=" << hidden_ << "]" << std::flush;
    }
    friend class pomdp_t;
};

inline std::ostream& operator<<(std::ostream &os, const belief_state_t &bel) {
    bel.print(os);
    return os;
}

struct feature_t : public POMDP::feature_t<belief_state_t> {
    feature_t(const belief_state_t &bel) {
#ifdef DEBUG
        std::cout << "bel=" << bel << std::endl;
        std::cout << "marginal:";
#endif
        marginals_ = std::vector<std::vector<float> >(1);
        marginals_[0] = std::vector<float>(beam_t::dim_, 0);
        for( beam_t::const_iterator it = bel.beam(0).begin(); it != bel.beam(0).end(); ++it ) {
            assert((it.value() >= 0) && (it.value() < beam_t::dim_));
            marginals_[0][it.value()] = it.prob();
#ifdef DEBUG
            std::cout << " " << it.prob() << "@" << it.value() << std::flush;
#endif
        }
#ifdef DEBUG
        std::cout << std::endl;
#endif
    }
    virtual ~feature_t() { }
};

class pomdp_t : public POMDP::pomdp_t<belief_state_t> {
  protected:
    int dim_;
    std::vector<POMDP::pomdp_t<belief_state_t>::varset_t> varsets_;

    mutable belief_state_t init_tmp_;

  public:
    pomdp_t(int dim) : POMDP::pomdp_t<belief_state_t>(DISCOUNT), dim_(dim) {
        POMDP::pomdp_t<belief_state_t>::varset_t varset;
        varset.push_back(0);
        varsets_.push_back(varset);
    }
    virtual ~pomdp_t() { }

    virtual Problem::action_t number_actions(const belief_state_t &bel) const {
        return 1 + dim_;
    }
    virtual const belief_state_t& init() const {
        init_tmp_ = belief_state_t(unsigned(-1), Random::random(0, dim_));
        return init_tmp_;
    }
    virtual bool terminal(const belief_state_t &bel) const {
        return bel.cardinality() == 1;
    }
    virtual bool dead_end(const belief_state_t &bel) const {
        return false;
    }
    virtual bool applicable(const belief_state_t &bel, ::Problem::action_t a) const {
        return true;
    }
    virtual float min_absolute_cost() const { return 1; }
    virtual float max_absolute_cost() const { return 1; }
    virtual float cost(const belief_state_t &bel, Problem::action_t a) const {
        return 1;
    }
    virtual int max_action_branching() const {
        return dim_;
    }
    virtual int max_state_branching() const {
        return 2;
    }
    virtual void next(const belief_state_t &bel, Problem::action_t a, std::vector<std::pair<belief_state_t, float> > &outcomes) const {
        ++expansions_;
        outcomes.clear();
        outcomes.reserve(2);
        belief_state_t lower = bel.apply(a, 0);
        float p_lower = float(lower.cardinality()) / float(bel.cardinality());
        belief_state_t upper = bel.apply(a, 1);
        float p_upper = float(upper.cardinality()) / float(bel.cardinality());
        assert(p_lower + p_upper == 1);
        if( p_lower > 0 ) outcomes.push_back(std::make_pair(lower, p_lower));
        if( p_upper > 0 ) outcomes.push_back(std::make_pair(upper, p_upper));
    }

    // POMDP virtual methods
    virtual int num_variables() const {
        return 1;
    }
    virtual int num_beams() const {
        return 1;
    }
    virtual const POMDP::pomdp_t<belief_state_t>::varset_t& varset(int bid) const {
        return varsets_[0];
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

    virtual void apply_action(belief_state_t &bel_a, Problem::action_t a) const { /* work done below */ }
    virtual void apply_obs(belief_state_t &bel_ao, Problem::action_t a, POMDP::observation_t obs) const {
        belief_state_t nbel = bel_ao.apply(a, obs);
        bel_ao = nbel;
    }
    virtual POMDP::observation_t sample_observation_using_hidden_state(const belief_state_t &bel, const belief_state_t &bel_a, Problem::action_t a) const {
        return bel.hidden_ < a ? 0 : 1;
    }

    virtual void print(std::ostream &os) const {
    }

    const beam_t& beam(const belief_state_t &bel, int bid) const {
        return bel.beam(bid);
    }
};

inline std::ostream& operator<<(std::ostream &os, const pomdp_t &p) {
    p.print(os);
    return os;
}

#undef DEBUG

