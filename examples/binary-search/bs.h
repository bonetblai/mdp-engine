#include <iostream>
#include <iomanip>
#include <strings.h>

#define DISCOUNT            1
#define BITS_IN_UNSIGNED    (8 * sizeof(unsigned))

//#define DEBUG

struct bitmap_t {
    unsigned *bitmap_;

    static int dim_;
    static int dim_in_words_;
    static int bits_in_last_word_;
    static unsigned last_word_mask_;

    struct const_iterator {
        const unsigned *bitmap_;
        int offset_;
        int pos_;
        int max_pos_;

        enum { Begin, End }; // iterator type

        void increase_pos() {
            ++pos_;
            offset_ = (1 + offset_ ) & 0x1f;
            if( offset_ == 0 ) ++bitmap_;
        }
        int current_bit() {
            return (*bitmap_ >> offset_) & 1;
        }

        const_iterator(const unsigned *bitmap, int dim_in_words, int bits_in_last_word, int type, int max_pos) {
            max_pos_ = max_pos;
            if( type == Begin ) {
                bitmap_ = bitmap;
                offset_ = 0;
                pos_ = 0;
                if( current_bit() == 0 ) ++(*this);
            } else {
                if( bits_in_last_word == BITS_IN_UNSIGNED ) {
                    bitmap_ = &bitmap[dim_in_words];
                    offset_ = 0;
                } else {
                    bitmap_ = &bitmap[dim_in_words - 1];
                    offset_ = bits_in_last_word;
                }
                pos_ = max_pos_;
            }
        }
        const_iterator(const const_iterator &it)
          : bitmap_(it.bitmap_), offset_(it.offset_), pos_(it.pos_), max_pos_(it.max_pos_) {
        }

        bool operator==(const const_iterator &it) const {
            return (bitmap_ == it.bitmap_) && (offset_ == it.offset_) && (pos_ == it.pos_) && (max_pos_ == it.max_pos_);
        }
        bool operator!=(const const_iterator &it) const {
            return !(*this == it);
        }
        const const_iterator& operator++() {
            if( pos_ < max_pos_ )
                for( increase_pos(); (pos_ < max_pos_) && (current_bit() == 0); increase_pos() );
            return *this;
        }

        int value() const {
            return pos_;
        }

        void print(std::ostream &os) const {
            os << "(ptr=" << bitmap_ << ",off=" << offset_ << ",pos=" << pos_ << ",mpos=" << max_pos_ << ")" << std::flush;
        }
    }; // const_iterator

    bitmap_t(unsigned bitmap = unsigned(-1)) {
        bitmap_ = new unsigned[dim_in_words_];
        for( int i = 0; i < dim_in_words_; ++i )
            bitmap_[i] = bitmap;
        bitmap_[dim_in_words_ - 1] = bitmap_[dim_in_words_ - 1] & last_word_mask_;
    }
    bitmap_t(const bitmap_t &bitmap) {
        bitmap_ = new unsigned[dim_in_words_];
        *this = bitmap;
    }
    bitmap_t(const bitmap_t &bitmap, const bitmap_t &mask) {
        bitmap_ = new unsigned[dim_in_words_];
        for( int i = 0; i < dim_in_words_; ++i )
            bitmap_[i] = bitmap[i] & mask[i];
    }
    bitmap_t(bitmap_t &&bitmap) {
        bitmap_ = bitmap.bitmap_;
        bitmap.bitmap_ = 0;
    }
    ~bitmap_t() {
        delete[] bitmap_;
    }

    static void set_dimension(int dim) {
        dim_ = dim;
        dim_in_words_ = dim >> 5;
        bits_in_last_word_ = dim_ - (dim_in_words_ << 5);
        if( bits_in_last_word_ > 0 )
            ++dim_in_words_;
        else
            bits_in_last_word_ = BITS_IN_UNSIGNED;

        last_word_mask_ = 0;
        for( int i = 0; i < bits_in_last_word_; ++i ) {
            last_word_mask_ = last_word_mask_ << 1;
            ++last_word_mask_;
        }
        std::cout << "bitmap_t: dim=" << dim_
                  << ", dim_in_words=" << dim_in_words_
                  << ", bits_in_last_word=" << bits_in_last_word_
                  << ", last_word_mask=";
        Utils::print_bits(std::cout, last_word_mask_, BITS_IN_UNSIGNED);
        std::cout << std::endl;
    }

    unsigned operator[](int i) const {
        return bitmap_[i];
    }
    const bitmap_t& operator=(const bitmap_t &bitmap) {
        for( int i = 0; i < dim_in_words_; ++i )
            bitmap_[i] = bitmap[i];
        return *this;
    }
    bool operator==(const bitmap_t &bitmap) const {
        for( int i = 0; i < dim_in_words_; ++i ) {
            if( bitmap_[i] != bitmap[i] )
                return false;
        }
        return true;
    }
    bool operator!=(const bitmap_t &bitmap) const {
        return !(*this == bitmap);
    }
    bool operator<(const bitmap_t &bitmap) const {
        for( int i = 0; i < dim_in_words_; ++i ) {
            if( bitmap_[i] < bitmap[i] )
                return true;
            else if( bitmap_[i] > bitmap[i] )
                return false;
        }
        return false;
    }

    int popcount() const {
        int pcount = 0;
        for( int i = 0; i < dim_in_words_; ++i )
            pcount += __builtin_popcount(bitmap_[i]);
        return pcount;
    }

    unsigned hash() const {
        unsigned value = 0;
        for( int i = 0; i < dim_in_words_; ++i )
            value = value ^ bitmap_[i];
        return value;
    }

    void lshift(unsigned initial_carry = 0) {
        unsigned carry = initial_carry;
        for( int i = 0; i < dim_in_words_; ++i ) {
            unsigned bitmap = bitmap_[i];
            bitmap_[i] = bitmap_[i] << 1;
            bitmap_[i] += carry;
            carry = bitmap >> (BITS_IN_UNSIGNED - 1);
        }
        bitmap_[dim_in_words_ - 1] = bitmap_[dim_in_words_ - 1] & last_word_mask_;
    }
    void rshift(unsigned initial_carry = (1 << (BITS_IN_UNSIGNED - 1))) {
        unsigned carry = initial_carry;
        for( int i = dim_in_words_ - 1; i >= 0; --i ) {
            unsigned bitmap = bitmap_[i];
            bitmap_[i] = bitmap_[i] >> 1;
            bitmap_[i] += carry;
            carry = (bitmap & 1) << (BITS_IN_UNSIGNED - 1);
        }
    }
    void complement() {
        for( int i = 0; i < dim_in_words_; ++i )
            bitmap_[i] = ~bitmap_[i];
        bitmap_[dim_in_words_ - 1] = bitmap_[dim_in_words_ - 1] & last_word_mask_;
    }

    const_iterator begin() const {
        return const_iterator(bitmap_, dim_in_words_, bits_in_last_word_, const_iterator::Begin, dim_);
    }
    const_iterator end() const {
        return const_iterator(bitmap_, dim_in_words_, bits_in_last_word_, const_iterator::End, dim_);
    }

    void print(std::ostream &os) const {
        os << "[bitmap=[";
        Utils::print_bits(os, bitmap_[dim_in_words_ - 1], bits_in_last_word_);
        os << "]";
        for( int i = dim_in_words_ - 2; i >= 0; --i ) {
            os << ":[";
            Utils::print_bits(os, bitmap_[i], BITS_IN_UNSIGNED);
            os << "]";
        }
        os << "]" << std::flush;
    }
};

inline std::ostream& operator<<(std::ostream &os, const bitmap_t &bitmap) {
    bitmap.print(os);
    return os;
}

inline std::ostream& operator<<(std::ostream &os, const bitmap_t::const_iterator &it) {
    it.print(os);
    return os;
}

struct beam_t {
    bitmap_t bitmap_;

    struct const_iterator : public bitmap_t::const_iterator {
        const_iterator(const bitmap_t::const_iterator &it) : bitmap_t::const_iterator(it) { }
    };

    beam_t() { }
    beam_t(const bitmap_t &bitmap, const bitmap_t &mask)
      : bitmap_(bitmap, mask) {
    }
    ~beam_t() { }

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
        return bitmap_.popcount();
    }
    unsigned hash() const {
        return bitmap_.hash();
    }

    virtual const_iterator begin() const {
        return bitmap_.begin();
    }
    virtual const_iterator end() const {
        return bitmap_.end();
    }

    void print(std::ostream &os) const {
        os << bitmap_;
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
    static std::vector<bitmap_t> action_mask_;

  public:
    belief_state_t(int hidden = 0) : hidden_(hidden) {
    }
    belief_state_t(const bitmap_t &bitmap, const bitmap_t &mask, int hidden)
      : beam_(bitmap, mask),
        hidden_(hidden) {
    }
    belief_state_t(const belief_state_t &bel)
      : beam_(bel.beam_),
        hidden_(bel.hidden_) {
    }
    belief_state_t(belief_state_t &&bel)
      : beam_(std::move(bel.beam_)),
        hidden_(bel.hidden_) {
    }
    ~belief_state_t() { }

    static void set_bitmap_mask(int dim) {
        dim_ = dim;

        bitmap_t lower(0);
        action_mask_.reserve(2 * (1 + dim_));
        for( int i = 0; i <= dim_; ++i ) {
            bitmap_t upper(lower);
            upper.complement();
            action_mask_.push_back(lower);
            action_mask_.push_back(upper);
            lower.lshift(1);
#ifdef DEBUG
            std::cout << "action_mask[a=" << i << ",lower]=" << action_mask_[2 * i] << std::endl;
            std::cout << "action_mask[a=" << i << ",upper]=" << action_mask_[2 * i + 1] << std::endl;
#endif
        }
    }

    size_t hash() const {
        return beam_.hash();
    }

    belief_state_t apply(Problem::action_t a, int side) const {
        belief_state_t bel(beam_.bitmap_, action_mask_[2 * a + side], hidden_);
        return bel;
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
        return !(*this == bel);
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
        init_tmp_ = belief_state_t(Random::random(0, dim_));
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

    virtual void apply_action(belief_state_t &bel_a, Problem::action_t a) const { /* real work is done below */ }
    virtual void apply_obs(belief_state_t &bel_ao, Problem::action_t a, POMDP::observation_t obs) const {
        belief_state_t nbel = bel_ao.apply(a, obs);
        bel_ao = nbel;
    }
    virtual POMDP::observation_t sample_observation_using_hidden_state(const belief_state_t &bel, const belief_state_t &bel_a, Problem::action_t a) const {
        return bel.hidden_ < a ? 0 : 1;
    }

    virtual void print(std::ostream &os) const {
    }
};

inline std::ostream& operator<<(std::ostream &os, const pomdp_t &p) {
    p.print(os);
    return os;
}

#undef DEBUG

