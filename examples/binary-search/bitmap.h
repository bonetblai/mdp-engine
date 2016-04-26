#include <iostream>
#include <iomanip>
#include <strings.h>

#define BITS_IN_UNSIGNED    (8 * sizeof(unsigned))

//#define DEBUG_CTOR_DTOR
//#define DEBUG

namespace Bitmap {

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
#ifdef DEBUG_CTOR_DTOR
        std::cout << "bitmap_t: ctor called" << std::endl;
#endif
    }
    bitmap_t(const bitmap_t &bitmap) {
        bitmap_ = new unsigned[dim_in_words_];
        *this = bitmap;
#ifdef DEBUG_CTOR_DTOR
        std::cout << "bitmap_t: copy ctor called" << std::endl;
#endif
    }
    bitmap_t(const bitmap_t &bitmap, const bitmap_t &mask) {
        bitmap_ = new unsigned[dim_in_words_];
        for( int i = 0; i < dim_in_words_; ++i )
            bitmap_[i] = bitmap.bitmap_[i] & mask.bitmap_[i];
#ifdef DEBUG_CTOR_DTOR
        std::cout << "bitmap_t: copy ctor with mask called" << std::endl;
#endif
    }
    bitmap_t(bitmap_t &&bitmap) {
        bitmap_ = bitmap.bitmap_;
        bitmap.bitmap_ = 0;
#ifdef DEBUG_CTOR_DTOR
        std::cout << "bitmap_t: move ctor called" << std::endl;
#endif
    }
    ~bitmap_t() {
        delete[] bitmap_;
#ifdef DEBUG_CTOR_DTOR
        std::cout << "bitmap_t: dtor called" << std::endl;
#endif
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

    int operator[](int i) const { // returns i-th bit
        return bit(i);
    }
    const bitmap_t& operator=(const bitmap_t &bitmap) {
        for( int i = 0; i < dim_in_words_; ++i )
            bitmap_[i] = bitmap.bitmap_[i];
        return *this;
    }
    bool operator==(const bitmap_t &bitmap) const {
        for( int i = 0; i < dim_in_words_; ++i ) {
            if( bitmap_[i] != bitmap.bitmap_[i] )
                return false;
        }
        return true;
    }
    bool operator!=(const bitmap_t &bitmap) const {
        return !(*this == bitmap);
    }
    bool operator<(const bitmap_t &bitmap) const {
        for( int i = 0; i < dim_in_words_; ++i ) {
            if( bitmap_[i] < bitmap.bitmap_[i] )
                return true;
            else if( bitmap_[i] > bitmap.bitmap_[i] )
                return false;
        }
        return false;
    }

    int bit(int i) const {
        int index = i / BITS_IN_UNSIGNED, offset = i % BITS_IN_UNSIGNED;
        return (bitmap_[index] >> offset) & 1;
    }
    void set_bit(int i, int bit) {
        int index = i / BITS_IN_UNSIGNED, offset = i % BITS_IN_UNSIGNED;
        unsigned bitmap = bitmap_[index] & ~(1 << offset);
        bitmap = bitmap | (bit << offset);
        bitmap_[index] = bitmap;
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

}; // namespace Bitmap

inline std::ostream& operator<<(std::ostream &os, const Bitmap::bitmap_t &bitmap) {
    bitmap.print(os);
    return os;
}

inline std::ostream& operator<<(std::ostream &os, const Bitmap::bitmap_t::const_iterator &it) {
    it.print(os);
    return os;
}

#undef DEBUG

