#ifndef BIN_H
#define BIN_H

#include "ordered_vector.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <math.h>

class bin_t {
    int row_;
    int col_;
    int type_;
    ordered_vector_t bin_;

    static int virgin_size_[];
    static int num_objs_[];

  public:
    bin_t(int row, int col, int type) : row_(row), col_(col), type_(type) { }
    explicit bin_t(const bin_t &bin)
      : row_(bin.row_), col_(bin.col_), type_(bin.type_), bin_(bin.bin_) { }
    bin_t(bin_t &&bin)
      : row_(bin.row_), col_(bin.col_), type_(bin.type_), bin_(std::move(bin.bin_)) { }
    ~bin_t() { }

    enum { TOP = 1, BOTTOM = 2, LEFT = 4, RIGHT = 8 };

    static void initialize() {
        static bool initialized = false;
        if( initialized ) return;
        initialized = true;

        std::cout << "bin_t: initialization" << std::endl;
        for( int p = 0; p < 512; ++p ) {
            int num = 0;
            for( int q = p; q != 0; q = q >> 1 ) {
                num += (q & 0x1);
            }
            num_objs_[p] = num;
        }

        for( int type = 0; type < 16; ++type ) {
            int num = -1;
            switch( type ) {
                case 0: num = 512; break;
                case 1:
                case 2:
                case 4:
                case 8: num = 64; break;
                case 5:
                case 6:
                case 9:
                case 10: num = 16; break;
            }
            virgin_size_[type] = num;
        }
    }

    int row() const { return row_; }
    int col() const { return col_; }
    bool empty() const { return bin_.empty(); }
    int size() const { return bin_.size(); }

    void clear() { bin_.clear(); }

    // determine (for sure) the status of the object at bin's cell
    bool status_obj_at(int status) const {
        for( ordered_vector_t::const_iterator it = bin_.begin(); it != bin_.end(); ++it ) {
            int p = *it;
            if( ((p >> 4) & 0x1) != status ) return false;
        }
        return true;
    }
    bool obj_at() const { return !empty() && status_obj_at(1); }
    bool no_obj_at() const { return !empty() && status_obj_at(0); }

    // determine max number of objects sorrounding this cell
    std::pair<int, int> num_surrounding_objs() const {
        int min_nobjs = 9, max_nobjs = 0;
        for( ordered_vector_t::const_iterator it = bin_.begin(); it != bin_.end(); ++it ) {
            int p = *it;
            if( p & 0x10 ) continue;
            int nobjs = num_objs_[p];
            min_nobjs = nobjs < min_nobjs ? nobjs : min_nobjs;
            max_nobjs = nobjs > max_nobjs ? nobjs : max_nobjs;
        }
        return std::make_pair(min_nobjs, max_nobjs);
    }

    float obj_probability(float prior) const {
        float mass = 0, total = 0;
        for( ordered_vector_t::const_iterator it = bin_.begin(); it != bin_.end(); ++it ) {
            int p = *it;
            int n = num_objs_[p];
            float prob = powf(prior, n) * powf(1.0 - prior, 9 - n);
            if( (p >> 4) & 0x1 ) {
                mass += prob;
            }
            total += prob;
        }
        return mass / total;
    }

    bool virgin() const { return bin_.size() == virgin_size_[type_]; }

    void erase_ordered_indices(const std::vector<int> &indices) {
        bin_.erase_ordered_indices(indices);
    }

    void set_as_unknown(int neighbourhood) {
        bin_.clear();
        for( int p = 0; p < 512; ++p ) {
            int q = p & neighbourhood;
            if( (type_ & TOP) && ((q & 0x40) || (q & 0x80) || (q & 0x100)) ) continue;
            if( (type_ & BOTTOM) && ((q & 0x1) || (q & 0x2) || (q & 0x4)) ) continue;
            if( (type_ & LEFT) && ((q & 0x1) || (q & 0x8) || (q & 0x40)) ) continue;
            if( (type_ & RIGHT) && ((q & 0x4) || (q & 0x20) || (q & 0x100)) ) continue;
            bin_.insert(q);
        }
    }

    void insert(int e) { bin_.insert(e); }

    void filter(int nobjs, bool at_least = false) {
        assert((0 <= nobjs) && (nobjs <= 9));
        static std::vector<int> indices_to_erase;
        indices_to_erase.clear();
        indices_to_erase.reserve(bin_.size());
        for( ordered_vector_t::const_iterator it = bin_.begin(); it != bin_.end(); ++it ) {
            int p = *it;
            if( nobjs == 9 ) {
                if( (p & 0x10) == 0 )
                    indices_to_erase.push_back(it.index());
            } else {
                if( (p & 0x10) ||
                    (!at_least && (num_objs_[p] != nobjs)) ||
                    (at_least && (num_objs_[p] < nobjs)) )
                    indices_to_erase.push_back(it.index());
            }
        }
        bin_.erase_ordered_indices(indices_to_erase);
    }

    bool operator==(const bin_t &bin) const {
        return (row_ == bin.row_) && (col_ == bin.col_) && (bin_ == bin.bin_);
    }
    bool operator!=(const bin_t &bin) const {
        return *this == bin ? false : true;
    }

    const bin_t& operator=(const bin_t &bin) {
//std::cout << "bin_t::operator=" << std::endl;
        row_ = bin.row_;
        col_ = bin.col_;
        type_ = bin.type_;
        bin_ = bin.bin_;
        return *this;
    }

    void print(std::ostream &os) const {
        os << bin_;
    }

    typedef ordered_vector_t::const_iterator const_iterator;
    const_iterator begin() const { return bin_.begin(); }
    const_iterator end() const { return bin_.end(); }
};

int bin_t::virgin_size_[16];
int bin_t::num_objs_[512];

inline std::ostream& operator<<(std::ostream &os, const bin_t &bin) {
    bin.print(os);
    return os;
}

#endif

