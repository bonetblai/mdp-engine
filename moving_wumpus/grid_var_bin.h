#ifndef GRID_VAR_BIN_H
#define GRID_VAR_BIN_H

#include "ordered_vector.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <math.h>

inline int intpow(int base, int expn) {
    int result = 1;
    for( int i = 0; i < expn; ++i )
        result *= base;
    return result;
}

class grid_var_bin_t {
    int nvars_;
    int num_particles_;
    ordered_vector_t bin_;

    static int nrows_; 
    static int ncols_; 
    static int ncells_;

  public:
    grid_var_bin_t(int nvars)
      : nvars_(nvars), num_particles_(intpow(ncells_, nvars_)) { }
    explicit grid_var_bin_t(const grid_var_bin_t &bin)
      : nvars_(bin.nvars_), num_particles_(bin.num_particles_),
        bin_(bin.bin_) { }
    grid_var_bin_t(grid_var_bin_t &&bin)
      : nvars_(bin.nvars_), num_particles_(bin.num_particles_),
        bin_(std::move(bin.bin_)) { }
    ~grid_var_bin_t() { }

    static void initialize(int nrows, int ncols) {
        static bool initialized = false;
        if( initialized && (nrows_ == nrows) && (ncols_ == ncols) ) return;
        initialized = true;

        std::cout << "grid_var_bin_t: initialization:"
                  << " nrows=" << nrows
                  << ", ncols=" << ncols
                  << std::endl;

        nrows_ = nrows;
        ncols_ = ncols;
        ncells_ = nrows_ * ncols_;
    }

    static int nrows() { return nrows_; }
    static int ncols() { return ncols_; }

    enum { manhattan_neighbourhood = 0, octile_neighbourhood = 1 };
    static bool adjacent_cells(int row1, int col1, int cell, int neighbourhood) {
        int row2 = cell / ncols_, col2 = cell % ncols_;
        int abs_rdiff = abs(row1 - row2), abs_cdiff = abs(col1 - col2);
        if( neighbourhood == manhattan_neighbourhood ) {
            return ((abs_rdiff == 0) && (abs_cdiff <= 1)) ||
                   ((abs_cdiff == 0) && (abs_rdiff <= 1));
        } else {
            //return (abs_rdiff < 2) && (abs_cdiff < 2);
            return (abs_rdiff < 3) && (abs_cdiff < 3);
        }
    }
    static bool adjacent_cells(int cell1, int cell2, int neighbourhood) {
        return adjacent_cells(cell1 / ncols_, cell1 % ncols_, cell2, neighbourhood);
    }

    static int target_cell(int cell, int move) {
        switch( move ) {
          case 0:
            return cell + ncols_ >= ncells_ ? cell : cell + ncols_;
          case 1:
            return (cell % ncols_) == ncols_ - 1 ? cell : cell + 1;
          case 2:
            return cell - ncols_ < 0 ? cell : cell - ncols_;
          case 3:
            return (cell % ncols_) == 0 ? cell : cell - 1;
          case 4:
            return cell;
        }
        assert(0); // can't reach here
    }

    int value(int var, int p) const {
        for( ; var > 0; --var, p /= ncells_ );
        return p % ncells_;
    }

    bool consistent(int p) {
        for( int i = 0; i < nvars_; ++i ) {
            int val = value(i, p);
            for( int j = i+1; j < nvars_; ++j ) {
                if( val == value(j, p) ) return false;
            }
        }
        return true;
    }

    bool empty() const { return bin_.empty(); }
    int size() const { return bin_.size(); }
    bool known() const { return bin_.size() == 1; }
    bool contains(int value) const { return bin_.contains(value); }
    bool consistent() const { return !bin_.empty(); }

    void clear() { bin_.clear(); }
    void insert(int e) { bin_.insert(e); }
    void erase(int e) { bin_.erase(e); }
    void erase_ordered_indices(const std::vector<int> &indices) {
        bin_.erase_ordered_indices(indices);
    }

    void set_as_unknown() {
        bin_.clear();
        for( int p = 0; p < num_particles_; ++p ) {
            if( consistent(p) ) {
                bin_.insert(p);
            }
        }
    }

    bool operator==(const grid_var_bin_t &bin) const {
        return bin_ == bin.bin_;
    }
    bool operator!=(const grid_var_bin_t &bin) const {
        return *this == bin ? false : true;
    }

    const grid_var_bin_t& operator=(const grid_var_bin_t &bin) {
        bin_ = bin.bin_;
        return *this;
    }

    void print(std::ostream &os) const {
        os << "{";
        for( const_iterator it = begin(); it != end(); ++it ) {
            int p = *it;
            os << "[p=" << p << ":";
            for( int var = 0; var < nvars_; ++var ) {
                int cell = value(var, p);
                os << "v" << var << "=(" << (cell % ncols_) << "," << (cell / ncols_) << "),";
            }
            os << "],";
        }
        os << "}";
    }

    typedef ordered_vector_t::const_iterator const_iterator;
    const_iterator begin() const { return bin_.begin(); }
    const_iterator end() const { return bin_.end(); }

    // determine if it is necessary that there is an object at or adjacent to
    // to given cell:
    //
    //   status_cell(cell, true, true) : checks if obj adjacent to cell
    //   status_cell(cell, true, false) : checks if obj at cell
    //   status_cell(cell, false, true) : checks if no obj adjacent to cell
    //   status_cell(cell, false, false) : checks if no obj at cell
    //
    // neighbourhood is used to determine adjacency: 0=manhattan, 1=octile
    bool status_cell(int cell, bool check_obj, bool adjacent_to, int neighbourhood = manhattan_neighbourhood) const {
        int row = cell / ncols_, col = cell % ncols_;
        for( const_iterator it = begin(); it != end(); ++it ) {
            int p = *it;
            bool found = false;
            for( int var = 0; var < nvars_; ++var ) {
                int var_value = value(var, p);
                if( (!adjacent_to && (var_value == cell)) ||
                    (adjacent_to && adjacent_cells(row, col, var_value, neighbourhood)) ) {
                    found = true;
                    break;
                }
            }
            if( (check_obj && !found) || (!check_obj && found) ) return false;
        }
        return true;
    }
    bool necessary_obj_at(int cell) const { return status_cell(cell, true, false); }
    bool necessary_no_obj_at(int cell) const { return status_cell(cell, false, false); }
    bool possible_obj_at(int cell) const { return !necessary_no_obj_at(cell); }
    bool possible_no_obj_at(int cell) const { return !necessary_obj_at(cell); }
    bool necessary_obj_adjacent_to(int cell, int neighbourhood) const { return status_cell(cell, true, true, neighbourhood); }
    bool necessary_no_obj_adjacent_to(int cell, int neighbourhood) const { return status_cell(cell, false, true, neighbourhood); }
    bool possible_obj_adjacent_to(int cell, int neigbourhood) const { return !necessary_no_obj_adjacent_to(cell, neigbourhood); }
    bool possible_no_obj_adjacent_to(int cell, int neigbourhood) const { return !necessary_obj_adjacent_to(cell, neigbourhood); }

    // filter cell; parameters similar to status_cell
    void filter_cell(int cell, bool check_obj, bool adjacent_to, int neighbourhood = manhattan_neighbourhood) {
        static std::vector<int> indices_to_erase;
        indices_to_erase.clear();
        indices_to_erase.reserve(bin_.size());
        int row = cell / ncols_, col = cell % ncols_;
        for( const_iterator it = begin(); it != end(); ++it ) {
            int p = *it;
            bool found = false;
            for( int var = 0; var < nvars_; ++var ) {
                int var_value = value(var, p);
                if( (!adjacent_to && (var_value == cell)) ||
                    (adjacent_to && adjacent_cells(row, col, var_value, neighbourhood)) ) {
                    found = true;
                    break;
                }
            }
            if( (check_obj && !found) || (!check_obj && found) )
                indices_to_erase.push_back(it.index());
        }
        bin_.erase_ordered_indices(indices_to_erase);
    }
    void filter_obj_at(int cell) { filter_cell(cell, true, false); }
    void filter_no_obj_at(int cell) { filter_cell(cell, false, false); }
    void filter_obj_adjacent_to(int cell, int neighbourhood) { filter_cell(cell, true, true, neighbourhood); }
    void filter_no_obj_adjacent_to(int cell, int neighbourhood) { filter_cell(cell, false, true, neighbourhood); }

    void recursive_move_non_det(int p, int q, int m, int var, ordered_vector_t &bin) {
        if( var == nvars_ ) {
            if( consistent(q) ) bin.insert(q);
        } else {
            int cell = p % ncells_;
            int np = p / ncells_;
            int nm = m * ncells_;
            for( int i = 0; i < 5; ++i ) {
                int ncell = target_cell(cell, i);
                int nq = q + (ncell * m);
                recursive_move_non_det(np, nq, nm, 1+var, bin);
            }
        }
    }

    void move_non_det() {
        static ordered_vector_t tmp;
        tmp.reserve(intpow(5, nvars_));
        ordered_vector_t nbin_; // can't be static because of the std::move()
        for( const_iterator it = begin(); it != end(); ++it ) {
            int p = *it;
            recursive_move_non_det(p, 0, 1, 0, tmp);
            nbin_.insert(tmp);
            tmp.clear();
        }
        bin_ = std::move(nbin_);
    }
};

int grid_var_bin_t::nrows_ = 0;
int grid_var_bin_t::ncols_ = 0;
int grid_var_bin_t::ncells_ = 0;

inline std::ostream& operator<<(std::ostream &os, const grid_var_bin_t &bin) {
    bin.print(os);
    return os;
}

#endif

