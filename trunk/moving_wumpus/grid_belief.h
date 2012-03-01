#ifndef GRID_BELIEF_H
#define GRID_BELIEF_H

#include "cell_bin.h"
#include "arc_consistency.h"
#include <cassert>
#include <iostream>
#include <vector>

class grid_arc_consistency_t : public CSP::arc_consistency_t<cell_bin_t> {
    mutable int row_x_, row_y_, row_diff_;
    mutable int col_x_, col_y_, col_diff_;
    static std::vector<int> cells_to_revise_[25];

    // disallow copy constructor
    explicit grid_arc_consistency_t(const grid_arc_consistency_t &);
    explicit grid_arc_consistency_t(grid_arc_consistency_t &&);

  public:
    grid_arc_consistency_t();
    virtual ~grid_arc_consistency_t() { }
    static void initialize(int neighbourhood);

    virtual void arc_reduce_preprocessing(int bin_x, int bin_y);

    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const {
        int index = (row_diff_ + 2) * 5 + (col_diff_ + 2);
        assert((0 <= index) && (index < 25));
        for( int k = 0, ksz = cells_to_revise_[index].size(); k < ksz; ++k ) {
            int entry = cells_to_revise_[index][k];
            int i = entry / 9, j = entry % 9;
            if( ((val_x >> i) & 0x1) != ((val_y >> j) & 0x1) ) {
                return false;
            }
        }
        return true;
    }

    virtual void arc_reduce_postprocessing(int var_x, int var_y) { }

    const grid_arc_consistency_t& operator=(const grid_arc_consistency_t &arc) {
        assert(0); // shouldn't be called
    }
    bool operator==(const grid_arc_consistency_t &arc) {
        assert(0); // shouldn't be called
    }
};

class grid_belief_t {
  protected:
    static int rows_;
    static int cols_;
    static int *types_;
    static CSP::constraint_digraph_t cg_;

  public:
    grid_belief_t() { }
    grid_belief_t(const grid_belief_t &bel) { }
    //grid_belief_t(grid_belief_t &&bel) { }
    virtual ~grid_belief_t() { }

    enum { manhattan_neighbourhood = 186, octile_neighbourhood = 511 };

    // neighbourhood is used to set up the cells to revise when
    // doing arc consistency. Two values had been validated (tested):
    // the value 511 for octile neighbourboods, and the value
    // 186 for 'manhattan' neighbourhoods.
    static void initialize(int rows, int cols, int neighbourhood) {
        static bool initialized = false;
        if( initialized && (rows_ == rows) && (cols_ == cols) ) return;
        initialized = true;

        std::cout << "grid_belief_t: initialization: "
                  << "rows=" << rows
                  << ", cols=" << cols
                  << ", neighbourhood=" << neighbourhood
                  << std::endl;

        rows_ = rows;
        cols_ = cols;

        // bin types
        types_ = new int[rows_ * cols_];
        for( int r = 0; r < rows_; ++r ) {
            for( int c = 0; c < cols_; ++c ) {
                int type = 0;
                if( r == 0 ) type += cell_bin_t::BOTTOM;
                if( r == rows_ - 1 ) type += cell_bin_t::TOP;
                if( c == 0 ) type += cell_bin_t::LEFT;
                if( c == cols_ - 1 ) type += cell_bin_t::RIGHT;
                types_[r * cols_ + c] = type;
            }
        }

        // constraint graph
        construct_constraint_graph(rows_, cols_, neighbourhood);
        grid_arc_consistency_t::initialize(neighbourhood);
    }

    static bool skip_cell(int cell, int i) {
        int type = types_[cell];
        if( (type & cell_bin_t::TOP) && ((i == 6) || (i == 7) || (i == 8)) ) return true;
        if( (type & cell_bin_t::BOTTOM) && ((i == 0) || (i == 1) || (i == 2)) ) return true;
        if( (type & cell_bin_t::LEFT) && ((i == 0) || (i == 3) || (i == 6)) ) return true;
        if( (type & cell_bin_t::RIGHT) && ((i == 2) || (i == 5) || (i == 8)) ) return true;
        return false;
    }

    static int rows() { return rows_; }
    static int cols() { return cols_; }
    static const CSP::constraint_digraph_t& cg() { return cg_; }

    void filter(std::vector<cell_bin_t*> &bins, int cell, int nobjs, bool at_least = false) {
        assert((0 <= cell) && (cell < rows_ * cols_));
        bins[cell]->filter(nobjs, at_least);
    }

    virtual const grid_belief_t& operator=(const grid_belief_t &bel) {
        return *this;
    }

    virtual bool operator==(const grid_belief_t &bel) const {
        if( (rows_ != bel.rows_) || (cols_ != bel.cols_) )
            return false;
        else
            return true;
    }
    virtual bool operator!=(const grid_belief_t &bel) const {
        return *this == bel ? false : true;
    }

    virtual void print(std::ostream &os) const = 0;

    // Consistency methods
    static void construct_constraint_graph(int rows, int cols, int neighbourhood) {
        cg_.create_empty_graph(rows * cols);
        for( int r = 0; r < rows; ++r ) {
            for( int c = 0; c < cols; ++c ) {
                int cell = r * cols + c;
                cg_.reserve_edge_list(cell, 25);
                for( int dr = -2; dr < 3; ++dr ) {
                    if( (r + dr < 0) || (r + dr >= rows) ) continue;
                    for( int dc = -2; dc < 3; ++dc ) {
                        if( (c + dc < 0) || (c + dc >= cols) ) continue;
                        if( (neighbourhood == manhattan_neighbourhood) &&
                            ((dr == -2) || (dr == 2)) && (dc != 0) )
                            continue;
                        if( (neighbourhood == manhattan_neighbourhood) &&
                            ((dc == -2) || (dc == 2)) && (dr != 0) )
                            continue;
                        int ncell = (r + dr) * cols + (c + dc);
                        if( ncell != cell ) {
                            cg_.add_edge(ncell, cell);
                        }
                    }
                }
            }
        }
        std::cout << "cg: #edges=" << cg_.nedges() << std::endl;
    }

};

int grid_belief_t::rows_ = 0;
int grid_belief_t::cols_ = 0;
int *grid_belief_t::types_ = 0;
CSP::constraint_digraph_t grid_belief_t::cg_;

grid_arc_consistency_t::grid_arc_consistency_t() : CSP::arc_consistency_t<cell_bin_t>(grid_belief_t::cg()) {
}

void grid_arc_consistency_t::initialize(int neighbourhood) {
    for( int k = 0; k < 25; ++k ) {
        cells_to_revise_[k].reserve(9);
        int row_diff = (k / 5) - 2, col_diff = (k % 5) - 2;
        int bin_x = 2 * 5 + 2;
        int bin_y = (2 + row_diff) * 5 + (2 + col_diff);
        for( int i = 0; i < 9; ++i ) {
            if( (neighbourhood & (1 << i)) == 0 ) continue;
            int cell_x = bin_x + (((i / 3) - 1) * 5 + ((i % 3) - 1));
            for( int j = 0; j < 9; ++j ) {
                if( (neighbourhood & (1 << j)) == 0 ) continue;
                int cell_y = bin_y + (((j / 3) - 1) * 5 + ((j % 3) - 1));
                if( cell_x == cell_y ) {
                    int entry = i * 9 + j;
                    cells_to_revise_[k].push_back(entry);
                }
            }
        }
    }
}

void grid_arc_consistency_t::arc_reduce_preprocessing(int bin_x, int bin_y) {
    row_x_ = bin_x / grid_belief_t::cols(), col_x_ = bin_x % grid_belief_t::cols();
    row_y_ = bin_y / grid_belief_t::cols(), col_y_ = bin_y % grid_belief_t::cols();
    row_diff_ = row_y_ - row_x_;
    col_diff_ = col_y_ - col_x_;
}

std::vector<int> grid_arc_consistency_t::cells_to_revise_[25];

#endif

