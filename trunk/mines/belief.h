#ifndef BELIEF_H
#define BELIEF_H

#include "bin.h"
#include <cassert>
#include <iostream>
#include <vector>

class belief_t {
  protected:
    static int rows_;
    static int cols_;
    static int *types_;
    static int offsets_[512][9];

    typedef std::pair<int, int> edge_t;
    typedef std::vector<int> node_list_t;
    typedef std::vector<edge_t> edge_list_t;

    static int n_cg_edges_;
    static std::vector<edge_list_t> cg_edges_incident_at_;
    static std::vector<int> cells_to_revise_[25];

  public:
    belief_t() { }
    belief_t(const belief_t &bel) { }
    belief_t(belief_t &&bel) { }
    virtual ~belief_t() { }

    enum { manhattan_neighbourhood = 186, octile_neighbourhood = 511 };

    // neighbourhood is used to set up the cells to revise when
    // doing arc consistency. Two values had been validated (tested):
    // the value 511 for octile neighbourboods, and the value
    // 186 for 'manhattan' neighbourhoods.
    static void initialize(int rows, int cols, int neighbourhood) {
        static bool initialized = false;
        if( initialized && (rows_ == rows) && (cols_ == cols) ) return;
        initialized = true;

        std::cout << "belief_t: initialization: "
                  << "rows=" << rows
                  << ", cols=" << cols
                  << ", neighbourhood=" << neighbourhood
                  << std::endl;

        rows_ = rows;
        cols_ = cols;

        for( int p = 0; p < 512; ++p ) {
            for( int i = 0; i < 9; ++i ) {
                offsets_[p][i] = (p & (1<<i)) ? 1+i : -(1+i);
            }
        }

        // bin types
        types_ = new int[rows_ * cols_];
        for( int r = 0; r < rows_; ++r ) {
            for( int c = 0; c < cols_; ++c ) {
                int type = 0;
                if( r == 0 ) type += bin_t::BOTTOM;
                if( r == rows_ - 1 ) type += bin_t::TOP;
                if( c == 0 ) type += bin_t::LEFT;
                if( c == cols_ - 1 ) type += bin_t::RIGHT;
                types_[r * cols_ + c] = type;
            }
        }

        // constraint graph
        construct_constraint_graph(rows_, cols_, neighbourhood);

        // cells to revise
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

    static bool skip_cell(int cell, int i) {
        int type = types_[cell];
        if( (type & bin_t::TOP) && ((i == 6) || (i == 7) || (i == 8)) ) return true;
        if( (type & bin_t::BOTTOM) && ((i == 0) || (i == 1) || (i == 2)) ) return true;
        if( (type & bin_t::LEFT) && ((i == 0) || (i == 3) || (i == 6)) ) return true;
        if( (type & bin_t::RIGHT) && ((i == 2) || (i == 5) || (i == 8)) ) return true;
        return false;
    }

    virtual bool inconsistent() const = 0;
    virtual void clear() = 0;
    virtual void set_as_unknown() = 0;

    void filter(std::vector<bin_t*> &bins, int cell, int nobjs, bool at_least = false) {
        assert((0 <= cell) && (cell < rows_ * cols_));
        bins[cell]->filter(nobjs, at_least);
    }

    virtual const belief_t& operator=(const belief_t &bel) {
        return *this;
    }

    virtual bool operator==(const belief_t &bel) const {
        if( (rows_ != bel.rows_) || (cols_ != bel.cols_) )
            return false;
        else
            return true;
    }
    virtual bool operator!=(const belief_t &bel) const {
        return *this == bel ? false : true;
    }

    virtual void print(std::ostream &os) const = 0;

#if 0
    void decompose_state(const int *state, belief_t *bel) const {
        for( int r = 0; r < rows_; ++r ) {
            for( int c = 0; c < cols_; ++c ) {
                int cell = r * cols_ + c;
                int i = 0, p = 0;
                for( int dr = -1; dr < 2; ++dr ) {
                    for( int dc = -1; dc < 2; ++dc, ++i ) {
                        if( skip_cell(cell, i) ) continue;
                        int dcell = dr * cols_ + dc;
                        if( state[cell + dcell] == 1 ) p += (1<<i);
                    }
                }
                assert(p < 512);
                bel->bins_[cell]->insert(p);
            }
        }
    }

    void print_state(const int *state, belief_t*) const {
        std::cout << "State:";
        for( int i = 0; i < rows_ * cols_; ++i ) {
            std::cout << ((i > 0) && (i % cols_ == 0) ? " | " : " ");
            std::cout << state[i];
        }
        std::cout << std::endl;
    }

    void count_states(const int *, belief_t*) const {
        ++num_states_;
    }

    void recursive_join(const std::vector<bin_t*> &bins,
                        int index,
                        int *state,
                        belief_t *bel,
                        void (belief_t::*func)(const int*, belief_t*) const) const {

        if( index == (int)bins.size() ) {
            // if we reach this point is becase state is consistent
            (this->*func)(state, bel);
        } else {
            std::vector<int> marks;
            marks.reserve(9);

            bin_t &bin = *bins[index];
            int cell = bin.row() * cols_ + bin.col();
            for( bin_t::const_iterator it = bin.begin(); it != bin.end(); ++it ) {
                // mark and do consistency check
                marks.clear();
                bool consistent = true;
                for( int i = 0; i < 9; ++i ) {
                    if( skip_cell(cell, i) ) continue;

                    int entry = offsets_[*it][i];
                    assert(entry != 0);
                    int obj = entry > 0 ? 1 : 0;
                    entry = entry > 0 ? entry - 1 : -entry - 1;
                    int drow = (entry / 3) - 1;
                    int dcol = (entry % 3) - 1;
                    int dcell = drow * cols_ + dcol;

                    if( state[cell + dcell] == -1 ) {
                        marks.push_back(cell + dcell);
                        state[cell + dcell] = obj;
                    } else if( state[cell + dcell] != obj ) {
                        consistent = false;
                        break;
                    }
                }

                // recursive call
                if( consistent )
                    recursive_join(bins, index + 1, state, bel, func);

                // undo marking
                for( int i = 0; i < (int)marks.size(); ++i )
                    state[marks[i]] = -1;
            }
        }
    }

    void join_and_decompose(belief_t &bel) const {
        std::vector<bin_t*> ordered_bins(bins_);
        std::vector<int> state(rows_ * cols_, -1);
        recursive_join(ordered_bins, 0, &state[0], &bel, &belief_t::decompose_state);
    }

    void print_join() const {
        std::vector<bin_t*> ordered_bins(bins_);
        std::vector<int> state(rows_ * cols_, -1);
        recursive_join(ordered_bins, 0, &state[0], 0, &belief_t::print_state);
    }

    unsigned count_states() const {
        num_states_ = 0;
        std::vector<bin_t*> ordered_bins(bins_);
        std::vector<int> state(rows_ * cols_, -1);
        recursive_join(ordered_bins, 0, &state[0], 0, &belief_t::count_states);
        return num_states_;
    }
#endif

    // Consistency methods
    static void construct_constraint_graph(int rows, int cols, int neighbourhood) {
        int n_cg_edges_ = 0;
        cg_edges_incident_at_.clear();
        cg_edges_incident_at_ = std::vector<edge_list_t>(rows * cols);
        for( int r = 0; r < rows; ++r ) {
            for( int c = 0; c < cols; ++c ) {
                int cell = r * cols + c;
                cg_edges_incident_at_[cell].reserve(25);
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
                            cg_edges_incident_at_[cell].push_back(std::make_pair(ncell, cell));
                            ++n_cg_edges_;
                        }
                    }
                }
            }
        }
        std::cout << "cg: #edges=" << n_cg_edges_ << std::endl;
    }

    bool consistent(int row_diff, int col_diff, int px, int py) const {
        int index = (row_diff + 2) * 5 + (col_diff + 2);
        assert((0 <= index) && (index < 25));
        for( int k = 0, ksz = cells_to_revise_[index].size(); k < ksz; ++k ) {
            int entry = cells_to_revise_[index][k];
            int i = entry / 9, j = entry % 9;
            if( ((px >> i) & 0x1) != ((py >> j) & 0x1) ) {
                return false;
            }
        }
        return true;
    }

    bool arc_reduce(std::vector<bin_t*> &bins, int bin_x, int bin_y) {
        assert(bin_x != bin_y);
        static std::vector<int> indices_to_erase;
        indices_to_erase.reserve(bins[bin_x]->size());
        indices_to_erase.clear();

        int row_x = bin_x / cols_, col_x = bin_x % cols_;
        int row_y = bin_y / cols_, col_y = bin_y % cols_;
        int row_diff = row_y - row_x;
        int col_diff = col_y - col_x;

        int oldsz = bins[bin_x]->size();
        for( ordered_vector_t::const_iterator it = bins[bin_x]->begin(); it != bins[bin_x]->end(); ++it ) {
            bool found = false;
            for( ordered_vector_t::const_iterator jt = bins[bin_y]->begin(); jt != bins[bin_y]->end(); ++jt ) {
                if( consistent(row_diff, col_diff, *it, *jt) ) {
                    found = true;
                    break;
                }
            }
            if( !found ) indices_to_erase.push_back(it.index());
        }
        bins[bin_x]->erase_ordered_indices(indices_to_erase);
        assert(indices_to_erase.empty() || (bins[bin_x]->size() < oldsz));
        return !indices_to_erase.empty();
    }

    void arc_revise(int bin_x, int bin_y, edge_list_t &worklist) {
        for( int i = 0, isz = cg_edges_incident_at_[bin_x].size(); i < isz; ++i ) {
            int nbin = cg_edges_incident_at_[bin_x][i].first;
            assert(nbin != bin_x);
            if( nbin != bin_y )
                worklist.push_back(std::make_pair(nbin, bin_x));
        }
    }

    // used to update mark cells discovered by constraint propagation
    virtual void mark_cell(std::vector<bin_t*> &bins, int cell, bool hazard) = 0;

    // AC3: Arc Consistency. Time is O(ed^3) where e is #edges in constraint graph and
    // d is size of larges domain: there are at most 2ed arc revisions since each revision
    // or arc (y,x) is caused by a removed element from the domain of x (D_x), and each
    // such revision takes time d^2 because for each element of D_x, a consistent element
    // of D_y must be found. Space is O(e) since this is the maximum size of the worklist.
    void ac3(std::vector<bin_t*> &bins, int seed_bin, bool propagate) {
        assert(seed_bin >= 0);
        assert(seed_bin < rows_ * cols_);
        assert(hazard_at(bins, seed_bin) || no_hazard_at(bins, seed_bin));

        // initial worklist are all edges ?bin -> seed_bin
        edge_list_t worklist;
        worklist.reserve(n_cg_edges_);
        worklist = cg_edges_incident_at_[seed_bin];

        // revise arcs until worklist becomes empty
        while( !worklist.empty() ) {
            edge_t edge = worklist.back();
            worklist.pop_back();
            int bin_x = edge.first;
            int bin_y = edge.second;

            // update hazard and no_hazard lists
            if( hazard_at(bins, bin_y) ) mark_cell(bins, bin_y, true);
            if( no_hazard_at(bins, bin_y) ) mark_cell(bins, bin_y, false);

            // try to reduce arc bin_x -> bin_y
            if( arc_reduce(bins, bin_x, bin_y) ) {
                if( bins[bin_x]->empty() ) {
                    clear(); // bin_x became empty, clear all bins
                    return;
                } else {
                    // some element was removed from domain of bin_x,
                    // schedule revision of arc incident at bin_x
                    // different from arc bin_y -> bin_x
                    if( propagate ) {
                        arc_revise(bin_x, bin_y, worklist);
                    } else {
                        // save for future propagation
                        assert(0);
                    }
                }
            }
        }
    }

    // Knowledge-query methods
    virtual bool hazard_at(const std::vector<bin_t*> &bins, int cell) const {
        return bins[cell]->obj_at();
    }
    bool no_hazard_at(const std::vector<bin_t*> &bins, int cell) const {
        return bins[cell]->no_obj_at();
    }
    std::pair<int, int> num_surrounding_objs(const std::vector<bin_t*> &bins, int cell) const {
        return bins[cell]->num_surrounding_objs();
    }
    float obj_probability(const std::vector<bin_t*> &bins, int cell, float prior) const {
        return bins[cell]->obj_probability(prior);
    }
    bool virgin(const std::vector<bin_t*> &bins, int cell) const {
        return bins[cell]->virgin();
    }

};

int belief_t::rows_ = 0;
int belief_t::cols_ = 0;
int *belief_t::types_ = 0;
int belief_t::offsets_[512][9];

int belief_t::n_cg_edges_ = 0;
std::vector<belief_t::edge_list_t> belief_t::cg_edges_incident_at_;
std::vector<int> belief_t::cells_to_revise_[25];

#endif

