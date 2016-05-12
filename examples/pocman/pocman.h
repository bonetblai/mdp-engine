#include <iostream>
#include <iomanip>
#include <strings.h>

#include "random.h"

#define DISCOUNT            0.95
#define NUM_ROWS            19
#define NUM_COLS            17
#define NUM_CELLS           (NUM_ROWS * NUM_COLS)

//#define DEBUG_CTOR_DTOR
//#define DEBUG

struct maze_t; // forward reference

struct loc_t {
    int col_;
    int row_;

    loc_t(int cell = 0)
      : col_(cell % NUM_COLS),
        row_(cell / NUM_COLS) {
        assert(cell >= 0);
    }
    loc_t(int col, int row)
      : col_(col),
        row_(row) {
        assert((col_>= 0) && (col_ < NUM_COLS));
        assert((row_>= 0) && (row_ < NUM_ROWS));
    }
    loc_t(const loc_t &loc)
      : col_(loc.col_),
        row_(loc.row_) {
    }
    loc_t(loc_t &&loc) = default;
    ~loc_t() { }

    const loc_t& operator=(const loc_t &loc) {
        col_ = loc.col_;
        row_ = loc.row_;
        return *this;
    }

    int as_integer() const {
        return row_ * NUM_COLS + col_;
    }

    enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
    void move_north(const maze_t &maze);
    void move_east(const maze_t &maze);
    void move_south(const maze_t &maze);
    void move_west(const maze_t &maze);
    void move(const maze_t &maze, int dir) {
        if( dir == NORTH )
            move_north(maze);
        else if( dir == EAST )
            move_east(maze);
        else if( dir == SOUTH )
            move_south(maze);
        else
            move_west(maze);
    }
    loc_t move(const maze_t &maze, int dir) const {
        loc_t loc(*this);
        loc.move(maze, dir);
        return loc;
    }

    void print(std::ostream &os) const {
        os << "(" << 1 + col_ << "," << 1 + row_ << ")";
    }
};

inline std::ostream& operator<<(std::ostream &os, const loc_t &loc) {
    loc.print(os);
    return os;
}

struct maze_t {
    int num_food_;
    int num_pills_;
    unsigned char *cells_;
    enum { FREE = 0, FOOD = 1, WALL = 2, PILL = 3 };

    static int num_valid_cells_;
    static unsigned char walls_[NUM_CELLS];
    static int cell_map_[NUM_CELLS];
    static int inv_cell_map_[NUM_CELLS];

    maze_t() {
        // construct maze from template
        cells_ = new unsigned char[NUM_CELLS];
        memcpy(cells_, walls_, NUM_CELLS);
        num_pills_ = 4;

        // randomly place food pellets
        num_food_ = 0;
        for( int i = 0; i < NUM_CELLS; ++i ) {
            if( (cells_[i] == FREE) && (i != loc_t(8, 8).as_integer()) )
                cells_[i] = Random::random(2) == 0 ? FREE : FOOD;
            num_food_ += cells_[i] == FOOD ? 1 : 0;
        }
    }
    maze_t(const maze_t &maze)
      : num_food_(maze.num_food_),
        num_pills_(maze.num_pills_) {
        cells_ = new unsigned char[NUM_CELLS];
        memcpy(cells_, maze.cells_, NUM_CELLS);
    }
    maze_t(maze_t &&maze)
      : num_food_(maze.num_food_),
        num_pills_(maze.num_pills_),
        cells_(maze.cells_) {
        maze.cells_ = 0;
    }
    virtual ~maze_t() {
        delete[] cells_;
    }

    static void set_static_members() {
        unsigned char walls[] = // 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 (this map grows downward (i.e. it looks inverted but it isn't))
                                 { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   // row  1
                                   0, 2, 2, 2, 2, 2, 2, 0, 2, 0, 2, 2, 2, 2, 2, 2, 0,   // row  2
                                   0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 0,   // row  3
                                   2, 0, 2, 0, 2, 0, 2, 2, 2, 2, 2, 0, 2, 0, 2, 0, 2,   // row  4
                                   3, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 3,   // row  5
                                   0, 2, 2, 0, 2, 2, 2, 0, 2, 0, 2, 2, 2, 0, 2, 2, 0,   // row  6
                                   0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0,   // row  7
                                   2, 2, 2, 0, 2, 0, 2, 2, 2, 2, 2, 0, 2, 0, 2, 2, 2,   // row  8
                                   2, 2, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 2, 2,   // row  9
                                   2, 2, 2, 0, 2, 0, 2, 2, 2, 2, 2, 0, 2, 0, 2, 2, 2,   // row 10
                                   0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0,   // row 11
                                   2, 2, 2, 0, 2, 0, 2, 2, 2, 2, 2, 0, 2, 0, 2, 2, 2,   // row 12
                                   2, 2, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 2, 2,   // row 13
                                   2, 2, 2, 0, 2, 2, 2, 0, 2, 0, 2, 2, 2, 0, 2, 2, 2,   // row 14
                                   0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 0,   // row 15
                                   0, 2, 2, 0, 2, 0, 2, 2, 2, 2, 2, 0, 2, 0, 2, 2, 0,   // row 16
                                   3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3,   // row 17
                                   0, 2, 2, 0, 2, 2, 2, 0, 2, 0, 2, 2, 2, 0, 2, 2, 0,   // row 18
                                   0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0 }; // row 19
        memcpy(walls_, walls, NUM_CELLS);

        // set cell maps
        num_valid_cells_ = 0;
        for( int cell = 0; cell < NUM_CELLS; ++cell ) {
            cell_map_[cell] = -1;
            inv_cell_map_[cell] = -1;
            if( walls_[cell] != WALL ) {
                cell_map_[cell] = num_valid_cells_;
                inv_cell_map_[num_valid_cells_] = cell;
                ++num_valid_cells_;
            }
        }

        // set cell map for initial position for ghosts (special case as this is a wall position)
        int cell = loc_t(8, 11).as_integer();
        cell_map_[cell] = num_valid_cells_;
        inv_cell_map_[num_valid_cells_] = cell;

        std::cout << "maze_t: #cells=" << NUM_CELLS << ", #valid-cells=" << num_valid_cells_ << std::endl;
    }

    bool valid(const loc_t &loc) const {
        int status = cells_[loc.as_integer()];
        return status != WALL;
    }
    bool can_move_north(const loc_t &loc) const {
        return (1 + loc.row_ < NUM_ROWS) && valid(loc_t(loc.col_, 1 + loc.row_));
    }
    bool can_move_east(const loc_t &loc) const {
        return ((1 + loc.col_ < NUM_COLS) && valid(loc_t(1 + loc.col_, loc.row_))) || ((loc.col_ == 16) && (loc.row_ == 10));
    }
    bool can_move_south(const loc_t &loc) const {
        return (loc.row_ > 0) && valid(loc_t(loc.col_, loc.row_ - 1));
    }
    bool can_move_west(const loc_t &loc) const {
        return ((loc.col_ > 0) && valid(loc_t(loc.col_ - 1, loc.row_))) || ((loc.col_ == 0) && (loc.row_ == 10));
    }
    bool can_move(const loc_t &loc, int dir) const {
        if( dir == loc_t::NORTH )
            return can_move_north(loc);
        else if( dir == loc_t::EAST )
            return can_move_east(loc);
        else if( dir == loc_t::SOUTH )
            return can_move_south(loc);
        else
            return can_move_west(loc);
    }
    int food_in_line_of_sight(const loc_t &loc, int dir) const {
        int food = 0;
        int row = loc.row_;
        int col = loc.col_;
        if( (dir == loc_t::NORTH) || (dir == loc_t::SOUTH) )
            row += dir == loc_t::NORTH ? 1 : -1;
        else
            col += dir == loc_t::EAST ? 1 : -1;

        while( (row >= 0) && (row < NUM_ROWS) && (col >= 0) && (col < NUM_COLS) && (cells_[row * NUM_COLS + col] != WALL) ) {
            food += cells_[row * NUM_COLS + col] == FOOD ? 1 : 0;
            if( (dir == loc_t::NORTH) || (dir == loc_t::SOUTH) )
                row += dir == loc_t::NORTH ? 1 : -1;
            else
                col += dir == loc_t::EAST ? 1 : -1;
        }
        return food;
    }

    bool food_at(const loc_t &loc) const {
        return cells_[loc.as_integer()] == FOOD;
    }
    bool pill_at(const loc_t &loc) const {
        return cells_[loc.as_integer()] == PILL;
    }

    void eat_food(const loc_t &loc) {
        assert(food_at(loc));
        cells_[loc.as_integer()] = FREE;
        --num_food_;
    }
    void eat_pill(const loc_t &loc) {
        assert(pill_at(loc));
        cells_[loc.as_integer()] = FREE;
        --num_pills_;
    }

    void print(std::ostream &os) const {
        os << "+----+-----------------+----+" << std::endl;
        os << "|    |12345678901234567|    |" << std::endl;
        os << "+----+-----------------+----+" << std::endl;
        for( int row = NUM_ROWS - 1; row >= 0; --row ) {
            os << "| " << std::setw(2) << 1 + row << " " << (row == 10 ? ' ' : '|');
            for( int col = 0; col < NUM_COLS; ++col ) {
                int status = cells_[row * NUM_COLS + col];
                if( status == FOOD )
                    os << '.';
                else if( status == WALL )
                    os << '#';
                else if( status == PILL )
                    os << '*';
                else
                    os << ' ';
            }
            os << (row == 10 ? ' ' : '|') << ' ' << 1 + row << (row < 9 ? "  |" : " |") << std::endl;
        }
        os << "+----+-----------------+----+" << std::endl;
        os << "|    |12345678901234567|    |" << std::endl;
        os << "+----+-----------------+----+" << std::endl;
    }
};

inline std::ostream& operator<<(std::ostream &os, const maze_t &maze) {
    maze.print(os);
    return os;
}

inline void loc_t::move_north(const maze_t &maze) {
    assert(maze.can_move_north(*this));
    ++row_;
}

inline void loc_t::move_east(const maze_t &maze) {
    assert(maze.can_move_east(*this));
    if( col_ == 16 )
        col_ = 0;
    else
        ++col_;
}

inline void loc_t::move_south(const maze_t &maze) {
    assert(maze.can_move_south(*this));
    --row_;
}

inline void loc_t::move_west(const maze_t &maze) {
    assert(maze.can_move_west(*this));
    if( col_ == 0 )
        col_ = 16;
    else
        --col_;
}

struct state_t {
    int ghosts_loc_;
    int ghosts_last_dir_;
    loc_t pacman_;
    maze_t maze_;

    state_t()
      : ghosts_loc_(0),
        ghosts_last_dir_(0),
        pacman_(8, 8) {
        std::pair<int, int> ghost_loc_and_dir[4];
        for( int i = 0; i < 4; ++i )
            ghost_loc_and_dir[i] = std::make_pair(loc_t(8, 11).as_integer(), 4); // dummy initial dir = 4 (regular values are 0 ... 3)
        std::pair<int, int> p = encode_ghost_loc_and_dir(ghost_loc_and_dir);
        ghosts_loc_ = p.first;
        ghosts_last_dir_ = p.second;
        std::pair<int, int> tmp[4];
        decode_ghost_loc_and_dir(tmp);
        assert(tmp[0] == ghost_loc_and_dir[0]);
        assert(tmp[1] == ghost_loc_and_dir[1]);
        assert(tmp[2] == ghost_loc_and_dir[2]);
        assert(tmp[3] == ghost_loc_and_dir[3]);
    }
    state_t(const state_t &state)
      : ghosts_loc_(state.ghosts_loc_),
        ghosts_last_dir_(state.ghosts_last_dir_),
        pacman_(state.pacman_),
        maze_(state.maze_) {
    }
    state_t(state_t &&state) = default;
    virtual ~state_t() {
    }

    std::pair<int, int> encode_ghost_loc_and_dir(const std::pair<int, int> *ghost_loc_and_dir) const {
        std::pair<int, int> p(0, 0);
        for( int i = 0; i < 4; ++i ) {
            int cell = ghost_loc_and_dir[i].first;
            int dir = ghost_loc_and_dir[i].second;
            assert((cell >= 0) && (cell < NUM_CELLS));
            assert((maze_t::cell_map_[cell] >= 0) && (maze_t::cell_map_[cell] <= maze_t::num_valid_cells_));
            assert((dir >= 0) && (dir < 5));
            p.first = p.first * (1 + maze_t::num_valid_cells_) + maze_t::cell_map_[cell];
            p.second = p.second * 5 + dir;
        }
        assert((p.first >= 0) && (p.second >= 0));
        return p;
    }
    void decode_ghost_loc_and_dir(std::pair<int, int> *ghost_loc_and_dir) const {
        int ghost_loc = ghosts_loc_;
        int ghost_dir = ghosts_last_dir_;
        for( int i = 0; i < 4; ++i ) {
            int loc = ghost_loc % (1 + maze_t::num_valid_cells_);
            int dir = ghost_dir % 5;
            ghost_loc /= 1 + maze_t::num_valid_cells_;;
            ghost_dir /= 5;
            assert(maze_t::inv_cell_map_[loc] != -1);
            ghost_loc_and_dir[3 - i].first = maze_t::inv_cell_map_[loc];
            ghost_loc_and_dir[3 - i].second = dir;
        }
    }

    void combine_ghost_movements(const std::vector<std::vector<std::pair<std::pair<int, int>, float> > > &ghost_outcomes, std::vector<std::pair<std::pair<int, int>, float> > &outcomes) const {
        assert(ghost_outcomes.size() == 4);
        int n = 1;
        for( int i = 0; i < 4; ++i )
            n *= ghost_outcomes[i].size();
        assert((n > 0) && (n <= 81));
        //std::cout << "n=" << n << std::endl;

        outcomes.clear();
        outcomes.reserve(n);
        for( int i = 0; i < n; ++i ) {
            int g_indices[4];
            int index = i;
            for( int j = 3; j > 0; --j ) {
                g_indices[j] = index % ghost_outcomes[j].size();
                index /= ghost_outcomes[j].size();
            }
            g_indices[0] = index;

            float probability = 1;
            std::pair<int, int> ghost_loc_and_dir[4];
            for( int j = 0; j < 4; ++j ) {
                ghost_loc_and_dir[j] = ghost_outcomes[j][g_indices[j]].first;
                probability *= ghost_outcomes[j][g_indices[j]].second;
            }
            outcomes.push_back(std::make_pair(encode_ghost_loc_and_dir(ghost_loc_and_dir), probability));
        }
    }

    void move_ghosts(std::vector<std::pair<std::pair<int, int>, float> > &outcomes) const {
        std::pair<int, int> ghost_loc_and_dir[4];
        decode_ghost_loc_and_dir(ghost_loc_and_dir);

        std::vector<std::vector<std::pair<std::pair<int, int>, float> > > ghost_outcomes(4);
        for( int i = 0; i < 4; ++i ) {
            const loc_t loc(ghost_loc_and_dir[i].first);
            int forbidden_dir = (ghost_loc_and_dir[i].second + 2) % 4;
            //std::cout << "ghost[i=" << i << "]: loc=" << loc << ", dir=" << ghost_loc_and_dir[i].second << ", forbidden=" << forbidden_dir << std::endl;
           
            int num_targets = 0;
            float total_mass = 0;
            float target_prob[4];
            std::pair<int, int> target[4];
            for( int dir = 0; dir < 4; ++dir ) {
                if( (forbidden_dir != dir) && maze_.can_move(loc, dir) ) {
                    target[num_targets] = std::make_pair(loc.move(maze_, dir).as_integer(), dir);
                    target_prob[num_targets] = maze_.food_in_line_of_sight(loc, dir);
                    //std::cout << "dir=" << dir << ", can-move=" << maze_.can_move(loc, dir) << ", loc=" << loc_t(target[num_targets].first) << ", food=" << target_prob[num_targets] << std::endl;
                    total_mass += target_prob[num_targets];
                    ++num_targets;
                }
            }
            assert(num_targets > 0);
            //std::cout << "num-targets=" << num_targets << ", mass=" << total_mass << std::endl;

            if( total_mass == 0 ) {
                for( int j = 0; j < num_targets; ++j )
                    target_prob[j] = 1 / float(num_targets);
            } else {
                for( int j = 0; j < num_targets; ++j ) {
                    while( (target_prob[j] == 0) && (j < num_targets) ) {
                        target[j] = target[num_targets - 1];
                        target_prob[j] = target_prob[num_targets - 1];
                        --num_targets;
                    }
                }
                for( int j = 0; j < num_targets; ++j ) {
                    assert(target_prob[j] != 0);
                    target_prob[j] /= total_mass;
                }
            }
            assert(num_targets > 0);

            ghost_outcomes[i].reserve(num_targets);
            for( int j = 0; j < num_targets; ++j )
                ghost_outcomes[i].push_back(std::make_pair(target[j], target_prob[j]));
        }

        combine_ghost_movements(ghost_outcomes, outcomes);
    }

    void move_ghosts() { // non-deterministically move ghosts
        std::vector<std::pair<std::pair<int, int>, float> > outcomes;
        move_ghosts(outcomes);
        std::vector<float> cdf(outcomes.size(), 0);
        cdf[0] = outcomes[0].second;
        for( int i = 1; i < outcomes.size(); ++i )
            cdf[i] = cdf[i - 1] + outcomes[i].second;
        int sampled_index = Random::sample_from_distribution(cdf.size(), &cdf[0]);
        ghosts_loc_ = outcomes[sampled_index].first.first;
        ghosts_last_dir_ = outcomes[sampled_index].first.second;
    }

    void print(std::ostream &os) const {
        os << "(pacman=" << pacman_ << ", ghosts={";
        std::pair<int, int> ghost_loc_and_dir[4];
        decode_ghost_loc_and_dir(ghost_loc_and_dir);
        for( int i = 0; i < 4; ++i ) {
            os << loc_t(ghost_loc_and_dir[i].first) << ":" << ghost_loc_and_dir[i].second;
            if( i + 1 < 4 ) os << ",";
        }
        os << "}, food=" << maze_.num_food_
           << ", pills=" << maze_.num_pills_
           << ")"
           << std::endl;
        os << maze_;
    }
};

inline std::ostream& operator<<(std::ostream &os, const state_t &state) {
    state.print(os);
    return os;
}








#if 0
struct beam_t {
    Bitmap::bitmap_t bitmap_;

    struct const_iterator : public Bitmap::bitmap_t::const_iterator {
        const_iterator(const Bitmap::bitmap_t::const_iterator &it) : Bitmap::bitmap_t::const_iterator(it) { }
    };

    beam_t() {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "beam_t: ctor called" << std::endl;
#endif
    }
    beam_t(const Bitmap::bitmap_t &bitmap) : bitmap_(bitmap) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "beam_t: copy ctor called" << std::endl;
#endif
    }
    beam_t(Bitmap::bitmap_t &&bitmap) : bitmap_(std::move(bitmap)) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "beam_t: move ctor called" << std::endl;
#endif
    }
    beam_t(const Bitmap::bitmap_t &bitmap, const Bitmap::bitmap_t &mask)
      : bitmap_(bitmap, mask) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "beam_t: copy ctor w/ mask called" << std::endl;
#endif
    }
    virtual ~beam_t() {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "beam_t: dtor called" << std::endl;
#endif
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
    static std::vector<Bitmap::bitmap_t> action_mask_;

  public:
    belief_state_t(int hidden = 0) : hidden_(hidden) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: ctor called" << std::endl;
#endif
    }
    belief_state_t(const belief_state_t &bel)
      : beam_(bel.beam_),
        hidden_(bel.hidden_) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: copy ctor called" << std::endl;
#endif
    }
    belief_state_t(belief_state_t &&bel)
      : beam_(std::move(bel.beam_)),
        hidden_(bel.hidden_) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: move ctor called" << std::endl;
#endif
    }
    belief_state_t(const Bitmap::bitmap_t &bitmap, const Bitmap::bitmap_t &mask, int hidden)
      : beam_(bitmap, mask),
        hidden_(hidden) {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: copy ctor w/ mask called" << std::endl;
#endif
    }
    ~belief_state_t() {
#ifdef DEBUG_CTOR_DTOR
        std::cout << "belief_state_t: dtor called" << std::endl;
#endif
    }

    static void set_bitmap_mask(int dim) {
        dim_ = dim;
        Bitmap::bitmap_t lower(0);
        action_mask_.reserve(2 * (1 + dim_));
        for( int i = 0; i <= dim_; ++i ) {
            Bitmap::bitmap_t upper(lower);
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

    int value(int vid) const {
        assert(vid == 0);
        int v = cardinality() > 1 ? -1 : *beam_.begin();
        return v;
    }
    void fill_values_for_variable(int vid, std::vector<float> &probabilities) const {
        assert(vid == 0);
        probabilities = std::vector<float>(dim_, 0);
        float p = 1.0 / float(cardinality());
        for( beam_t::const_iterator it = beam_.begin(); it != beam_.end(); ++it ) {
            assert((*it >= 0) && (*it < probabilities.size()));
            probabilities[*it] = p;
        }
    }
    void fill_values_for_variable(int vid, std::vector<std::pair<int, float> > &values) const {
        assert(vid == 0);
        values.clear();
        values.reserve(cardinality());
        float p = 1.0 / float(cardinality());
        for( beam_t::const_iterator it = beam_.begin(); it != beam_.end(); ++it )
            values.push_back(std::make_pair(*it, p));
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

#if 0 // CHECK
struct feature_t : public POMDP::feature_t<belief_state_t> {
    feature_t(const belief_state_t &bel) {
#ifdef DEBUG
        std::cout << "bel=" << bel << std::endl;
        std::cout << "marginal:";
#endif
        marginals_ = std::vector<std::vector<float> >(1);
        marginals_[0] = std::vector<float>(Bitmap::bitmap_t::dim_, 0);
        float p = 1.0 / bel.cardinality();
        for( beam_t::const_iterator it = bel.beam(0).begin(); it != bel.beam(0).end(); ++it ) {
            assert((it.value() >= 0) && (it.value() < Bitmap::bitmap_t::dim_));
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
#endif

class pomdp_t : public POMDP::pomdp_t<belief_state_t> {
  protected:
    int dim_;

    int number_actions_;
    int number_variables_;
    int number_beams_;

    //std::vector<POMDP::pomdp_t<belief_state_t>::varset_t> varsets_; // CHECK

    mutable belief_state_t init_tmp_;

  public:
    pomdp_t(int dim) : POMDP::pomdp_t<belief_state_t>(DISCOUNT), dim_(dim) {
        number_actions_ = 1 + dim_;
        number_variables_ = 1;
        number_beams_ = 1;
#if 0 // CHECK
        POMDP::pomdp_t<belief_state_t>::varset_t varset;
        varset.push_back(0);
        varsets_.push_back(varset);
#endif
    }
    virtual ~pomdp_t() { }

    virtual Problem::action_t number_actions(const belief_state_t &bel) const {
        return number_actions_;
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
        return number_actions_;
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
    virtual int number_beams() const {
        return number_beams_;
    }
    virtual int number_variables() const {
        return number_variables_;
    }
    virtual int number_determined_variables() const {
        return 0;
    }

    virtual bool determined(int vid) const {
        assert(vid == 0);
        return false;
    }
    virtual int domain_size(int vid) const {
        assert(vid == 0);
        return dim_;
    }
    virtual int value(const belief_state_t &belief, int vid) const {
        assert(vid == 0);
        return belief.value(vid);
    }
    virtual void fill_values_for_variable(const belief_state_t &belief, int vid, std::vector<float> &probabilities) const {
        assert(vid == 0);
        belief.fill_values_for_variable(vid, probabilities);
    }
    virtual void fill_values_for_variable(const belief_state_t &belief, int vid, std::vector<std::pair<int, float> > &values) const {
        assert(vid == 0);
        belief.fill_values_for_variable(vid, values);
    }

#if 0 // CHECK
    virtual const POMDP::pomdp_t<belief_state_t>::varset_t& varset(int bid) const {
        return varsets_[0];
    }
    virtual int cardinality(const belief_state_t &bel) const {
        return bel.cardinality();
    }
    virtual POMDP::feature_t<belief_state_t> *get_feature(const belief_state_t &bel) const {
        return new feature_t(bel); // CHECK
    }
    virtual void remove_feature(const POMDP::feature_t<belief_state_t> *feature) const {
        delete feature;
    }
#endif

    virtual void apply_action(belief_state_t &bel_a, Problem::action_t a) const { /* real work is done below */ }
    virtual void apply_obs(belief_state_t &bel_ao, Problem::action_t a, POMDP::observation_t obs) const {
        belief_state_t nbel = bel_ao.apply(a, obs);
        bel_ao = nbel;
    }
    virtual POMDP::observation_t sample_observation_using_hidden_state(const belief_state_t &bel, const belief_state_t &bel_a, Problem::action_t a) const {
        return bel.hidden_ < a ? 0 : 1;
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
#endif

#undef DEBUG

