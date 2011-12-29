#include <cassert>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>
#include <limits>
#include <set>

#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "dispatcher.h"

#define DISCOUNT 1.00

struct effect_t {
    std::vector<std::pair<int, int> > condition_;
    std::vector<std::pair<int, int> > effect_;
};

struct action_t {
    std::vector<std::pair<int, int> > precondition_;
    std::vector<effect_t> effects_;
};

struct sample_t {
    char *vars_;
    static unsigned dim_;

    sample_t() { vars_ = new char[dim_]; }
    sample_t(const sample_t &s) {
        vars_ = new char[dim_];
        memcpy(vars_, s.vars_, dim_);
    }
    ~sample_t() { delete[] vars_; }

    static void initialize(unsigned dim) {
        dim_ = dim;
    }

    bool holds(int var, int value) const {
        return vars_[var] == value;
    }
    bool holds(const std::vector<std::pair<int, int> > &condition) const {
        for( unsigned i = 0, isz = condition.size(); i < isz; ++i ) {
            if( !holds(condition[i].first, condition[i].second) ) return false;
        }
        return true;
    }

    void apply(int var, int value) {
        if( vars_[var] != -1 )
            vars_[var] = value;
    }
    void apply(const action_t &a, std::set<sample_t*> &container) const {
        sample_t *sample = new sample_t(*this);
        for( unsigned i = 0, isz = a.effects_.size(); i < isz; ++i ) {
            const effect_t &eff = a.effects_[i];
            if( holds(eff.condition_) ) {
                for( unsigned j = 0, jsz = eff.effect_.size(); j < jsz; ++j ) {
                    sample->apply(eff.effect_[j].first, eff.effect_[j].second);
                }
            }
        }
        container.insert(sample);
    }

    void print(std::ostream &os) const {
        os << "(";
        for( unsigned i = 0; i < dim_; ++i ) {
            os << "V" << i << "=" << vars_[i] << ",";
        }
        os << ")";
    }
};

inline std::ostream& operator<<(std::ostream &os, const sample_t &s) {
    s.print(os);
    return os;
}

struct belief_component_t {
    typedef std::set<sample_t*> container_type;
    container_type *samples_;
   
    belief_component_t() {
        samples_ = new container_type;
    }
    ~belief_component_t() {
        clear_samples();
        delete samples_;
    }

    void clear_samples() {
        for( container_type::iterator it = samples_->begin(); it != samples_->end(); ++it )
            delete *it;
        samples_->clear();
    }

    void insert_sample(const sample_t &sample) {
        sample_t *s = new sample_t(sample);
        samples_->insert(s);
    }

    bool holds(int var, int value) const {
        for( container_type::iterator it = samples_->begin(); it != samples_->end(); ++it ) {
            if( !(*it)->holds(var, value) ) return false;
        }
        return true;
    }

    void apply(const action_t &a, belief_component_t &bc) {
        bc.clear_samples();
        for( container_type::iterator it = samples_->begin(); it != samples_->end(); ++it ) {
            (*it)->apply(a, *bc.samples_);
        }
    }

    void print(std::ostream &os) const {
        os << "{";
        for( container_type::iterator it = samples_->begin(); it != samples_->end(); ++it ) {
            os << **it << ",";
        }
        os << "}";
    }
};

inline std::ostream& operator<<(std::ostream &os, const belief_component_t &bc) {
    bc.print(os);
    return os;
}

struct belief_t {
    std::vector<belief_component_t> components_;
    static unsigned dim_;

    belief_t() : components_(dim_) { }
    ~belief_t() { }

    static void initialize(unsigned dim) {
        dim_ = dim;
    }

    bool holds(int var, int value) const {
        assert(0);
        return false;
    }

    void apply(const action_t &a, belief_t &bel) {
        for( unsigned i = 0; i < dim_; ++i ) {
            components_[i].apply(a, bel.components_[i]);
        }
    }

    void print(std::ostream &os) const {
        for( unsigned i = 0; i < dim_; ++i ) {
            os << "comp[" << i << "]=" << components_[i] << std::endl;
        }
    }
};

inline std::ostream& operator<<(std::ostream &os, const belief_t &b) {
    b.print(os);
    return os;
}






#if 0


inline unsigned rotation(unsigned x) {
    return (x << 16) | (x >> 16);
}


template<typename T> struct bits_t {
    T field_;
    int base_;
    bits_t(const T field, int base) : field_(field), base_(base) { }
    void print(std::ostream &os) const {
        if( field_ != 0 ) {
            T aux = field_;
            for( int i = 0; aux != 0; ++i, aux = aux >> 1 ) {
                if( aux & 0x1 ) {
                    os << base_ + i << ",";
                }
            }
        }
    }
};

template<typename T> inline std::ostream& operator<<(std::ostream &os, const bits_t<T> &bits) {
    bits.print(os);
    return os;
}



struct state_info_t {
    unsigned known_[WORDS_FOR_EDGES];
    unsigned blocked_[WORDS_FOR_EDGES];
    static int words_for_edges_;

    state_info_t() {
        memset(known_, 0, WORDS_FOR_EDGES * sizeof(unsigned));
        memset(blocked_, 0, WORDS_FOR_EDGES * sizeof(unsigned));
    }
    ~state_info_t() { }

    static void initialize(int words_for_edges) {
        words_for_edges_ = words_for_edges;
    }

    size_t hash() const {
        unsigned rv = 0;
        for( int i = 0; i < words_for_edges_; ++i )
            rv = rv ^ known_[i];
        for( int i = 0; i < words_for_edges_; ++i )
            rv = rv ^ rotation(blocked_[i]);
        return rv;
    }

    void clear() {
        memset(known_, 0, words_for_edges_ * sizeof(unsigned));
        memset(blocked_, 0, words_for_edges_ * sizeof(unsigned));
    }

    bool known(int edge) const {
        int n = edge >> 5, offset = edge & 0x1F;
        int mask = 1 << offset;
        return known_[n] & mask;
    }
    bool traversable(int edge) const {
        int n = edge >> 5, offset = edge & 0x1F;
        int mask = 1 << offset;
        return (blocked_[n] & mask) != 0 ? false : true;
    }
    void set_edge_status(int edge, bool blocked) {
        int n = edge >> 5, offset = edge & 0x1F;
        int mask = 1 << offset;
        known_[n] |= mask;
        if( blocked )
            blocked_[n] |= mask;
        else
            blocked_[n] &= ~mask;
    }

    const state_info_t& operator=(const state_info_t &info) {
        memcpy(known_, info.known_, words_for_edges_ * sizeof(unsigned));
        memcpy(blocked_, info.blocked_, words_for_edges_ * sizeof(unsigned));
        return *this;
    }

    bool operator==(const state_info_t &info) const {
        return (memcmp(known_, info.known_, words_for_edges_ * sizeof(unsigned)) == 0) &&
               (memcmp(blocked_, info.blocked_, words_for_edges_ * sizeof(unsigned)) == 0);
    }
    bool operator!=(const state_info_t &info) const {
        return *this == info ? false : true;
    }
    bool operator<(const state_info_t &info) const {
        return (memcmp(known_, info.known_, words_for_edges_ * sizeof(unsigned)) < 0) ||
               ((memcmp(known_, info.known_, words_for_edges_ * sizeof(unsigned)) == 0) &&
                (memcmp(blocked_, info.blocked_, words_for_edges_ * sizeof(unsigned)) < 0));
    }

    void print(std::ostream &os) const {
        os << "K={";
        for( int i = 0; i < words_for_edges_; ++i )
            os << bits_t<unsigned>(known_[i], i << 5);
        os << "},B={";
        for( int i = 0; i < words_for_edges_; ++i )
            os << bits_t<unsigned>(blocked_[i], i << 5);
        os << "}";
    }
};

inline std::ostream& operator<<(std::ostream &os, const state_info_t &info) {
    info.print(os);
    return os;
}

struct state_t {
    int current_;
    state_info_t info_;
    unsigned visited_[WORDS_FOR_NODES];
    mutable bool shared_;
    //mutable const int *distances_;
    mutable int *distances_;
    mutable int heuristic_;

    static const CTP::graph_t *graph_;
    static int num_nodes_;
    static int num_edges_;
    static int words_for_nodes_;
    static int words_for_edges_;
    static shortest_path_cache_t cache_;
    static bool use_cache_;

  public:
    state_t(int current = -1)
      : current_(current), shared_(true), distances_(0), heuristic_(-1) {
        memset(visited_, 0, WORDS_FOR_NODES * sizeof(unsigned));
    }
    state_t(const state_t &s)
      : shared_(true), distances_(0) {
        *this = s;
    }
    ~state_t() {
        if( !shared_ ) delete[] distances_;
    }

    static void initialize(const CTP::graph_t &graph, bool use_cache, int cache_capacity) {
        graph_ = &graph;
        num_nodes_ = graph.num_nodes_;
        num_edges_ = graph.num_edges_;

        if( num_nodes_ > (int)(WORDS_FOR_NODES * sizeof(unsigned) * 8) ) {
            std::cout << "error: number of nodes must be <= "
                      <<  WORDS_FOR_NODES * sizeof(unsigned) * 8
                      << std::endl;
            exit(1);
        }

        if( num_edges_ > (int)(WORDS_FOR_EDGES * sizeof(unsigned) * 8) ) {
            std::cout << "error: number of nodes must be <= "
                      <<  WORDS_FOR_EDGES * sizeof(unsigned) * 8
                      << std::endl;
            exit(1);
        }

        words_for_nodes_ = num_nodes_ >> 5;
        if( (num_nodes_ & 0x1F) != 0 ) ++words_for_nodes_;
        words_for_edges_ = num_edges_ >> 5;
        if( (num_edges_ & 0x1F) != 0 ) ++words_for_edges_;
        std::cout << "init: #nodes=" << num_nodes_
                  << ", #edges=" << num_edges_
                  << ", #words-for-nodes=" << words_for_nodes_
                  << ", #words-for-edges=" << words_for_edges_
                  << std::endl;
        state_info_t::initialize(words_for_edges_);
        use_cache_ = use_cache;
        cache_.initialize(num_nodes_, cache_capacity);
    }

    static void clear_cache() {
        cache_.clear();
    }

    static void print_stats(std::ostream &os) {
        cache_.print_stats(os);
    }

    size_t hash() const { return info_.hash(); }

    void compute_heuristic() const {
        if( heuristic_ == -1 ) {
            heuristic_ = graph_->bfs(current_ == -1 ? 0 : current_, num_nodes_ - 1,
                                     info_.known_, info_.blocked_, true);
        }
    }

    bool known(int edge) const { return info_.known(edge); }
    bool traversable(int edge) const { return info_.traversable(edge); }
    bool visited(int node) const {
        int n = node >> 5, offset = node & 0x1F;
        int mask = 1 << offset;
        return visited_[n] & mask;
    }
    bool reachable(int node) const {
        preprocess();
        return distances_[node] < INT_MAX;
    }
    int distance_to(int node) const {
        preprocess();
        return distances_[node];
    }
    bool perimeter(int node) const {
        return !visited(node) && reachable(node);
    }
    bool is_dead_end() const {
        compute_heuristic();
        return (current_ != -1) && (heuristic_ == INT_MAX);
    }

    void set_edge_status(int edge, bool blocked) {
        info_.set_edge_status(edge, blocked);
    }
    void move_to(int node) {
        int n = node >> 5, offset = node & 0x1F;
        int mask = 1 << offset;
        current_ = node;
        visited_[n] |= mask;
    }

    void preprocess() const {
        if( distances_ == 0 ) {
            assert(shared_);
            std::pair<bool, const int*> p(false, 0);
            if( use_cache_ ) p = cache_.lookup(*graph_, current_, info_);
            if( p.first ) {
                if( !shared_ ) delete[] distances_;
                distances_ = const_cast<int*>(p.second);
                shared_ = true;
            } else {
                if( shared_ ) distances_ = new int[num_nodes_];
                assert(distances_ != 0);
                shared_ = false;
                if( p.second ) {
                    memcpy(const_cast<int*>(distances_), p.second, num_nodes_ * sizeof(int));
                } else {
                    graph_->dijkstra(current_, distances_, info_.known_, info_.blocked_, false);
                    //distances_[current_] = graph_->bfs(current_, num_nodes_ - 1, info_.known_, info_.blocked_, true);
                }
            }
            //heuristic_ = distances_[current_];
        }
    }

    void clear() {
        current_ = 0;
        info_.clear();
        memset(visited_, 0, words_for_nodes_ * sizeof(unsigned));
        if( !shared_ ) {
            delete[] distances_;
            shared_ = true;
        }
        distances_ = 0;
        heuristic_ = -1;
    }

    const state_t& operator=(const state_t &s) {
        current_ = s.current_;
        info_ = s.info_;
        memcpy(visited_, s.visited_, words_for_nodes_ * sizeof(unsigned));
        if( s.shared_ ) {
            if( !shared_ ) delete[] distances_;
            distances_ = s.distances_;
            shared_ = true;
        } else {
            if( shared_ ) {
                distances_ = new int[num_nodes_];
                shared_ = false;
            }
            assert(distances_ != 0);
            memcpy(const_cast<int*>(distances_), s.distances_, num_nodes_ * sizeof(int));
        }
        heuristic_ = s.heuristic_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return (current_ == s.current_) &&
               (info_ == s.info_) &&
               (memcmp(visited_, s.visited_, words_for_nodes_ * sizeof(unsigned)) == 0);
    }
    bool operator!=(const state_t &s) const {
        return *this == s ? false : true;
    }
    bool operator<(const state_t &s) const {
        return (current_ < s.current_) ||
               ((current_ == s.current_) && (info_ < s.info_)) ||
               ((current_ == s.current_) && (info_ == s.info_) &&
                (memcmp(visited_, s.visited_, words_for_nodes_ * sizeof(unsigned)) < 0));
    }
    void print(std::ostream &os) const {
        os << "(" << current_ << "," << info_ << ",V={";
        for( int i = 0; i < words_for_nodes_; ++i )
            os << bits_t<unsigned>(visited_[i], i << 5);
        os << "}";
        if( is_dead_end() ) os << ",DEAD";
        os << ")";
    }
};

inline std::ostream& operator<<(std::ostream &os, const state_t &s) {
    s.print(os);
    return os;
}

class problem_t : public Problem::problem_t<state_t> {
  public:
    const CTP::graph_t &graph_;
    const state_t init_;
    int start_, goal_;
    mutable int max_branching_;
    bool use_cache_;
    mutable next_cache_t next_cache_;

  public:
    problem_t(CTP::graph_t &graph, bool use_cache = false, unsigned cache_size = (int)1e4)
      : Problem::problem_t<state_t>(DISCOUNT, (int)1e3), // change dead_end_value
        graph_(graph), init_(-1), start_(0), goal_(graph_.num_nodes_ - 1),
        max_branching_(0), use_cache_(use_cache) {
        next_cache_.initialize(graph.num_nodes_, cache_size);
    }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions(const state_t &s) const {
        return s.current_ == -1 ? 1 : graph_.num_nodes_;
    }
    virtual bool applicable(const state_t &s, Problem::action_t a) const {
        return ((s.current_ == -1) && (a == 0)) ||
               ((s.current_ != -1) && s.perimeter(a));
    }
    virtual const state_t& init() const { return init_; }
    virtual bool terminal(const state_t &s) const {
        return s.current_ == goal_;
    }
    virtual bool dead_end(const state_t &s) const {
        return s.is_dead_end();
    }
    virtual float cost(const state_t &s, Problem::action_t a) const {
        return s.current_ == -1 ? 0 : s.distance_to(a);
    }
    virtual void next(const state_t &s,
                      Problem::action_t a,
                      std::vector<std::pair<state_t, float> > &outcomes) const {

        if( use_cache_ ) {
            const std::vector<std::pair<state_t, float> > *ptr = next_cache_.lookup(s, a);
            if( ptr != 0 ) {
                outcomes = *ptr;
                return;
            }
        }

        //std::cout << "next" << s << " w/ a=" << a << " is:" << std::endl;

        ++expansions_;

        int to_node = -1;
        if( s.current_ == -1 ) {
            to_node = start_;
        } else {
            to_node = a;
            assert(to_node != s.current_);
        }

        // collect edges adjacent at to_node of unknown status
        int k = 0;
        std::vector<int> unknown_edges;
        unknown_edges.reserve(graph_.num_edges_);
        for( int i = 0, isz = graph_.at_[to_node].size(); i < isz; ++i ) {
            int e = graph_.at_[to_node][i];
            if( !s.known(e) ) {
                unknown_edges.push_back(e);
                ++k;
            }
        }
        max_branching_ = (1<<k) > max_branching_ ? (1<<k) : max_branching_;

        // generate subsets of unknowns edges and update weathers
        outcomes.clear();
        outcomes.reserve(1 << k);
        for( int i = 0, isz = 1 << k; i < isz; ++i ) {
            state_t next(s);
            float p = 1;
            int subset = i;
            for( int j = 0; j < k; ++j ) {
                int e = unknown_edges[j];
                p *= (subset & 1) ? 1 - graph_.prob(e) : graph_.prob(e);
                next.info_.set_edge_status(e, subset & 1);
                subset = subset >> 1;
            }
            next.move_to(to_node);
            next.heuristic_ = -1;
            if( next.shared_ ) {
                next.distances_ = 0;
            } else {
                assert(next.distances_ != 0);
                delete[] next.distances_;
                next.distances_ = 0;
                next.shared_ = true;
            }
            assert(next.distances_ == 0);
            if( p > 0 ) {
                //next.preprocess();
                outcomes.push_back(std::make_pair(next, p));
                //std::cout << "    " << next << " w.p. " << p << std::endl;
            }
        }

        if( use_cache_ ) next_cache_.insert(s, a, outcomes);
    }
    virtual void print(std::ostream &os) const { }

    void print_stats(std::ostream &os) {
        if( use_cache_ ) next_cache_.print_stats(os);
    }
};

#endif

