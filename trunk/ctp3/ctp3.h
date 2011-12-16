#include <cassert>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>
#include <limits.h>
#include <tr1/unordered_map>

#include "graph.h"
#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "mcts.h"
#include "dispatcher.h"

#define DISCOUNT 1.00

#define WORDS_FOR_NODES 4 // max. 128 nodes
#define WORDS_FOR_EDGES 9 // max. 288 edges


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
        //std::cout << "hash of "; print(std::cout); std::cout << " = " << rv << std::endl;
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

int state_info_t::words_for_edges_ = 0;

inline std::ostream& operator<<(std::ostream &os, const state_info_t &info) {
    info.print(os);
    return os;
}


struct cache_functions_t {
    bool operator()(const state_info_t &info1, const state_info_t &info2) const {
        return info1 == info2;
    }
    size_t operator()(const state_info_t &info) const {
        return info.hash();
    }
};

class shortest_path_cache_t : public std::tr1::unordered_map<state_info_t, int*, cache_functions_t, cache_functions_t> {
  public:
    void print_stats(std::ostream &os) const {
        int maxsz = 0;
        for( int i = 0; i < (int)bucket_count(); ++i ) {
            int sz = bucket_size(i);
            maxsz = sz > maxsz ? sz : maxsz;
        }
        os << "#entries=" << size()
           << ", #buckets=" << bucket_count()
           << ", max-bucket-size=" << maxsz;
    }
};

class cache_t {
    int num_nodes_;
    int *distances_;
    shortest_path_cache_t **caches_;
    unsigned capacity_;
    unsigned size_;
    unsigned lookups_;
    unsigned hits_;

  public:
    cache_t() : num_nodes_(0), caches_(0), capacity_(0), size_(0), lookups_(0), hits_(0) { }
    ~cache_t() {
        for( int i = 0; i < num_nodes_; ++i )
            delete caches_[i];
        delete[] caches_;
    }

    void initialize(int num_nodes, unsigned capacity) {
        num_nodes_ = num_nodes;
        capacity_ = capacity;
        distances_ = new int[num_nodes_];
        caches_ = new shortest_path_cache_t*[num_nodes_];
        for( int i = 0; i < num_nodes_; ++i )
            caches_[i] = new shortest_path_cache_t;
    }

    std::pair<bool, const int*> lookup(const CTP::graph_t &graph, int source, const state_info_t &info) {
        ++lookups_;
        shortest_path_cache_t *cache = caches_[source];
        shortest_path_cache_t::const_iterator it = cache->find(info);
        if( it != cache->end() ) {
            ++hits_;
            return std::make_pair(true, it->second);
        } else {
            if( size_ < capacity_ ) {
                ++size_;
                int *distances = new int[num_nodes_];
                graph.dijkstra(source, distances, info.known_, info.blocked_, false);
                distances[source] = graph.bfs(source, num_nodes_ - 1, info.known_, info.blocked_, true);
                cache->insert(std::make_pair(info, distances));
                return std::make_pair(true, distances);
            } else {
                graph.dijkstra(source, distances_, info.known_, info.blocked_, false);
                distances_[source] = graph.bfs(source, num_nodes_ - 1, info.known_, info.blocked_, true);
                return std::make_pair(false, distances_);
            }
        }
    }

    unsigned lookups() const { return lookups_; }
    float hit_ratio() const { return (float)hits_ / (float)lookups_; }
    void print_stats(std::ostream &os) {
        os << "cache: #entries=" << size_
           << ", #lookups=" << lookups()
           << ", %hit=" << hit_ratio()
           << std::endl;
        for( int i = 0; i < num_nodes_; ++i ) {
            os << "  cache[" << i << "]: ";
            caches_[i]->print_stats(os);
            os << std::endl;
        }
    }
};

struct state_t {
    int current_;
    state_info_t info_;
    unsigned visited_[WORDS_FOR_NODES];
    mutable bool shared_;
    mutable const int *distances_;
    int heuristic_;

    static int num_nodes_;
    static int num_edges_;
    static int words_for_nodes_;
    static int words_for_edges_;
    static cache_t cache_;

  public:
    state_t(int current = -1)
      : current_(current), shared_(true), distances_(0), heuristic_(0) {
        memset(visited_, 0, WORDS_FOR_NODES * sizeof(unsigned));
    }
    state_t(const state_t &s) { *this = s; }
    ~state_t() {
        if( !shared_ ) delete[] distances_;
    }

    static void initialize(const CTP::graph_t &graph, int cache_capacity) {
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
        cache_.initialize(num_nodes_, cache_capacity);
    }

    static void print_stats(std::ostream &os) {
        cache_.print_stats(os);
    }

    size_t hash() const { return info_.hash(); }

    bool known(int edge) const { return info_.known(edge); }
    bool traversable(int edge) const { return info_.traversable(edge); }
    bool visited(int node) const {
        int n = node >> 5, offset = node & 0x1F;
        int mask = 1 << offset;
        return visited_[n] & mask;
    }
    bool reachable(int node) const {
        return distances_[node] < INT_MAX;
    }
    bool perimeter(int node, const CTP::graph_t &graph) const {
        return !visited(node) && reachable(node);
    }
    int distance_to(int node) const {
        return distances_[node];
    }
    bool is_dead_end() const {
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

    void preprocess(const CTP::graph_t &graph) {
        std::cout << "hola1" << std::endl;
        std::pair<bool, const int*> p = cache_.lookup(graph, current_, info_);
        if( p.first ) {
            if( !shared_ ) delete[] distances_;
            distances_ = p.second;
            shared_ = true;
        } else {
            if( shared_ ) distances_ = new int[num_nodes_];
            memcpy(const_cast<int*>(distances_), p.second, num_nodes_ * sizeof(int));
            shared_ = false;
        }
        heuristic_ = distances_[current_];
        std::cout << "hola2" << std::endl;
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
        heuristic_ = 0;
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
                shared_ = false;
                distances_ = new int[num_nodes_];
            }
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

    int bfs(const CTP::graph_t &graph,
            int start,
            int goal,
            bool optimistic = false) const {
        return graph.bfs(start, goal, info_.known_, info_.blocked_, optimistic);
    }
    void dijkstra(const CTP::graph_t &graph,
                  int source,
                  int *distances,
                  bool optimistic = false) const {
        graph.dijkstra(source, distances, info_.known_, info_.blocked_, optimistic);
    }
};

int state_t::num_nodes_ = 0;
int state_t::num_edges_ = 0;
int state_t::words_for_nodes_ = 0;
int state_t::words_for_edges_ = 0;
cache_t state_t::cache_;

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

  public:
    problem_t(CTP::graph_t &graph)
      : Problem::problem_t<state_t>(DISCOUNT, 1e3), // change dead_end_value
        graph_(graph), init_(-1), start_(0), goal_(graph_.num_nodes_ - 1),
        max_branching_(0) { }
    virtual ~problem_t() { }

    virtual Problem::action_t number_actions(const state_t &s) const {
        return s.current_ == -1 ? 1 : graph_.num_nodes_;
    }
    virtual bool applicable(const state_t &s, Problem::action_t a) const {
        return ((s.current_ == -1) && (a == 0)) ||
               ((s.current_ != -1) && s.perimeter(a, graph_));
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
        //std::cout << "next" << s << " w/ a=" << a << " is:" << std::endl;

        ++expansions_;
        outcomes.clear();

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
            if( p > 0 ) {
                next.preprocess(graph_);
                outcomes.push_back(std::make_pair(next, p));
                //std::cout << "    " << next << " w.p. " << p << std::endl;
            }
        }
    }
    virtual void print(std::ostream &os) const { }
};

inline std::ostream& operator<<(std::ostream &os, const problem_t &p) {
    p.print(os);
    return os;
}


class problem_with_hidden_state_t : public problem_t {
    mutable state_t hidden_;

  public:
    problem_with_hidden_state_t(CTP::graph_t &graph) : problem_t(graph) { }
    virtual ~problem_with_hidden_state_t() { }

    void set_hidden(state_t &hidden) const {
        hidden_ = hidden;
    }

    virtual void next(const state_t &s,
                      Problem::action_t a,
                      std::vector<std::pair<state_t, float> > &outcomes) const {
        ++expansions_;
        outcomes.clear();
        outcomes.reserve(1);

        int to_node = -1;
        if( s.current_ == -1 ) {
            to_node = start_;
        } else {
            to_node = a;
            assert(to_node != s.current_);
        }

        // set unique outcome using hidden state
        state_t next(s);
        for( int i = 0, isz = graph_.at_[to_node].size(); i < isz; ++i ) {
            int e = graph_.at_[to_node][i];
            next.info_.set_edge_status(e, hidden_.traversable(e) ? false : true);
        }
        next.move_to(to_node);
        next.preprocess(graph_);
        outcomes.push_back(std::make_pair(next, 1));
    }
};


class min_min_t : public Heuristic::heuristic_t<state_t> {
  public:
    min_min_t() { }
    virtual ~min_min_t() { }
    virtual float value(const state_t &s) const { return (float)s.heuristic_; }
    virtual void reset_stats() const { }
    virtual float setup_time() const { return 0; }
    virtual float eval_time() const { return 0; }
    virtual size_t size() const { return 0; }
    virtual void dump(std::ostream &os) const { }
    float operator()(const state_t &s) const { return value(s); }
};


inline void sample_weather(const CTP::graph_t &graph, state_t &state) {
    state.clear();
    int num_edges = graph.with_shortcut_ ? graph.num_edges_ - 1 : graph.num_edges_;
    for( int e = 0; e < num_edges; ++e ) {
        float p = graph.prob(e);
        if( Random::real() < p ) {
            state.info_.set_edge_status(e, false);
        } else {
            state.info_.set_edge_status(e, true);
        }
    }
    if( graph.with_shortcut_ ) state.info_.set_edge_status(graph.num_edges_ - 1, true);
}

inline float probability_bad_weather(const CTP::graph_t &graph,
                                     unsigned nsamples) {
    float prob = 0;
    state_t weather(0);
    for( unsigned i = 0; i < nsamples; ++i ) {
        sample_weather(graph, weather);
        weather.preprocess(graph);
        prob += weather.reachable(graph.num_nodes_ - 1) ? 0 : 1;
    }
    return prob / nsamples;
}


