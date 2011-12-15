#include <cassert>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>
#include <limits>

#include "parsing.h"
#include "algorithm.h"
#include "parameters.h"
#include "heuristic.h"

#include "policy.h"
#include "rollout.h"
#include "mcts.h"
#include "dispatcher.h"

#define DISCOUNT 1.00
#undef INT_MAX
#define INT_MAX std::numeric_limits<int>::max()

#define WORDS_FOR_NODES 4 // max. 128 nodes
#define WORDS_FOR_EDGES 9 // max. 288 edges

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

struct state_t {
    int current_;
    unsigned known_[WORDS_FOR_EDGES];
    unsigned blocked_[WORDS_FOR_EDGES];
    unsigned visited_[WORDS_FOR_NODES];
    mutable int *distances_;
    int heuristic_;

    static int num_nodes_;
    static int num_edges_;
    static int words_for_nodes_;
    static int words_for_edges_;
    static int static_distances_[];

  public:
    state_t(int current = -1)
      : current_(current), distances_(0), heuristic_(0) {
        memset(known_, 0, WORDS_FOR_EDGES * sizeof(unsigned));
        memset(blocked_, 0, WORDS_FOR_EDGES * sizeof(unsigned));
        memset(visited_, 0, WORDS_FOR_NODES * sizeof(unsigned));
        distances_ = new int[num_nodes_];
    }
    state_t(const state_t &s) {
        distances_ = new int[num_nodes_];
        *this = s;
    }
    ~state_t() { delete[] distances_; }

    static void initialize(unsigned num_nodes, unsigned num_edges) {
        num_nodes_ = num_nodes;
        num_edges_ = num_edges;
        assert(num_nodes_ <= WORDS_FOR_NODES * 4 * 8);
        assert(num_edges_ <= WORDS_FOR_EDGES * 4 * 8);
        words_for_nodes_ = num_nodes_ >> 5;
        if( (num_nodes_ & 0x1F) != 0 ) ++words_for_nodes_;
        words_for_edges_ = num_edges_ >> 5;
        if( (num_edges_ & 0x1F) != 0 ) ++words_for_edges_;
        std::cout << "init: #nodes=" << num_nodes_
                  << ", #edges=" << num_edges_
                  << ", #words-for-nodes=" << words_for_nodes_
                  << ", #words-for-edges=" << words_for_edges_
                  << std::endl;
    }

    size_t hash() const {
        unsigned rv = current_;
        for( int i = 0; i < words_for_edges_; ++i )
            rv = rv ^ known_[i];
        for( int i = 0; i < words_for_edges_; ++i )
            rv = rv ^ blocked_[i];
        return rv;
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
    int distance_to(int node) const { return distances_[node]; }
    bool is_dead_end() const {
        return (current_ != -1) && (heuristic_ == INT_MAX);
    }

    void move_to(int node) {
        int n = node >> 5, offset = node & 0x1F;
        int mask = 1 << offset;
        current_ = node;
        visited_[n] |= mask;
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
    void preprocess(const CTP::graph_t &graph) {
        compute_distances(graph);
        compute_distances(graph, static_distances_, true);
        heuristic_ = static_distances_[graph.num_nodes_ - 1];
    }

    void clear() {
        current_ = 0;
        memset(known_, 0, words_for_edges_ * sizeof(unsigned));
        memset(blocked_, 0, words_for_edges_ * sizeof(unsigned));
        memset(visited_, 0, words_for_nodes_ * sizeof(unsigned));
        memset(distances_, 0, num_nodes_ * sizeof(int));
        heuristic_ = 0;
    }

    const state_t& operator=(const state_t &s) {
        current_ = s.current_;
        memcpy(known_, s.known_, words_for_edges_ * sizeof(unsigned));
        memcpy(blocked_, s.blocked_, words_for_edges_ * sizeof(unsigned));
        memcpy(visited_, s.visited_, words_for_nodes_ * sizeof(unsigned));
        memcpy(distances_, s.distances_, num_nodes_ * sizeof(int));
        heuristic_ = s.heuristic_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return (current_ == s.current_) &&
               (memcmp(known_, s.known_, words_for_edges_ * sizeof(unsigned)) == 0) &&
               (memcmp(blocked_, s.blocked_, words_for_edges_ * sizeof(unsigned)) == 0) &&
               (memcmp(visited_, s.visited_, words_for_nodes_ * sizeof(unsigned)) == 0);
    }
    bool operator!=(const state_t &s) const {
        return *this == s ? false : true;
    }
    bool operator<(const state_t &s) const {
        return (current_ < s.current_) ||
               ((current_ == s.current_) &&
                (memcmp(known_, s.known_, words_for_edges_ * sizeof(unsigned)) < 0)) ||
               ((current_ == s.current_) &&
                (memcmp(known_, s.known_, words_for_edges_ * sizeof(unsigned)) == 0) &&
                (memcmp(blocked_, s.blocked_, words_for_edges_ * sizeof(unsigned)) < 0)) ||
               ((current_ == s.current_) &&
                (memcmp(known_, s.known_, words_for_edges_ * sizeof(unsigned)) == 0) &&
                (memcmp(blocked_, s.blocked_, words_for_edges_ * sizeof(unsigned)) == 0) &&
                (memcmp(visited_, s.visited_, words_for_nodes_ * sizeof(unsigned)) < 0));
    }
    void print(std::ostream &os) const {
        os << words_for_edges_ << ":(" << current_ << ",K={";
        for( int i = 0; i < words_for_edges_; ++i )
            os << bits_t<unsigned>(known_[i], i << 5);
        os << "},B={";
        for( int i = 0; i < words_for_edges_; ++i )
            os << bits_t<unsigned>(blocked_[i], i << 5);
        os << "},V={";
        for( int i = 0; i < words_for_nodes_; ++i )
            os << bits_t<unsigned>(visited_[i], i << 5);
        os << "}";
        if( is_dead_end() ) os << ",DEAD";
        os << ")";
    }

    void compute_distances(const CTP::graph_t &graph,
                           int *dist,
                           bool optimistic = false) const {

        // compute shortest paths in known (or optimistic) graph
        static std::priority_queue<std::pair<int, int>,
                                   std::vector<std::pair<int, int> >,
                                   CTP::open_list_cmp> queue;

        // initialization of values
        for( int n = 0; n < graph.num_nodes_; ++n ) {
            dist[n] = INT_MAX;
        }

        // Dijsktra's with goal node as seed
        int start = current_;
        dist[start] = 0;
        queue.push(std::make_pair(start, 0));
        while( !queue.empty() ) {
            std::pair<int, int> p = queue.top();
            queue.pop();
            if( p.second <= dist[p.first] ) {
                for( int i = 0, isz = graph.at_[p.first].size(); i < isz; ++i ) {
                    int j = graph.at_[p.first][i];
                    if( (known(j) && traversable(j)) ||
                        (optimistic && !known(j)) ) {
                        const CTP::graph_t::edge_t &e = graph.edge_list_[j];
                        int cost = p.second + e.cost_;
                        int next = p.first == e.to_ ? e.from_ : e.to_;
                        if( cost < dist[next] ) {
                            dist[next] = cost;
                            queue.push(std::make_pair(next, cost));
                        }
                    }
                }
            }
        }
    }
    void compute_distances(const CTP::graph_t &graph) {
        compute_distances(graph, distances_);
    }

    void print_path(const CTP::graph_t &graph, const int *dist) const {
        int path_cost = 0;
        std::cout << "path=<" << std::flush;
        if( dist[current_] < INT_MAX ) {
            int node = current_;
            std::cout << "n=" << node << std::flush;
            while( node != graph.num_nodes_ - 1 ) {
                int best = -1, cost = INT_MAX;
                for( int i = 0, isz = graph.at_[node].size(); i < isz; ++i ) {
                    int j = graph.at_[node][i];
                    if( known(j) && traversable(j) ) {
                        const CTP::graph_t::edge_t &e = graph.edge_list_[j];
                        int next = node == e.to_ ? e.from_ : e.to_;
                        if( dist[next] < cost ) {
                            best = j;
                            cost = dist[next];
                        }
                    } else {
                        //std::cout << "edge " << j << " is unknown or non-traversable" << std::endl;
                    }
                }
                const CTP::graph_t::edge_t &e = graph.edge_list_[best];
                int next = node == e.to_ ? e.from_ : e.to_;
                node = next;
                path_cost += e.cost_;
                std::cout << ",e=" << best << std::flush;
                std::cout << ",n=" << node << std::flush;
            }
            std::cout << ">, cost=" << path_cost << std::endl;
        }
    }

};

int state_t::num_nodes_ = 0;
int state_t::num_edges_ = 0;
int state_t::words_for_nodes_ = 0;
int state_t::words_for_edges_ = 0;
int state_t::static_distances_[WORDS_FOR_NODES*32];

inline std::ostream& operator<<(std::ostream &os, const state_t &s) {
    s.print(os);
    return os;
}

//bool cmp_function(const std::pair<state_t, float> &p1, const std::pair<state_t, float> &p2) {
//    return p1.second > p2.second;
//}

class problem_t : public Problem::problem_t<state_t> {
  public:
    const CTP::graph_t &graph_;
    const state_t init_;
    int start_, goal_;

  public:
    problem_t(CTP::graph_t &graph)
      : Problem::problem_t<state_t>(DISCOUNT, 1e3), // change dead_end_value
        graph_(graph), init_(-1), start_(0), goal_(graph_.num_nodes_ - 1) { }
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

        // generate subsets of unknowns edges and update weathers
        outcomes.reserve(1 << k);
        for( int i = 0, isz = 1 << k; i < isz; ++i ) {
            state_t next(s);
            float p = 1;
            int subset = i;
            for( int j = 0; j < k; ++j ) {
                int e = unknown_edges[j];
                p *= (subset & 1) ? 1 - graph_.prob(e) : graph_.prob(e);
                next.set_edge_status(e, subset & 1);
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
            next.set_edge_status(e, hidden_.traversable(e) ? false : true);
        }
        next.move_to(to_node);
        next.preprocess(graph_);
        outcomes.push_back(std::make_pair(next, 1));
    }
};

inline void sample_weather(const CTP::graph_t &graph, state_t &state) {
    state.clear();
    int num_edges = graph.with_shortcut_ ? graph.num_edges_ - 1 : graph.num_edges_;
    for( int e = 0; e < num_edges; ++e ) {
        float p = graph.prob(e);
        if( Random::real() < p ) {
            state.set_edge_status(e, false);
        } else {
            state.set_edge_status(e, true);
        }
    }
    if( graph.with_shortcut_ ) state.set_edge_status(graph.num_edges_ - 1, true);
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

