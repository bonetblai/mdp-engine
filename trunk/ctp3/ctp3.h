#include <cassert>
#include <strings.h>
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
#undef UINT_MAX
#define UINT_MAX std::numeric_limits<unsigned>::max()

#define B_EDGES 36 // max. 288 edges
#define B_NODES 16 // max. 128 nodes

struct state_t {
    int current_;
    unsigned char known_[B_EDGES];
    unsigned char blocked_[B_EDGES];
    unsigned char visited_[B_NODES];
    mutable unsigned *distances_;
    unsigned heuristic_;

    static unsigned static_distances_[1<<B_NODES];

  public:
    state_t(int current = -1)
      : current_(current), distances_(0), heuristic_(UINT_MAX) {
        bzero(known_, B_EDGES);
        bzero(blocked_, B_EDGES);
        bzero(visited_, B_NODES);
    }
    state_t(const state_t &s) { *this = s; }
    ~state_t() { delete[] distances_; }

    size_t hash() const { return current_ + (known_ ^ blocked_); }

    bool known(int edge) const {
        int mask = 1 << edge;
        return known_ & mask;
    }
    bool traversable(int edge) const {
        int mask = 1 << edge;
        return (blocked_ & mask) != 0 ? false : true;
    }
    bool visited(int node) const {
        int mask = 1 << node;
        return visited_ & mask;
    }
    bool reachable(int node) const {
        return distances_[node] < UINT_MAX;
    }
    bool perimeter(int node, const CTP::graph_t &graph) const {
        return !visited(node) && reachable(node);
    }
    int distance_to(int node) const { return distances_[node]; }
    bool is_dead_end() const {
        return (current_ != -1) && (heuristic_ == UINT_MAX);
    }

    void move_to(int node) {
        int mask = 1 << node;
        current_ = node;
        visited_ |= mask;
    }
    void set_edge_status(int edge, bool blocked) {
        int mask = 1 << edge;
        known_ |= mask;
        if( blocked )
            blocked_ |= mask;
        else
            blocked_ &= ~mask;
    }
    void preprocess(const CTP::graph_t &graph) {
        compute_distances(graph);
        compute_distances(graph, static_distances_, true);
        heuristic_ = static_distances_[graph.num_nodes_ - 1];
    }

    const state_t& operator=(const state_t &s) {
        current_ = s.current_;
        bcopy(s.known_, known_, B_EDGES);
        bcopy(s.blocked_, blocked_, B_EDGES);
        bcopy(s.visited_, visited_, B_NODES);
        distances_ = s.distances_; // TODO: THIS IS WRONG!!!!!
        heuristic_ = s.heuristic_;
        return *this;
    }
    bool operator==(const state_t &s) const {
        return (current_ == s.current_) && (known_ == s.known_) &&
               (blocked_ == s.blocked_) && (visited_ == s.visited_);
    }
    bool operator!=(const state_t &s) const {
        return (current_ != s.current_) || (known_ != s.known_) ||
               (blocked_ != s.blocked_) || (visited_ != s.visited_);
    }
    bool operator<(const state_t &s) const {
        return (current_ < s.current_) ||
               ((current_ == s.current_) && (known_ < s.known_)) ||
               ((current_ == s.current_) && (known_ == s.known_) &&
                (blocked_ < s.blocked_)) ||
               ((current_ == s.current_) && (known_ == s.known_) &&
                (blocked_ == s.blocked_) && (visited_ < s.visited_));
    }
    void print(std::ostream &os) const {
        os << "("
           << current_
           << "," << known_
           << "," << blocked_
           << "," << visited_;
        if( is_dead_end() ) os << ",DE";
        os << ")";
    }

    void compute_distances(const CTP::graph_t &graph,
                           unsigned *dist,
                           bool optimistic = false) const {

        if( dist == 0 ) {
            dist = distances_ = new unsigned[graph.num_nodes_];
        }

        // compute shortest paths in known (or optimistic) graph
        static std::priority_queue<std::pair<unsigned, unsigned>,
                                   std::vector<std::pair<unsigned, unsigned> >,
                                   CTP::open_list_cmp> queue;

        // initialization of values
        for( unsigned n = 0; n < graph.num_nodes_; ++n ) {
            dist[n] = UINT_MAX;
        }

        // Dijsktra's with goal node as seed
        int start = current_;
        dist[start] = 0;
        queue.push(std::make_pair(start, 0));
        while( !queue.empty() ) {
            std::pair<unsigned, unsigned> p = queue.top();
            queue.pop();
            if( p.second <= dist[p.first] ) {
                for( int i = 0, isz = graph.at_[p.first].size(); i < isz; ++i ) {
                    int j = graph.at_[p.first][i];
                    if( (known(j) && traversable(j)) ||
                        (optimistic && !known(j)) ) {
                        const CTP::graph_t::edge_t &e = graph.edge_list_[j];
                        unsigned cost = p.second + e.cost_;
                        unsigned next = p.first == e.to_ ? e.from_ : e.to_;
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

    void print_path(const CTP::graph_t &graph, const unsigned *dist) const {
        int path_cost = 0;
        std::cout << "path=<" << std::flush;
        if( dist[current_] < UINT_MAX ) {
            unsigned node = current_;
            std::cout << "n=" << node << std::flush;
            while( node != graph.num_nodes_ - 1 ) {
                int best = -1;
                unsigned cost = UINT_MAX;
                for( int i = 0, isz = graph.at_[node].size(); i < isz; ++i ) {
                    int j = graph.at_[node][i];
                    if( known(j) && traversable(j) ) {
                        const CTP::graph_t::edge_t &e = graph.edge_list_[j];
                        unsigned next = node == e.to_ ? e.from_ : e.to_;
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

//unsigned state_t::static_distances_[128];

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

inline state_t sample_weather(const CTP::graph_t &graph) {
    state_t state(0);
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
    return state;
}

inline float probability_bad_weather(const CTP::graph_t &graph,
                                     unsigned nsamples) {
    unsigned *distances = new unsigned[graph.num_nodes_];
    float prob = 0;
    for( unsigned i = 0; i < nsamples; ++i ) {
        state_t weather = sample_weather(graph);
        weather.compute_distances(graph, distances);
        prob += weather.reachable(graph.num_nodes_ - 1) ? 0 : 1;
    }
    delete[] distances;
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

