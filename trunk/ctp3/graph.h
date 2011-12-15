#include <iostream>
#include <vector>
#include <queue>
#include <limits>

#undef INT_MAX
#define INT_MAX std::numeric_limits<int>::max()

namespace CTP {

struct open_list_cmp {
    bool operator()(const std::pair<int, int> &p1, const std::pair<int, int> &p2) {
        return p1.second > p2.second;
    }
};

struct graph_t {
    int num_nodes_;
    int num_edges_;
    bool with_shortcut_;
    int shortcut_cost_;

    struct edge_t {
        int from_, to_;
        int cost_;
        float prob_; // probability that edge is traversable (priv. comm. from Malte et al.)
        edge_t(int from, int to, int cost, float prob)
          : from_(from), to_(to), cost_(cost), prob_(prob) { }
    };

    std::vector<edge_t> edge_list_;
    std::vector<std::vector<int> > at_;
    int *edges_;

    graph_t(bool with_shortcut = false, int shortcut_cost = 1000)
      : num_nodes_(0), num_edges_(0),
        with_shortcut_(with_shortcut), shortcut_cost_(shortcut_cost),
        edges_(0) { }
    ~graph_t() { delete[] edges_; }

    int from(int edge) const { return edge_list_[edge].from_; }
    int to(int edge) const { return edge_list_[edge].to_; }
    int cost(int edge) const { return edge_list_[edge].cost_; }
    float prob(int edge) const { return edge_list_[edge].prob_; }

    int add_edge(const edge_t &edge) {
        assert((int)edge_list_.size() == num_edges_);
        edge_list_.push_back(edge);
        at_[edge.from_].push_back(num_edges_);
        at_[edge.to_].push_back(num_edges_);
        edges_[edge.from_ * num_nodes_ + edge.to_] = num_edges_;
        edges_[edge.to_ * num_nodes_ + edge.from_] = num_edges_;
        return num_edges_++;
    }

    bool parse(std::istream &is) {
        // read graph from file
        edge_list_.clear();
        at_.clear();

        std::string token;
        is >> token;
        if( token == "p" ) {
            int n_edges;
            is >> num_nodes_ >> n_edges;

            if( num_nodes_ > 128 ) {
                std::cout << "error: number of nodes must be <= 128." << std::endl;
                return false;
            }
            if( n_edges > 288 ) {
                std::cout << "error: number of edges must be <= 288." << std::endl;
                return false;
            }

            edges_ = new int[num_nodes_ * num_nodes_];
            for( int n1 = 0; n1 < num_nodes_; ++n1 ) {
                for( int n2 = 0; n2 < num_nodes_; ++n2 )
                    edges_[n1*num_nodes_ + n2] = -1;
            }

            at_.resize(num_nodes_);
            for( int e = 0; e < n_edges; ++e ) {
                is >> token;
                if( token == "e" ) {
                    int from, to, cost;
                    float prob;
                    is >> from >> to >> prob >> cost;
                    --from;
                    --to;
                    add_edge(edge_t(from, to, cost, prob));
                } else {
                    std::cout << "error reading input file: road doesn't start with 'e'." << std::endl;
                    return false;
                }
            }
        } else {
            std::cout << "error reading input file: file doesn't start with 'p'." << std::endl;
            return false;
        }

        // insert shortcut (s,t) edge
        if( with_shortcut_ ) {
            std::cout << "info: adding (s,t) shortcut w/ cost " << shortcut_cost_ << std::endl;
            if( 1+num_edges_ > 288 ) {
                std::cout << "error: number of edges must be <= 288." << std::endl;
                return false;
            }
            add_edge(edge_t(0, num_nodes_-1, shortcut_cost_, 1.0));
        }

        return true;
    }

    bool known(int edge, const unsigned *bitmap) const {
        int n = edge >> 5, offset = edge & 0x1F;
        int mask = 1 << offset;
        return bitmap[n] & mask;
    }
    bool traversable(int edge, const unsigned *bitmap) const {
        int n = edge >> 5, offset = edge & 0x1F;
        int mask = 1 << offset;
        return (bitmap[n] & mask) != 0 ? false : true;
    }
    bool good(int edge, const unsigned *k_bitmap, const unsigned *b_bitmap, bool optimistic) const {
        return (known(edge, k_bitmap) && traversable(edge, b_bitmap)) ||
               (optimistic && !known(edge, k_bitmap));
    }
    int edge(int n1, int n2,
             const unsigned *k_bitmap,
             const unsigned *b_bitmap,
             bool optimistic) const {
        int edge = edges_[n1*num_nodes_ + n2];
        if( edge != -1 ) {
            return good(edge, k_bitmap, b_bitmap, optimistic) ? edge : -1;
        } else {
            return -1;
        }
    }

    int bfs(int start,
            int goal,
            const unsigned *k_bitmap,
            const unsigned *b_bitmap,
            bool optimistic = false) const {
        return 0; // CHECK: implement bfs
    }

    void dijkstra(int source,
                  int *distances,
                  const unsigned *k_bitmap,
                  const unsigned *b_bitmap,
                  bool optimistic = false) const {

        static std::priority_queue<std::pair<int, int>,
                                   std::vector<std::pair<int, int> >,
                                   open_list_cmp> queue;

        // initialization
        for( int node = 0; node < num_nodes_; ++node )
            distances[node] = INT_MAX;

        // Dijsktra's
        distances[source] = 0;
        queue.push(std::make_pair(source, 0));
        while( !queue.empty() ) {
            std::pair<int, int> p = queue.top();
            queue.pop();
            if( p.second <= distances[p.first] ) {
                for( int i = 0, isz = at_[p.first].size(); i < isz; ++i ) {
                    int idx = at_[p.first][i];
                    if( good(idx, k_bitmap, b_bitmap, optimistic) ) {
                        const edge_t &edge = edge_list_[idx];
                        int cost = p.second + edge.cost_;
                        int next = p.first == edge.to_ ? edge.from_ : edge.to_;
                        if( cost < distances[next] ) {
                            distances[next] = cost;
                            queue.push(std::make_pair(next, cost));
                        }
                    }
                }
            }
        }
    }

    void floyd_warshall(int *distances,
                        const unsigned *k_bitmap,
                        const unsigned *b_bitmap,
                        bool optimistic = false) const {

        // initialization
        for( int n = 0; n < num_nodes_ * num_nodes_; ++n ) {
            distances[n] = INT_MAX;
        }
        for( int n = 0; n < num_nodes_; ++n ) {
            distances[n*num_nodes_ + n] = 0;
        }
        for( int e = 0; e < num_edges_; ++e ) {
            if( good(e, k_bitmap, b_bitmap, optimistic) ) {
                int n1 = to(e), n2 = from(e), edge_cost = cost(e);
                distances[n1*num_nodes_ + n2] = edge_cost;
                distances[n2*num_nodes_ + n1] = edge_cost;
            }
        }

        // Floyd-Warshall's
        for( int k = 0; k < num_nodes_; ++k ) {
            for( int i = 0; i < num_nodes_; ++i ) {
                if( distances[i*num_nodes_+k] == INT_MAX ) continue;
                for( int j = 0; j < num_nodes_; ++j ) {
                    if( distances[k*num_nodes_+j] == INT_MAX ) continue;
                    int ncost = distances[i*num_nodes_+k]+distances[k*num_nodes_+j];
                    if( ncost < distances[i*num_nodes_+j] ) {
                        distances[i*num_nodes_+j] = ncost;
                    }
                }
            }
        }

#if 0
        for( int k = 0; k < num_nodes_; ++k ) {
            for( int i = 0; i < num_nodes_; ++i ) {
                int edge_ik = edge(i, k, k_bitmap, b_bitmap, optimistic);
                if( edge_ik != -1 ) {
                    int cost_ik = cost(edge_ik);
                    std::cout << "edge " << edge_ik << "=(" << i << "," << k << ") is good: cost=" << cost_ik << std::endl;
                    for( int j = 0; j < num_nodes_; ++j ) {
                        int edge_kj = edge(k, j, k_bitmap, b_bitmap, optimistic);
                        if( edge_kj != -1 ) {
                            std::cout << "edge " << edge_kj << "=(" << k << "," << j << ") is good: cost=" << cost(edge_kj) << std::endl;
                            int old_cost = distances[i*num_nodes_ + j];
                            int new_cost = cost_ik + cost(edge_kj);
                            if( new_cost < old_cost )
                                distances[i*num_nodes_ + j] = new_cost;
                        }
                    }
                }
            }
        }
#endif
    }
};

}; // namespace CTP

