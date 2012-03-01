#ifndef ARC_CONSISTENCY_H
#define ARC_CONSISTENCY_H

#include <cassert>
#include <iostream>
#include <vector>

namespace CSP {

class constraint_digraph_t {
  public:
    typedef std::pair<int, int> edge_t;
    typedef std::vector<int> node_list_t;
    typedef std::vector<edge_t> edge_list_t;

  protected:
    int nvars_;
    int nedges_;
    std::vector<edge_list_t> edges_pointing_to_;

  public:
    constraint_digraph_t(int nvars = 0) {
        create_empty_graph(nvars);
    }
    constraint_digraph_t(const constraint_digraph_t &digraph)
      : nvars_(digraph.nvars_), nedges_(digraph.nedges_),
        edges_pointing_to_(digraph.edges_pointing_to_) {
    }
    constraint_digraph_t(const constraint_digraph_t &&digraph)
      : nvars_(digraph.nvars_), nedges_(digraph.nedges_),
        edges_pointing_to_(std::move(digraph.edges_pointing_to_)) {
    }
    ~constraint_digraph_t() { }

    void create_empty_graph(int nvars) {
        nvars_ = nvars;
        nedges_ = 0;
        edges_pointing_to_ = std::vector<edge_list_t>(nvars_);
    }

    void clear() {
        nedges_ = 0;
        edges_pointing_to_ = std::vector<edge_list_t>(nvars_);
    }

    int nvars() const { return nvars_; }
    int nedges() const { return nedges_; }

    const edge_list_t& edges_pointing_to(int var) const {
        return edges_pointing_to_[var];
    }

    void reserve_edge_list(int var, int reservation) {
        edges_pointing_to_[var].reserve(reservation);
    }

    void add_edge(int var_x, int var_y) {
        ++nedges_;
        edges_pointing_to_[var_y].push_back(std::make_pair(var_x, var_y));
    }
};

template<typename T> class arc_consistency_t {
  protected:
    int nvars_;
    std::vector<T*> domain_;
    const constraint_digraph_t &digraph_;

  public:
    arc_consistency_t(const constraint_digraph_t &digraph)
      : nvars_(digraph.nvars()), domain_(nvars_, static_cast<T*>(0)), digraph_(digraph) { }
    explicit arc_consistency_t(const arc_consistency_t &ac)
      : nvars_(ac.nvars_), domain_(ac.domain_), digraph_(ac.digraph_) { }
    arc_consistency_t(const arc_consistency_t &&ac)
      : nvars_(ac.nvars_), domain_(std::move(ac.domain_)),
        digraph_(std::move(ac.digraph_)) { }
    ~arc_consistency_t() { clear(); }

    typedef constraint_digraph_t::edge_list_t edge_list_t;
    typedef constraint_digraph_t::edge_t edge_t;

    T* domain(int var) { return domain_[var]; }
    const T* domain(int var) const { return domain_[var]; }
    void set_domain(int var, T *domain) { domain_[var] = domain; }

    void clear() {
        for( int var = 0; var < nvars_; ++var ) {
            delete domain_[var];
            domain_[var] = 0;
        }
    }
    void clear_domains() {
        for( int var = 0; var < nvars_; ++var ) {
            domain_[var]->clear();
        }
    }

    // check whether the partial assignment [ var_x = val_x, var_y = val_y ] is
    // consistent with the constraints. The preprocessing and postprocessing 
    // methods are called before and after making the consistency checks
    // associated with arc var_x -> var_y. By using them, one can safe time in
    // some cases; but they are not strictly needed.
    virtual bool consistent(int var_x, int var_y, int val_x, int val_y) const = 0;
    virtual void arc_reduce_preprocessing(int var_x, int var_y) = 0;
    virtual void arc_reduce_postprocessing(int var_x, int var_y) = 0;

    // reduce arc var_x -> var_y. That is, for each value of var_x, find a
    // compatible value for var_y. If such value is not found, the value of
    // var_x is removed from the domain of var_x. The method returns whether
    // some element of var_x is removed or not.
    bool arc_reduce(int var_x, int var_y) {
        assert(var_x != var_y);
        static std::vector<int> indices_to_erase;
        indices_to_erase.reserve(domain_[var_x]->size());
        indices_to_erase.clear();

        arc_reduce_preprocessing(var_x, var_y);
        int old_domainsz = domain_[var_x]->size();
        for( typename T::const_iterator it = domain_[var_x]->begin(); it != domain_[var_x]->end(); ++it ) {
            bool found_compatible_value = false;
            for( typename T::const_iterator jt = domain_[var_y]->begin(); jt != domain_[var_y]->end(); ++jt ) {
                if( consistent(var_x, var_y, *it, *jt) ) {
                    found_compatible_value = true;
                    break;
                }
            }
            if( !found_compatible_value ) indices_to_erase.push_back(it.index());
        }
        domain_[var_x]->erase_ordered_indices(indices_to_erase);
        arc_reduce_postprocessing(var_x, var_y);

        assert(indices_to_erase.empty() || (domain_[var_x]->size() < old_domainsz));
        return !indices_to_erase.empty();
    }

    // revise arc var_x -> var_y. This method is called after the reduction of
    // the edge removed some element in the domain of var_x. Then, all edges
    // ?var -> var_x such that ?var is different from var_y must be scheduled
    // for revision.
    void arc_revise(int var_x, int var_y, edge_list_t &worklist) {
        const edge_list_t &edges = digraph_.edges_pointing_to(var_x);
        for( int i = 0, isz = edges.size(); i < isz; ++i ) {
            int nvar = edges[i].first;
            assert(nvar != var_x);
            if( nvar != var_y ) {
                worklist.push_back(std::make_pair(nvar, var_x));
            }
        }
    }

    // AC3: Arc Consistency. Time is O(ed^3) where e is #edges in constraint
    // graph and d is size of larges domain: there are at most 2ed arc revisions
    // since each revision or arc (y,x) is caused by a removed element from the
    // domain of x (D_x), and each such revision takes time d^2 because for each
    // element of D_x, a consistent element of D_y must be found. Space is O(e)
    // since this is the maximum size of the worklist.
    void ac3(int seed_var, std::vector<bool> &revised_vars, bool propagate = true) {
        // initial worklist are all edges ?var -> seed_var
        edge_list_t worklist;
        worklist.reserve(digraph_.nedges());
        worklist = digraph_.edges_pointing_to(seed_var);

        // initially, no variable is revised
        revised_vars = std::vector<bool>(nvars_, false);

        // revise arcs until worklist becomes empty
        while( !worklist.empty() ) {
            edge_t edge = worklist.back();
            worklist.pop_back();
            int var_x = edge.first;
            int var_y = edge.second;

            // keep record of revised vars
            revised_vars[var_y] = true;

            // try to reduce arc var_x -> var_y
            if( arc_reduce(var_x, var_y) ) {
                if( domain_[var_x]->empty() ) {
                    // domain of var_x became empty. This means that the
                    // CSP is inconsistent. Clear all other domains.
                    clear_domains();
                    return;
                } else {
                    // some element was removed from domain of var_x,
                    // schedule revision of all arcs pointing to var_x
                    // different from arc the arc var_y -> var_x
                    if( propagate ) {
                        arc_revise(var_x, var_y, worklist);
                    } else {
                        assert(0); // delayed propagation not yet implemented
                    }
                }
            }
        }
    }
};

}; // end of namespace CSP

#endif

