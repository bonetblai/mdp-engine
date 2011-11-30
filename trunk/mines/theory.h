#ifndef THEORY_INCLUDE
#define THEORY_INCLUDE

#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <list>
#include <set>
#include <vector>
#include <cassert>

#include <CnfManager.h> // Jinbo Huang's SAT engine, tinisat

struct theory_t {
    int rows_;
    int cols_;
    int ncells_;
    int bombs_;

    CnfManager *cnf_manager_;

    mutable std::vector<int> clause_;
    std::list<std::vector<int> > clauses_;
    std::vector<std::vector<int> > adjacent_;
    std::vector<std::vector<int> > bitsums_;
    std::set<int> used_vars_;

    theory_t(int rows, int cols, int bombs = 0)
        : rows_(rows), cols_(cols), bombs_(bombs), cnf_manager_(0) {

        rows_ += 2; // 2 extra (boundary) rows
        cols_ += 2; // 2 extra (boundary) cols
        ncells_ = rows_ * cols_;

        // fill adjacent cells for inner cells
        adjacent_ = vector<vector<int> >(ncells_, vector<int>(9, 0));
        for( int r = 1; r < rows_ - 1; ++r ) {
            for( int c = 1; c < cols_ - 1; ++c ) {
                int p = cell(r, c);
                assert(adjacent_[p].size() == 9);
                adjacent_[p][0] = cell(r-1, c-1);
                adjacent_[p][1] = cell(r-1, c);
                adjacent_[p][2] = cell(r-1, c+1);
                adjacent_[p][3] = cell(r, c-1);
                adjacent_[p][4] = cell(r, c);
                adjacent_[p][5] = cell(r, c+1);
                adjacent_[p][6] = cell(r+1, c-1);
                adjacent_[p][7] = cell(r+1, c);
                adjacent_[p][8] = cell(r+1, c+1);
            }
        }

#if 0
        // fill bitsums
        bitsums_ = std::vector<std::vector<int> >(10, std::vector<int>());
        for( int n = 0; n < 512; ++n ) {
            int count = 0, aux = n;
            for( ; aux != 0; aux = aux >> 1 ) {
                if( aux & 1 ) ++count;
            }
            bitsums_[count].push_back(n);
        }

        for( int count = 0; count <= 8; ++count ) {
            std::cout << "bitsums[" << count << "] = {";
            for( int i = 0; i < bitsums_[count].size(); ++i ) {
                std::cout << " " << bitsums_[count][i];
            }
            std::cout << " }, sz = " << bitsums_[count].size() << std::endl;
        }
#endif
    }
    ~theory_t() { }

    // cells
    int cell(int r, int c) const { return r * cols_ + c; }
    bool inner(int p) const {
        int col = p % cols_, row = p / cols_;
        return (col != 0) && (col != cols_ - 1) && (row != 0) && (row != rows_ - 1);
    }
    bool corner(int p) const {
        int col = p % cols_, row = p / cols_;
        return ((row == 1) && ((col == 1) || (col == cols_ - 2))) ||
               ((row == rows_ - 2) && ((col == 1) || (col == cols_ - 2)));
    }
    bool side(int p) const {
        int col = p % cols_, row = p / cols_;
        return !corner(p) &&
               ((row == 1) || (row == rows_ - 2) || (col == 1) || (col == cols_ - 2));
    }

    // propositional variables: bomb(b,p), hasbomb(p), T(p,n), count(p,n)
    // Note: T(p,n) must be first variables to match indices used in mines.h
    int tuple(int p, int n) const { return p * 512 + n; }
    int hasbomb(int p) const { return tuple(ncells_, 0) + p; }
    int bomb(int b, int p) const { return 0; }
    int num_vars() const { return hasbomb(ncells_); }

    void push(int var, bool sign) {
        clause_.push_back(sign ? 1+var : -(1+var));
        used_vars_.insert(var);
    }
    void insert() {
        clauses_.push_back(clause_);
        clause_.clear();
    }

    void cell_theory(int p, int n) {
        assert(inner(p));

        // terms: T(p,n) iff the binary encoding of the 8 hashbomb(p')
        // variables (for p' adjacent to p) is equal to n

        // forward implication (=>)
        for( int i = 0; i < 9; ++i ) {
            int q = adjacent_[p][i];
            if( (n >> i) & 1 ) {
                push(tuple(p, n), false);
                push(hasbomb(q), true);
                insert();
            } else {
                push(tuple(p, n), false);
                push(hasbomb(q), false);
                insert();
            }
        }

        // backward implication (<=)
        for( int i = 0; i < 9; ++i ) {
            int q = adjacent_[p][i];
            if( (n >> i) & 1 ) {
                push(hasbomb(q), false);
            } else {
                push(hasbomb(q), true);
            }
        }
        push(tuple(p, n), true);
        insert();
    }

    void cell_theory(int p) {
        assert(inner(p));
        for( int n = 0; n < 512; ++n ) {
            cell_theory(p, n);
        }
    }

#if 0
    void obs_theory(int p, int count) {
        cell_theory(p);
        if( count == 9 ) {
            push(hasbomb(p), true);
            insert();
        } else {
            push(hasbomb(p), false);
            insert();
            for( unsigned i = 0; i < bitsums_[count].size(); ++i ) {
                push(tuple(p, bitsums_[count][i]), true);
            }
            insert();
        }
    }

    void set_unused_vars() {
        for( int var = 0; var < num_vars(); ++var ) {
            if( used_vars_.find(var) == used_vars_.end() ) {
                push(var, false);
                insert();
            }
        }
    }
#endif

    void theory() {
#if 0
        // phase 1: place bombs
        for( int b = 0; b < bombs_; ++b ) {
            // each bomb must be in at least one place: { bomb(b,p0), ..., bomb(b,pN) }
            for( int p = 0; p < ncells_; ++p ) {
                if( inner(p) )
                    clause_.push_back(bomb(b, p));
            }
            insert(clause_);

            // each bomb must be in at most one place: { -bomb(b,p_i), -bomb(b,p_j) }
            for( int p1 = 0; p1 < ncells_; ++p1 )
                for( int p2 = 0; p2 < p1; ++p2 ) {
                    if( inner(p1) && inner(p2) ) {
                        clause_.push_back(-bomb(b, p1));
                        clause_.push_back(-bomb(b, p2));
                        insert(clause_);
                    }
                }
        }

        // no two bombs at the same place: { -bomb(b_i,p), -bomb(b_j,p) }
        for( int p = 0; p < ncells_; ++p ) {
            if( inner(p) ) {
                for( int b1 = 0; b1 < bombs_; ++b1 )
                    for( int b2 = 0; b2 < b1; ++b2 ) {
                        clause_.push_back(-bomb(b1, p));
                        clause_.push_back(-bomb(b2, p));
                        insert(clause_);
                    }
            }
        }
#endif

#if 0
        // phase 2: hasbomb(p) iff place p has a bomb
        for( int p = 0; p < ncells_; ++p ) {
            if( inner(p) ) {
                // hasbomb(p) => bomb(b1,p) v ... v bomb(bN,p)
                for( int b = 0; b < bombs_; ++b )
                    clause_.push_back(bomb(b, p));
                clause_.push_back(-hasbomb(p));
                insert(clause_);

                // hasbomb(p) <= bomb(b_i,p)
                for( int b = 0; b < bombs_; ++b ) {
                    clause_.push_back(-bomb(b, p));
                    clause_.push_back(hasbomb(p));
                    insert(clause_);
                }
            }
        }
#endif

        // phase 3: tuples
        for( int p = 0; p < ncells_; ++p ) {
            if( inner(p) ) cell_theory(p);
        }

#if 0
        // phase 4: count(p,n) iff there are n bombs adjacent to the cell
        //          count(p,9) iff there is a bomb at p
        for( int p = 0; p < ncells_; ++p ) {
            if( inner(p) ) {
                // special case n = 9
                clause_.push_back(-count(p, 9));
                clause_.push_back(hasbomb(p));
                insert(clause_);
                clause_.push_back(-hasbomb(p));
                clause_.push_back(count(p, 9));
                insert(clause_);
            
                // other cases
                for( int n = 0; n < 9; ++n ) {
                    // forward implication (=>)
                    clause_.push_back(-count(p, n));
                    for( int i = 0; i < bitsums_[n].size(); ++i )
                        clause_.push_back(tuple(p, bitsums_[n][i]));
                    insert(clause_);

                    // backward implication (<=)
                    for( int i = 0; i < bitsums_[n].size(); ++i ) {
                        clause_.push_back(-tuple(p, bitsums_[n][i]));
                        clause_.push_back(count(p, n));
                        insert(clause_);
                    }
                }
            }
        }
#endif

        // phase 5: set value for boundary cells
        for( int p = 0; p < ncells_; ++p ) {
            if( !inner(p) ) {
#if 0
                for( int b = 0; b < bombs_; ++b ) {
                    clause_.push_back(-bomb(b, p));
                    insert(clause_);
                }
#endif
                push(hasbomb(p), false);
                insert();

                for( int n = 0; n < 512; ++n ) {
                    push(tuple(p, n), false);
                    insert();
                }

#if 0
                for( int n = 0; n < 10; ++n ) {
                    clause_.push_back(-count(p, n));
                    insert(clause_);
                }
#endif
            }
        }

        // phase 6: set hasbomb(b) as used vars
        for( int p = 0; p < ncells_; ++p ) {
            used_vars_.insert(hasbomb(p));
        }
    }

    void print(std::ostream &os, const std::vector<int> &clause) const {
        for( unsigned i = 0; i < clause.size(); ++i ) {
            int lit = clause[i];
            os << lit << " ";
        }
        os << "0" << std::endl;
    }
    void print(std::ostream &os) const {
        os << "p cnf " << num_vars() << " " << clauses_.size() << std::endl;
        for( std::list<std::vector<int> >::const_iterator it = clauses_.begin(); it != clauses_.end(); ++it ) {
            print(os, *it);
        }
        os << "%" << std::endl;
    }

    void create_manager() {
        theory();
        Cnf *cnf = new Cnf(num_vars(), clauses_.size());
        int clause_index = 0;
        for( std::list<std::vector<int> >::const_iterator it = clauses_.begin(); it != clauses_.end(); ++it ) {
            std::vector<int> literals = *it;
            literals.push_back(0);
            clause_index = cnf->add_clause(clause_index, &literals[0], literals.size());
            //std::cout << "add clause: "; print(std::cout, literals);
        }
        cnf->no_more_clauses(clause_index);
        cnf_manager_ = new CnfManager(*cnf);
        delete cnf;
    }

    void extract_derived_literals(const std::set<int> &base_literals, std::set<int> &derived_literals) {
        for( int var = 1; var <= num_vars(); ++var ) {
            int value = cnf_manager_->value(var);
            if( value != 2 ) {
                int lit = value == 0 ? -var : var;
                if( base_literals.find(lit) == base_literals.end() )
                    derived_literals.insert(lit);
            }
        }
    }

    void deduction(const std::set<int> &base_literals, const std::set<int> &literals, std::set<int> &new_literals) {
        // decide literals in base
        for( std::set<int>::const_iterator it = base_literals.begin(); it != base_literals.end(); ++it ) {
            int lit = *it;
            assert(lit != 0);
            int var = lit < 0 ? -lit : lit ;
            if( cnf_manager_->value(var) == 2 ) {
                cnf_manager_->decide(lit);
            }
        }

        // decide literals
        for( std::set<int>::const_iterator it = literals.begin(); it != literals.end(); ++it ) {
            int lit = *it;
            assert(lit != 0);
            int var = lit < 0 ? -lit : lit ;
            if( cnf_manager_->value(var) == 2 ) {
                cnf_manager_->decide(lit);
            }
        }

        // extract literals asserted by UP
        new_literals.clear();
        extract_derived_literals(base_literals, new_literals);

        // backtrack to dlevel 0; i.e., un-decide decisions
        cnf_manager_->backtrack(0);
    }
};

#endif

