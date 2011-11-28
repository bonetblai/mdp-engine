#include <iostream>
#include <stdlib.h>
#include <list>
#include <set>
#include <vector>
#include <cassert>

#include "mines.h"

using namespace std;

struct theory_t {
    int rows_;
    int cols_;
    int ncells_;
    int bombs_;

    mutable vector<int> clause_;
    list<vector<int> > clauses_;
    vector<vector<int> > adjacent_;
    vector<vector<int> > bitsums_;
    set<int> used_vars_;

    theory_t(int rows, int cols, int bombs = 0) : rows_(rows), cols_(cols), bombs_(bombs) {
        rows_ += 2; // 2 extra (boundary) rows
        cols_ += 2; // 2 extra (boundary) cols
        ncells_ = rows_ * cols_;

        // fill adjacent cells for inner cells
        adjacent_ = vector<vector<int> >(ncells_, vector<int>(8, 0));
        for( int r = 1; r < rows_ - 1; ++r ) {
            for( int c = 1; c < cols_ - 1; ++c ) {
                int p = cell(r, c);
                assert(adjacent_[p].size() == 8);
                adjacent_[p][0] = cell(r-1, c-1);
                adjacent_[p][1] = cell(r-1, c);
                adjacent_[p][2] = cell(r-1, c+1);
                adjacent_[p][3] = cell(r, c-1);
                adjacent_[p][4] = cell(r, c+1);
                adjacent_[p][5] = cell(r+1, c-1);
                adjacent_[p][6] = cell(r+1, c);
                adjacent_[p][7] = cell(r+1, c+1);
            }
        }

        // fill bitsums for inner cells
        bitsums_ = vector<vector<int> >(9, vector<int>());
        for( int n = 0; n < 256; ++n ) {
            int count = 0, aux = n;
            for( ; aux != 0; aux = aux >> 1 ) {
                if( aux & 1 ) ++count;
            }
            bitsums_[count].push_back(n);
        }
#if 0
        for( int count = 0; count <= 8; ++count ) {
            cout << "bitsums[" << count << "] = {";
            for( int i = 0; i < bitsums_[count].size(); ++i ) {
                cout << " " << bitsums_[count][i];
            }
            cout << " }, sz = " << bitsums_[count].size() << endl;
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
    int bomb(int b, int p) const { return 0 /*b * ncells_ + p*/; }
    int hasbomb(int p) const { return bomb(bombs_, 0) + p; }
    int tuple(int p, int n) const { return hasbomb(ncells_) + p * 256 + n; }
    int count(int p, int n) const { return tuple(ncells_, 0) + p * 10 + n; }
    int num_vars() const { return tuple(ncells_, 0) /*count(ncells_, 0)*/; }

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
        for( int i = 0; i < 8; ++i ) {
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
        for( int i = 0; i < 8; ++i ) {
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
        for( int n = 0; n < 256; ++n ) {
            cell_theory(p, n);
        }
    }

    void obs_theory(int p, int count) {
        assert(inner(p));
        cell_theory(p);
        if( count == 9 ) {
            push(hasbomb(p), true);
            insert();
        } else {
            push(hasbomb(p), false);
            insert();
            for( int i = 0; i < bitsums_[count].size(); ++i ) {
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

#if 0
        // phase 3: tuples
        for( int p = 0; p < ncells_; ++p ) {
            if( inner(p) ) cell_theory(p);
        }
#endif

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

                for( int n = 0; n < 256; ++n ) {
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
            if( inner(p) ) {
                used_vars_.insert(hasbomb(p));
            }
        }
    }

    void print(ostream &os, const vector<int> &clause) const {
        for( unsigned i = 0; i < clause.size(); ++i ) {
            int lit = clause[i];
            os << lit << " ";
        }
        os << "0" << endl;
    }
    void print(ostream &os) const {
        os << "p cnf " << num_vars() << " " << clauses_.size() << endl;
        for( list<vector<int> >::const_iterator it = clauses_.begin(); it != clauses_.end(); ++it ) {
            print(os, *it);
        }
        os << "%" << endl;
    }
};

int main(int argc, const char **argv) {
    ++argv;
    int rows = atoi(*argv++);
    int cols = atoi(*argv++);
    int obs = atoi(*argv++);

    unsigned short useed[3];
    useed[0] = useed[1] = useed[2] = 0;
    if( argc > 4 ) {
        useed[0] = useed[1] = useed[2] = atoi(*argv++);
    }
    seed48(useed);

    //int bombs = atoi(argv[3]);
    //cerr << "rows=" << rows << ", cols=" << cols << ", bombs=" << bombs << endl;

    cerr << "seed=" << useed[0] << endl;
    cerr << "rows=" << rows << ", cols=" << cols << ", obs=" << obs << endl;
    theory_t theory(rows, cols);
    theory.theory();

    for( int i = 0; i < obs; ++i ) {
        int r = atoi(*argv++); //lrand48() % rows;
        int c = atoi(*argv++); //lrand48() % cols;
        int p = theory.cell(1+r, 1+c);
        int count = atoi(*argv++);
#if 0
        int count = 0;
        if( theory.corner(p) ) {
            count = lrand48() % 4;
        } else if( theory.side(p) ) {
            count = lrand48() % 6;
        } else {
            count = lrand48() % 9;
        }
#endif
        cerr << "obs: (" << r << "," << c << ")=" << count << endl;
        theory.obs_theory(p, count);
    }
    theory.set_unused_vars();

    cout << "c seed " << useed[0] << endl;
    theory.print(cout);
    return 0;
}

