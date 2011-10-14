#include <iostream>
#include <iomanip>
#include <math.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <float.h>
#include <assert.h>

#define MAXOUTCOMES   4

#include "../common.h"

namespace Utils {
  int verbosity = 0;
  float kappa_log = log( 2.0 );
};

const Problem::action_t fwd = 1;
const Problem::action_t left = 2;
const Problem::action_t right = 3;

typedef unsigned short ushort_t;

class state_t {
  ushort_t row_;
  ushort_t col_;
public:
  state_t( ushort_t row = 0, ushort_t col = 0 ) : row_(row), col_(col) { }
  state_t( const state_t &s ) : row_(s.row_), col_(s.col_) { }
  ~state_t() { }
  size_t hash() const { return( row_ ^ col_ ); }
  size_t row() const { return( row_ ); }
  size_t col() const { return( col_ ); }
  void fwd( size_t rows ) { if( row() < rows-1 ) ++row_; }
  void left( size_t cols ) { if( col() > 0 ) --col_; }
  void right( size_t cols ) { if( col() < cols-1 ) ++col_; }
  const state_t& operator=( const state_t &s ) { row_ = s.row_; col_ = s.col_; return( *this ); }
  bool operator==( const state_t &s ) const { return( (row_ == s.row_) && (col_ == s.col_) ); }
  bool operator!=( const state_t &s ) const { return( (row_ != s.row_) || (col_ != s.col_) ); }
  bool operator<( const state_t &s ) const { return( (row_ < s.row_) || ((row_ == s.row_) && (col_ < s.col_)) ); }
  void print( std::ostream &os ) const { os << "( " << row() << " , " << col() << " )"; }
  friend class problem_t;
};

inline std::ostream& operator<<( std::ostream &os, const state_t &s ) { s.print( os ); return( os ); }

class problem_t : public Problem::problem_t<state_t> {
  size_t rows_;
  size_t cols_;
  float p_;
  state_t init_;
public:
  problem_t( size_t rows, size_t cols, float p = 1.0 ) : rows_(rows), cols_(cols), p_(p), init_(0,cols/2) { }
  virtual ~problem_t() { }
  virtual Problem::action_t last_action() const { return( 4 ); }
  virtual const state_t& init() const { return( init_ ); }
  virtual bool terminal( const state_t &s ) const { return( s.row() == rows_-1 ); }
  virtual void next( const state_t &s, Problem::action_t a, std::pair<state_t,float> *outcomes, size_t &osize ) const
  {
    ++expansions_;
    size_t i = 0;
    if( a != fwd ) {
      outcomes[i++] = std::make_pair( s, 1.0 );
      if( a == left )
        outcomes[0].first.left( cols_ );
      else if( a == right )
        outcomes[0].first.right( cols_ );
    }
    else if( a == fwd ) {
      if( p_ > 0 ) {
        outcomes[i] = std::make_pair( s, p_ );
        outcomes[i++].first.fwd( rows_ );
      }
      if( 1-p_ > 0 ) {
        outcomes[i] = std::make_pair( s, (1-p_)/2 );
        outcomes[i++].first.left( cols_ );
        outcomes[i] = std::make_pair( s, (1-p_)/2 );
        outcomes[i++].first.right( cols_ );
      }
    }
    osize = i;
  }
  virtual void print( std::ostream &os ) const { }
};

inline std::ostream& operator<<( std::ostream &os, const problem_t &p ) { p.print( os ); return( os ); }

void
usage( std::ostream &os )
{
  os << std::endl
     << "usage: rect [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-k <n>] [-K <f>] [-p <f>] [-s <n>] [-v <n>] <rows> <cols>"
     << std::endl << std::endl
     << "  -a <n>    Algorithm bitmask: 1=vi, 2=slrtdp, 4=ulrtdp, 8=blrtdp, 16=ilao, 32=plain-check, 64=elrtdp, 128=hdp-i, 256=hdp, 512=ldfs+, 1024=ldfs."
     << std::endl
     << "  -b <n>    Visits bound for blrtdp. Default: 0."
     << std::endl
     << "  -e <f>    Epsilon. Default: 0."
     << std::endl
     << "  -f        Formatted output."
     << std::endl
     << "  -g <f>    Parameter for epsilon-greedy. Default: 0."
     << std::endl
     << "  -h <n>    Heuristics: 0=zero, 1=minmin, 2=hdp(0). Default: 0."
     << std::endl
     << "  -k <n>    Kappa consistency level. Default: 0."
     << std::endl
     << "  -K <f>    Used to define kappa measures. Default: 2."
     << std::endl
     << "  -p <f>    Parameter p in [0,1]. Default: 1."
     << std::endl
     << "  -s <n>    Random seed. Default: 0."
     << std::endl
     << "  -v <n>    Verbosity. Default: 0."
     << std::endl
     << "  <rows>    Rows <= 2^16."
     << std::endl
     << "  <cols>    Cols <= 2^16."
     << std::endl << std::endl;
}

char *name[] = { "vi", "slrtdp", "ulrtdp", "blrtdp", "ilao", "elrtdp", "check", "hdp-i", "hdp", "ldfs+", "ldfs" };
size_t (*table[])( const Problem::problem_t<state_t>&, Hash::hash_t<state_t>&, float, size_t, float ) = {
  Algorithm::value_iteration<state_t>,
  Algorithm::standard_lrtdp<state_t>,
  Algorithm::uniform_lrtdp<state_t>,
  Algorithm::bounded_lrtdp<state_t>,
  Algorithm::improved_lao<state_t>,
  Algorithm::standard_lrtdp<state_t>,
  Algorithm::plain_check<state_t>,
  Algorithm::hdp_i<state_t>,
  Algorithm::hdp_driver<state_t>,
  Algorithm::ldfs_plus_driver<state_t>,
  Algorithm::ldfs_driver<state_t>,
  0
};

int
main( int argc, char **argv )
{
  size_t rows = 0;
  size_t cols = 0;
  float p = 1.0;
  unsigned alg = 0;
  float eps = 0.0;
  float g = 0.0;
  int h = 0;
  unsigned long seed = 0;
  size_t bound = 0;
  size_t kappa = 0;
  bool formatted = false;

  // parse arguments
  ++argv;
  --argc;
  while( argc > 1 ) {
    if( **argv != '-' ) break;
    switch( (*argv)[1] ) {
      case 'a':
        alg = strtoul( argv[1], 0, 0 );
        argv += 2;
        argc -= 2;
        break;
      case 'b':
        bound = strtol( argv[1], 0, 0 );
        argv += 2;
        argc -= 2;
        break;
      case 'e':
        eps = strtod( argv[1], 0 );
        argv += 2;
        argc -= 2;
        break;
      case 'f':
        formatted = true;
        ++argv;
        --argc;
        break;
      case 'g':
        g = strtod( argv[1], 0 );
        argv += 2;
        argc -= 2;
        break;
      case 'h':
        h = strtol( argv[1], 0, 0 );
        argv += 2;
        argc -= 2;
        break;
      case 'k':
        kappa = strtol( argv[1], 0, 0 );
        argv += 2;
        argc -= 2;
        break;
      case 'K':
        Utils::kappa_log = log( strtod( argv[1], 0 ) );
        argv += 2;
        argc -= 2;
        break;
      case 'p':
        p = strtod( argv[1], 0 );
        argv += 2;
        argc -= 2;
        break;
      case 's':
        seed = strtoul( argv[1], 0, 0 );
        argv += 2;
        argc -= 2;
        break;
      case 'v':
        Utils::verbosity = strtoul( argv[1], 0, 0 );
        argv += 2;
        argc -= 2;
        break;
      default:
        usage( std::cout );
        exit( -1 );
    }
  }
  if( argc == 2 ) {
    rows = strtoul( argv[0], 0, 0 );
    cols = strtoul( argv[1], 0, 0 );
  }
  else {
    usage( std::cout );
    exit( -1 );
  }

  // build problem instances
  srand48( seed );
  problem_t problem( rows, cols, p );

  // solve problem with algorithms
  size_t first = UINT_MAX;
  for( size_t i = 0; (i < 10) && (table[i] != 0); ++i ) {
    if( (alg>>i) % 2 ) {
      srand48( seed );
      first = Utils::min( first, i );
      Heuristic::heuristic_t<state_t> *heuristic = 0;

      if( h == 1 )
        heuristic = new Heuristic::min_min_heuristic_t<state_t>( problem );
      else if( h == 2 )
        heuristic = new Heuristic::hdp_heuristic_t<state_t>( problem, eps, 0 );

      float start_time = Utils::read_time_in_seconds();
      Hash::hash_t<state_t> hash( heuristic );
      problem.clear_expansions();
      size_t t = (*table[i])( problem, hash, eps, (i==3?bound:(i==7?kappa:UINT_MAX)), (i==5?g:0.0) );
      float end_time = Utils::read_time_in_seconds();
      float htime = (!heuristic?0:heuristic->time());
      float dtime = (!heuristic?0:heuristic->d_time());
      float atime = end_time - start_time -dtime;

      if( formatted ) {
        if( i == first ) {
          std::cout << std::setw(4) << "#" << " "
                    << std::setw(7) << "alg" << " "
                    << std::setw(12) << "V*(s0)" << " "
                    << std::setw(7) << "seed" << " "
                    << std::setw(12) << "trials" << " "
                    << std::setw(12) << "updates" << " "
                    << std::setw(12) << "expansions" << " "
                    << std::setw(12) << "h(s0)" << " "
                    << std::setw(12) << "hsize" << " "
                    << std::setw(12) << "psize" << " "
                    << std::setw(12) << "atime" << " "
                    << std::setw(12) << "htime" << std::endl;
        }
        std::cout << std::setw(4) << (1<<i) << " "
                  << std::setw(7) << name[i] << " "
                  << std::setw(12) << std::setprecision(9) << hash.value( problem.init() ) << " "
                  << std::setw(7) << seed << " "
                  << std::setw(12) << t << " "
                  << std::setw(12) << hash.updates() << " "
                  << std::setw(12) << problem.expansions() << " "
                  << std::setw(12) << std::setprecision(9) << (!heuristic?0:heuristic->value(problem.init())) << " "
                  << std::setw(12) << hash.size() << " "
                  << std::setw(12) << (i==7?0:problem.policy_size(hash,problem.init())) << " "
                  << std::setw(12) << std::setprecision(9) << atime << " "
                  << std::setw(12) << std::setprecision(9) << htime << std::endl;
      }
      else {
        std::cout << (1<<i) << " "
                  << name[i] << " "
                  << hash.value( problem.init() ) << " "
                  << seed << " "
                  << t << " "
                  << hash.updates() << " "
                  << problem.expansions() << " "
                  << std::setprecision(9) << (!heuristic?0:heuristic->value(problem.init())) << " "
                  << hash.size() << " "
                  << (i==7?0:problem.policy_size(hash, problem.init())) << " "
                  << std::setprecision(9) << atime << " "
                  << std::setprecision(9) << htime << std::endl;
      }

      delete heuristic;
    }
  }
  exit( 0 );
}

