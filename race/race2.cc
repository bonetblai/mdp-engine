#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <math.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <float.h>
#include <assert.h>

#define  MAXOUTCOMES  10

#include "parsing.h"
#include "../common.h"

#ifndef USHORT_MAX
#define USHORT_MAX    65535
#endif
#ifndef SHORT_MAX
#define SHORT_MAX     32767
#endif

namespace Utils {
  int verbosity = 0;
  float kappa_log = log( 2.0 );
};

class state_t {
  short x_;
  short y_;
  short dx_;
  short dy_;
public:
  state_t( short x = 0, short y = 0, short dx = 0, short dy = 0 ) : x_(x), y_(y), dx_(dx), dy_(dy) { }
  state_t( const state_t &s ) : x_(s.x_), y_(s.y_), dx_(s.dx_), dy_(s.dy_) { }
  ~state_t() { }
  size_t hash() const { return( (x_ | (y_<<16)) ^ (dx_ | (dy_<<16)) ); }
  short x() const { return( x_ ); }
  short y() const { return( y_ ); }
  short dx() const { return( dx_ ); }
  short dy() const { return( dy_ ); }
  const state_t& operator=( const state_t &s ) { x_ = s.x_; y_ = s.y_; dx_ = s.dx_; dy_ = s.dy_; return( *this ); }
  bool operator==( const state_t &s ) const { return( (x_ == s.x_) && (y_ == s.y_) && (dx_ == s.dx_) && (dy_ == s.dy_) ); }
  bool operator!=( const state_t &s ) const { return( (x_ != s.x_) || (y_ != s.y_) || (dx_ != s.dx_) || (dy_ != s.dy_) ); }
  bool operator<( const state_t &s ) const { return( (x_ < s.x_) || ((x_ == s.x_) && (y_ < s.y_)) || ((x_ == s.x_) && (y_ == s.y_) && (dx_ < s.dx_)) || ((x_ == s.x_) && (y_ == s.y_) && (dx_ == s.dx_) && (dy_ < s.dy_)) ); }
  void print( std::ostream &os ) const { os << "(" << x_ << "," << y_ << "," << dx_ << "," << dy_ << ")"; }
  friend class problem_t;
};

inline std::ostream& operator<<( std::ostream &os, const state_t &s ) { s.print( os ); return( os ); }

class ecache_t : public hashing::hash_map<size_t,std::pair<state_t,state_t> > { };
//class ecache_t : public std::map<size_t,std::pair<state_t,state_t> > { };

class problem_t : public Problem::problem_t<state_t> {
  const grid_t &grid_;
  float p_;
  size_t rows_;
  size_t cols_;
  state_t init_;
  std::vector<state_t> inits_;
  std::vector<state_t> goals_;
  mutable ecache_t *ecache_[9];
public:
  problem_t( grid_t &grid, float p = 1.0 ) : grid_(grid), p_(p), rows_(grid.rows()), cols_(grid.cols()), init_(SHORT_MAX,SHORT_MAX,SHORT_MAX,SHORT_MAX)
  {
    for( size_t i = 0; i < grid_.starts().size(); ++i ) {
      size_t s = grid_.start( i );
      inits_.push_back( state_t( s / cols_, s % cols_ ) );
    }
    for( size_t i = 0; i < grid_.goals().size(); ++i ) {
      size_t s = grid_.goal( i );
      goals_.push_back( state_t( s / cols_, s % cols_ ) );
    }
    for( size_t i = 0; i < 9; ++i )
      ecache_[i] = new ecache_t[rows_*cols_];
  }
  virtual ~problem_t() { for( size_t i = 0; i < 9; ++i ) delete[] ecache_[i]; }
  virtual Problem::action_t last_action() const { return( 10 ); }
  virtual const state_t& init() const { return( init_ ); }
  virtual bool terminal( const state_t &s ) const { return( (s != init_) && grid_.goal_pos( s.x(), s.y() ) ); }
  virtual void next( const state_t &s, Problem::action_t a, std::pair<state_t,float> *outcomes, size_t &osize ) const
  {
    ++expansions_;
    size_t i = 0;
    if( s == init_ ) {
      for( size_t j = 0; j < inits_.size(); ++j )
        outcomes[i++] = std::make_pair( inits_[j], 1.0/(float)inits_.size() );
    }
    else {
      size_t off = s.x() * cols_ + s.y();
      size_t key = (((unsigned short)s.dx())<<16) | (unsigned short)s.dy();
      ecache_t::const_iterator ci = ecache_[a-1][off].find( key );
      if( ci != ecache_[a-1][off].end() ) {
        outcomes[i++] = std::make_pair( (*ci).second.first, p_ );
        outcomes[i++] = std::make_pair( (*ci).second.second, 1-p_ );
      }
      else {
        std::pair<state_t,state_t> entry;
        short ox = 0, oy = 0, ux = ((a-1)/3)-1, uy = ((a-1)%3)-1;
        if( p_ > 0.0 ) {
          int dx = s.dx() + ux, dy = s.dy() + uy;
          int x = s.x() + dx, y = s.y() + dy;
          int rv = grid_.valid_path( s.x(), s.y(), x, y, ox, oy );
          if( rv == 0 )
            entry.first = state_t( x, y, dx, dy );
          else
            entry.first = state_t( ox, oy, 0, 0 );
          outcomes[i++] = std::make_pair( entry.first, p_ );
        }
        if( 1-p_ > 0.0 ) {
          int dx = s.dx(), dy = s.dy();
          int x = s.x() + dx, y = s.y() + dy;
          int rv = grid_.valid_path( s.x(), s.y(), x, y, ox, oy );
          if( rv == 0 )
            entry.second = state_t( x, y, dx, dy );
          else
            entry.second = state_t( ox, oy, 0, 0 );
          outcomes[i++] = std::make_pair( entry.second, 1-p_ );
        }
        ecache_[a-1][off].insert( std::make_pair( key, entry ) );
      }
    }
    osize = i;
  }
  virtual void print( std::ostream &os ) const { }
};

void
usage( std::ostream &os )
{
  os << std::endl
     << "usage: race [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-k <n>] [-K <f>] [-p <f>] [-s <n>] [-v <n>] <file>"
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
     << "  <file>    Racetrack file."
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
  FILE *is = 0;
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
  if( argc == 1 ) {
    is = fopen( argv[0], "r" );
  }
  else {
    usage( std::cout );
    exit( -1 );
  }

  // build problem instances
  srand48( seed );
  grid_t grid;
  grid.parse( std::cout, is );
  problem_t problem( grid, p );
  fclose( is );

  // solve problem with algorithms
  size_t first = UINT_MAX;
  for( size_t i = 0; (i < 12) && (table[i] != 0); ++i ) {
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
      float atime = end_time - start_time - dtime;

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
                  << std::setprecision(9) << hash.value( problem.init() ) << " "
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

