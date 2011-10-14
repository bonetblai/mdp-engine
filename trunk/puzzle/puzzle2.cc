#include <iostream>
#include <iomanip>
#include <math.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <float.h>
#include <assert.h>

#define MAXOUTCOMES   10

#include "../common.h"

namespace Utils {
  int verbosity = 0;
  float kappa_log = log( 2.0 );
};

const Problem::action_t up = 1;
const Problem::action_t left = 2;
const Problem::action_t down = 3;
const Problem::action_t right = 4;

class bits_t {
  size_t bits_;
public:
  bits_t( size_t bits ) : bits_(bits) { }
  ~bits_t() { }
  void print( std::ostream &os ) const { for( size_t i = 8*sizeof(size_t); i > 0; --i ) os << ((bits_>>(i-1))%2); }
};

inline std::ostream& operator<<( std::ostream &os, const bits_t &b ) { b.print( os ); return( os ); }

class state_t {
  size_t d0_;
  size_t d1_;
  size_t d2_;
public:
  state_t( size_t d0 = 0, size_t d1 = 0, size_t d2 = 0 ) : d0_(d0), d1_(d1), d2_(d2) { }
  state_t( const state_t &s ) : d0_(s.d0_), d1_(s.d1_), d2_(s.d2_) { }
  ~state_t() { }
  size_t hash() const { return( d1_ ^ d2_ ); }
  size_t rows() const { return( (d0_>>4) & 0xF ); }
  size_t cols() const { return( (d0_>>8) & 0xF ); }
  bool applicable( size_t rows, size_t cols, Problem::action_t a ) const
  {
    return( true );
    switch( a ) {
      case ::up: return( blank() >= cols );
      case ::right: return( blank()%cols < cols-1 );
      case ::down: return( blank()+cols < rows*cols );
      case ::left: return( blank()%cols > 0 );
    }
    return( false );
  }
  void set_tile( size_t t, size_t pos ) { size_t shift = (pos<8?pos<<2:(pos-8)<<2); if( pos < 8 ) { d1_ &= ~(0xF<<shift); d1_ |= t<<shift; } else { d2_ &= ~(0xF<<shift); d2_ |= t<<shift; } }
  void set_blank( size_t pos ) { d0_ &= ~(0xF); d0_ |= pos; set_tile( 0, pos ); }
  size_t blank() const { return( d0_ & 0xF ); }
  size_t tile( size_t pos ) const { size_t shift = (pos<8?pos<<2:(pos-8)<<2); return( pos<8 ? (d1_>>shift)&0xF : (d2_>>shift)&0xF ); }
  void up( size_t rows, size_t cols ) { size_t b = blank(); if( b >= cols ) { size_t p = b-cols, t = tile(p); set_tile(t,b); set_blank(p); } }
  void right( size_t rows, size_t cols ) { size_t b = blank(); if( b%cols < cols-1 ) { size_t p = b+1, t = tile(p); set_tile(t,b); set_blank(p); } }
  void down( size_t rows, size_t cols ) { size_t b = blank(); if( b+cols < rows*cols ) { size_t p = b+cols, t = tile(p); set_tile(t,b); set_blank(p); } }
  void left( size_t rows, size_t cols ) { size_t b = blank(); if( b%cols > 0 ) { size_t p = b-1, t = tile(p); set_tile(t,b); set_blank(p); } }
  void random_moves( size_t rows, size_t cols, size_t n = 0 ) { for( size_t i = 0; i < n; ++i ) { size_t m = lrand48()%4; if( m == 0 ) up(rows,cols); else if( m == 1 ) right(rows,cols); else if( m == 2 ) down(rows,cols); else left(rows,cols); } }
  void set_goal( size_t rows, size_t cols ) { d0_ = d1_ = d2_ = 0; d0_ |= (rows<<4); d0_ |= (cols<<8); set_blank( 0 ); for( size_t t = 0; t < rows*cols; ++t ) set_tile( t, 1+t ); }
  size_t manhattan() const { size_t sum = 0; for( size_t p = 0; p < rows()*cols(); ++p ) if( p != blank() ) { size_t t = tile(p); int r = ((1+t)/cols()) - (p/cols()), c = ((1+t)%cols()) - (p%cols()); sum += (r<0?-r:r) + (c<0?-c:c); } return( sum ); }
  const state_t& operator=( const state_t &s ) { d0_ = s.d0_; d1_ = s.d1_; d2_ = s.d2_; return( *this ); }
  bool operator==( const state_t &s ) const { return( (d0_ == s.d0_) && (d1_ == s.d1_) && (d2_ == s.d2_) ); }
  bool operator!=( const state_t &s ) const { return( (d0_ != s.d0_) || (d1_ != s.d1_) || (d2_ != s.d2_) ); }
  bool operator<( const state_t &s ) const { return( (d0_ < s.d0_) || ((d0_ == s.d0_) && (d1_ < s.d1_)) || ((d0_ == s.d0_) && (d1_ == s.d1_) && (d2_ < s.d2_)) ); }
  void print( std::ostream &os ) const { os << "|" << std::setw(2) << blank() << "|"; for( size_t i = 0; i < rows()*cols(); ++i ) os << tile(i) << (i<rows()*cols()-1?",":"|"); }
  friend class problem_t;
};

inline std::ostream& operator<<( std::ostream &os, const state_t &s ) { s.print( os ); return( os ); }

class problem_t : public Problem::problem_t<state_t> {
  size_t rows_;
  size_t cols_;
  float p_;
  state_t init_;
  state_t goal_;
public:
  problem_t( size_t rows, size_t cols, const state_t &init, float p = 1.0 ) : rows_(rows), cols_(cols), p_(p), init_(init) { goal_.set_goal( rows_, cols_ ); }
  virtual ~problem_t() { }
  virtual Problem::action_t last_action() const { return( 5 ); }
  virtual bool applicable( const state_t &s, Problem::action_t a ) const { return( s.applicable( rows_, cols_, a ) ); }
  virtual const state_t& init() const { return( init_ ); }
  virtual bool terminal( const state_t &s ) const { return( s == goal_ ); }
  virtual void next( const state_t &s, Problem::action_t a, std::pair<state_t,float> *outcomes, size_t &osize ) const
  {
    ++expansions_;
    size_t i = 0;
    if( p_ > 0 ) {
      outcomes[i++] = std::make_pair( s, p_ );
      if( a == up )
        outcomes[0].first.up( rows_, cols_ );
      else if( a == right )
        outcomes[0].first.right( rows_, cols_ );
      else if( a == down )
        outcomes[0].first.down( rows_, cols_ );
      else
        outcomes[0].first.left( rows_, cols_ );
    }
    if( 1-p_ > 0 ) {
        outcomes[i++] = std::make_pair( s, 1-p_ );
#if 0
      if( a != up ) {
        outcomes[i] = std::make_pair( s, (1-p_)/3 );
        outcomes[i++].first.up( rows_, cols_ ); 
      }
      if( a != right ) {
        outcomes[i] = std::make_pair( s, (1-p_)/3 );
        outcomes[i++].first.right( rows_, cols_ ); 
      }
      if( a != down ) {
        outcomes[i] = std::make_pair( s, (1-p_)/3 );
        outcomes[i++].first.down( rows_, cols_ ); 
      }
      if( a != left ) {
        outcomes[i] = std::make_pair( s, (1-p_)/3 );
        outcomes[i++].first.left( rows_, cols_ ); 
      }
#endif
    }
    osize = i;
  }
  virtual void print( std::ostream &os ) const { }
};

inline std::ostream& operator<<( std::ostream &os, const problem_t &p ) { p.print( os ); return( os ); }

class manhattan_t : public Heuristic::heuristic_t<state_t> {
public:
  manhattan_t() { }
  virtual ~manhattan_t() { }
  virtual float value( const state_t &s ) const { return( (float)s.manhattan() ); }
  virtual float s_time() const { return( 0 ); }
  virtual float d_time() const { return( 0 ); }
  virtual size_t size() const { return( 0 ); }
  virtual void dump( std::ostream &os ) const { }
};

void
usage( std::ostream &os )
{
  os << std::endl
     << "usage: puzzle [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-k <n>] [-K <f>] [-p <f>] [-s <n>] [-v <n>] <rows> <cols>"
     << std::endl << std::endl
     << "  -a <n>    Algorithm bitmask: 1=vi, 2=slrtdp, 4=ulrtdp, 8=blrtdp, 16=ilao, 32=plain-check, 64=elrtdp, 128=hdp, 256=ldfs."
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
     << "  <rows>    Rows <= ?."
     << std::endl
     << "  <cols>    Cols <= ?."
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
  state_t init;
  init.set_goal( rows, cols );
  init.random_moves( rows, cols, 500 );
  problem_t problem( rows, cols, init, p );

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
      else if( h == 3 )
        heuristic = new manhattan_t;

      if( i == first ) std::cout << "init: " << init << ", h=" << (!heuristic?0:heuristic->value(init)) << std::endl;

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

