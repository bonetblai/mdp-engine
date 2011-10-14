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

const Problem::action_t onelfwd = 1;
const Problem::action_t onerfwd = 2;

class bits_t {
  size_t bits_;
public:
  bits_t( size_t bits ) : bits_(bits) { }
  ~bits_t() { }
  void print( std::ostream &os ) const { for( size_t i = 8*sizeof(size_t); i > 0; --i ) os << ((bits_>>(i-1))%2); }
};

inline std::ostream& operator<<( std::ostream &os, const bits_t &b ) { b.print( os ); return( os ); }

class state_t {
  size_t data1_;
  size_t data2_;
public:
  state_t() : data1_(0), data2_(0) { }
  state_t( const state_t &s ) : data1_(s.data1_), data2_(s.data2_) { }
  ~state_t() { }
  size_t hash() const { return( data1_ ^ data2_ ); }
  size_t depth() const { return( data1_>>26 ); }
  size_t branch1() const { return( data1_ & ~(63<<26) ); }
  std::pair<size_t,size_t> branch() const { return( std::make_pair(branch1(),data2_) ); }
  size_t onebits() const
  {
    size_t count = 0;
    std::pair<size_t,size_t> b = branch();
    for( size_t i = 0; i < 8*sizeof(size_t); ++i ) {
      count += (b.first%2);
      b.first = b.first>>1;
      count += (b.second%2);
      b.second = b.second>>1;
    }
    return( count );
  }
  void onelfwd( size_t n ) { if( depth() < n-1 ) { size_t d = depth()+1; data1_ = (d<<26)|branch1(); } }
  void onerfwd( size_t n ) { if( depth() < n-1 ) { size_t d = depth()+1; data1_ = (d<<26)|branch1(); if( depth() <= 32 ) data2_ ^= (1<<(depth()-1)); else data1_ ^= (1<<(depth()-33)); } }
  void onebwd() { if( depth() > 0 ) { if( depth() <= 32 ) data2_ &= ~(1<<(depth()-1)); else data1_ &= ~(1<<(depth()-33)); size_t d = depth()-1; data1_ = (d<<26)|branch1(); } }
  const state_t& operator=( const state_t &s ) { data1_ = s.data1_; data2_ = s.data2_; return( *this ); }
  bool operator==( const state_t &s ) const { return( (data1_ == s.data1_) && (data2_ == s.data2_) ); }
  bool operator!=( const state_t &s ) const { return( (data1_ != s.data1_) || (data2_ != s.data2_) ); }
  bool operator<( const state_t &s ) const { return( (data1_ < s.data1_) || ((data1_ == s.data1_) && (data2_ < s.data2_)) ); }
  void print( std::ostream &os ) const { bits_t b1(branch1()), b2(data2_); os << "( " << depth() << " , [" << branch1() << "|" << data2_ << "]:" << b1 << "|" << b2 << " )"; }
  friend class problem_t;
};

inline std::ostream& operator<<( std::ostream &os, const state_t &s ) { s.print( os ); return( os ); }

class problem_t : public Problem::problem_t<state_t> {
  size_t n_;
  float p_;
  float q_;
  float r_;
  state_t init_;
  hashing::hash_set_t<state_t> noisy_;
public:
  problem_t( size_t n, float p, float q = 0.0, float r = 0.0 ) : n_(n), p_(p), q_(q), r_(r) { fill_noisy_states(); }
  virtual ~problem_t() { }
  void fill_noisy_states()
  {
    if( r_ > 0.0 ) {
      Hash::hash_t<state_t> hash;
      Algorithm::generate_space( *this, hash );
      if( Utils::verbosity >= 20 ) std::cout << "state space = " << hash.size() << std::endl;
      for( Hash::hash_t<state_t>::iterator hi = hash.begin(); hi != hash.end(); ++hi ) {
        if( drand48() < r_ ) noisy_.insert( (*hi).first );
      }
      if( Utils::verbosity >= 20 ) std::cout << "noisy size = " << noisy_.size() << std::endl;
    }
  }
  bool noisy( const state_t &s ) const { return(r_>0.0?(noisy_.find(s)!=noisy_.end()):false); }
  virtual Problem::action_t last_action() const { return( 3 ); }
  virtual const state_t& init() const { return( init_ ); }
  virtual bool terminal( const state_t &s ) const { return( s.depth() == n_-1 ); }
  virtual void next( const state_t &s, Problem::action_t a, std::pair<state_t,float> *outcomes, size_t &osize ) const
  {
    ++expansions_;
    size_t i = 0;
    if( a == Problem::noop ) {
      outcomes[i++] = std::make_pair( s, 1.0 );
    }
    else {
      size_t j = 0;
      float p = pow( p_, 1+s.onebits() );
      float q = (noisy(s)?p*q_:0.0);
      if( p-q > 0 ) outcomes[i++] = std::make_pair( s, p-q );
      if( q > 0 ) outcomes[i++] = std::make_pair( s, q );
      if( 1-p > 0 ) outcomes[i++] = std::make_pair( s, 1-p );
      if( a == onelfwd ) {
        if( p-q > 0 ) outcomes[j++].first.onelfwd( n_ );
        if( q > 0 ) outcomes[j++].first.onerfwd( n_ );
      }
      else if( a == onerfwd ) {
        if( p-q > 0 ) outcomes[j++].first.onerfwd( n_ );
        if( q > 0 ) outcomes[j++].first.onelfwd( n_ );
      }
      if( 1-p > 0 ) outcomes[j++].first.onebwd();
    }
    osize = i;
  }
  virtual void print( std::ostream &os ) const { }
};

void
usage( std::ostream &os )
{
  os << "usage: tree [-a <n>] [-b <n>] [-e <f>] [-f] [-g <f>] [-h <n>] [-k <n>] [-K <f>] [-p <f>] [-q <f>] [-r <f>] [-s <n>] [-v <n>] <size>"
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
     << "  -h <n>    Heuristics: 0=zero, 1=minmin. Default: 0."
     << std::endl
     << "  -k <n>    Kappa consistency level. Default: 0."
     << std::endl
     << "  -K <f>    Used to define kappa measures. Default: 2."
     << std::endl
     << "  -p <f>    Parameter p in [0,1]. Default: 1."
     << std::endl
     << "  -q <f>    Parameter q in [0,1]. Default: 1/2."
     << std::endl
     << "  -r <f>    Parameter r in [0,1]. Default: 0."
     << std::endl
     << "  -s <n>    Random seed. Default: 0."
     << std::endl
     << "  -v <n>    Verbosity. Default: 0."
     << std::endl
     << "  <size>    Depth of tree <= 58."
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
  size_t size = 0;
  unsigned alg = 0;
  float eps = 0.0;
  float g = 0.0;
  int h = 0;
  float p = 1.0;
  float q = 0.5;
  float r = 0.0;
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
      case 'q':
        q = strtod( argv[1], 0 );
        argv += 2;
        argc -= 2;
        break;
      case 'r':
        r = strtod( argv[1], 0 );
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
    size = strtoul( *argv, 0, 0 );
  }
  else {
    usage( std::cout );
    exit( -1 );
  }

  // build problem instances
  srand48( seed );
  problem_t problem( size, p, q, r );

  // solve problem with algorithms
  size_t first = UINT_MAX;
  for( size_t i = 0; (i <= 12) && (table[i] != 0); ++i ) {
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

