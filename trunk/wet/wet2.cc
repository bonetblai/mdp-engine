#include <iostream>
#include <iomanip>
#include <math.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <float.h>
#include <assert.h>

#define MAXOUTCOMES   10

#include "../common.h"

#define XVER          (version&0x1)
#define YVER          (version&0x2)
#define ZVER          (version&0x4)

namespace Utils {
  int verbosity = 0;
  float kappa_log = log( 2.0 );
};

int version = 0;
float kappa_table[] = { 0.0, 0.1, 0.3, 0.6 };

const Problem::action_t up = 1;
const Problem::action_t right = 2;
const Problem::action_t down = 3;
const Problem::action_t left = 4;
char *asymb[] = { "-", "^", ">", "v", "<" };

class state_t {
  unsigned s_;
protected:
  static size_t size_;
public:
  state_t( unsigned x, unsigned y ) : s_((x*size_)+y) { }
  state_t( unsigned s = 0 ) : s_(s) { }
  ~state_t() { }
  size_t hash() const { return( s_ ); }
  unsigned s() const { return( s_ ); }
  unsigned x() const { return( s_ / size_ ); }
  unsigned y() const { return( s_ % size_ ); }
  state_t move( Problem::action_t a ) const
  {
    switch( a ) {
      case 1:
        if( y() == 0 ) return( s() );
        return( state_t( x(), y()-1 ) );
        break;
      case 2:
        if( x() == size_ - 1 ) return( s() );
        return( state_t( x()+1, y() ) );
        break;
      case 3:
        if( y() == size_ - 1 ) return( s() );
        return( state_t( x(), y()+1 ) );
        break;
      case 4:
        if( x() == 0 ) return( s() );
        return( state_t( x()-1, y() ) );
        break;
    }
    return( s_ );
  }
  const state_t& operator=( const state_t &s ) { s_ = s.s_; return( *this ); }
  bool operator==( const state_t &s ) const { return( s_ == s.s_ ); }
  bool operator!=( const state_t &s ) const { return( s_ != s.s_ ); }
  bool operator<( const state_t &s ) const { return( s_ < s.s_ ); }
  void print( std::ostream &os ) const { os << "( " << x() << " , " << y() << " )"; }
  friend class problem_t;
};

size_t state_t::size_ = 0;
inline std::ostream& operator<<( std::ostream &os, const state_t &s ) { s.print( os ); return( os ); }

class problem_t : public Problem::problem_t<state_t> {
  size_t size_;
  state_t init_;
  state_t goal_;
  char *water_;
  float p_;
public:
  problem_t( size_t size, float p, const state_t &init = state_t(0,0), const state_t &goal = state_t(0,0) ) : size_(size), init_(init), goal_(goal), p_(p)
  {
    state_t::size_ = size_;
    water_ = new char[size_*size_];
    for( size_t x = 0; x < size_; ++x )
      for( size_t y = 0; y < size_; ++y )
        if( drand48() < p_ ) water_[(x*size_)+y] = 1+(lrand48()%(XVER?2:3));
  }
  virtual ~problem_t() { delete[] water_; }
  size_t size() const { return( size_ ); }
  unsigned water( size_t x, size_t y ) const { return( water_[(x*size_)+y] ); }
  const state_t& goal() const { return( goal_ ); }
  virtual Problem::action_t last_action() const { return( 5 ); }
  virtual const state_t& init() const { return( init_ ); }
  virtual bool terminal( const state_t &s ) const { return( s == goal_ ); }
  virtual void next( const state_t &s, Problem::action_t a, std::pair<state_t,float> *outcomes, size_t &osize ) const
  {
    ++expansions_;
    size_t i = 0;
    float e = kappa_table[water(s.x(),s.y())];
    float e2 = e*e;
    if( YVER ) {
      outcomes[i++] = std::make_pair( s.move(a), 1.0-e-e2 );                         // kappa = 0
      if( e - e2 > 0.0 ) {
        outcomes[i++] = std::make_pair( s.move(1+(a%4)), (e-e2)/2.0 );               // kappa = 1
        outcomes[i++] = std::make_pair( s.move(1+((a+2)%4)), (e-e2)/2.0 );           // kappa = 1
      }
      if( e2 > 0.0 ) outcomes[i++] = std::make_pair( s, 2*e2 );                      // kappa = 2
    }
    else if( ZVER ) {
      outcomes[i++] = std::make_pair( s.move(a), 1.0-e2 );                           // kappa = 0
      if( e2 > 0.0 ) {
        outcomes[i++] = std::make_pair( s.move(1+(a%4)), e2/2.0 );                   // kappa = 2
        outcomes[i++] = std::make_pair( s.move(1+((a+2)%4)), e2/2.0 );               // kappa = 2
      }
    }
    else {
      outcomes[i++] = std::make_pair( s.move(a), 1.0-e-e2 );                         // kappa = 0
      if( e - e2 > 0.0 ) {
        outcomes[i++] = std::make_pair( (s.move(a)).move(1+(a%4)), (e-e2)/4.0 );     // kappa = 1
        outcomes[i++] = std::make_pair( (s.move(a)).move(1+((a+2)%4)), (e-e2)/4.0 ); // kappa = 1
        outcomes[i++] = std::make_pair( s.move(1+(a%4)), (e-e2)/4.0 );               // kappa = 1
        outcomes[i++] = std::make_pair( s.move(1+((a+2)%4)), (e-e2)/4.0 );           // kappa = 1
      }
      if( e2 > 0.0 ) outcomes[i++] = std::make_pair( s, 2*e2 );                      // kappa = 2
    }
    osize = i;
  }
  virtual void print( std::ostream &os ) const
  {
    os << "size = " << size_ << std::endl
       << "init = " << init_ << std::endl
       << "goal = " << goal_ << std::endl;
    for( size_t x = 0; x < size_; ++x ) os << "--";
    os << "---" << std::endl;
    for( size_t y = 0; y < size_; ++y ) {
      os << "|";
      for( size_t x = 0; x < size_; ++x ) {
        os << " " << water(x,y);
      }
      os << " |" << std::endl;
    }
    for( size_t x = 0; x < size_; ++x ) os << "--";
    os << "---" << std::endl;
  }
  void print_solution( std::ostream &os, const Hash::hash_t<state_t> &hash ) const
  {
    for( size_t x = 0; x < size_; ++x ) os << "--";
    os << "---" << std::endl;
    for( size_t y = 0; y < size_; ++y ) {
      os << "|";
      for( size_t x = 0; x < size_; ++x ) {
        if( terminal( state_t(x,y) ) )
          os << " *";
        else {
          std::pair<Problem::action_t,float> p = hash.bestQValue( *this, state_t(x,y) );
          os << " " << asymb[p.first];
        }
      }
      os << " |" << std::endl;
    }
    for( size_t x = 0; x < size_; ++x ) os << "--";
    os << "---" << std::endl;
  }
};

inline std::ostream& operator<<( std::ostream &os, const problem_t &p ) { p.print( os ); return( os ); }

void
usage( std::ostream &os )
{
  os << "usage: wet [-a <n>] [-b <n>] [-e <f>] [-g <f>] [-h <n>] [-p <f>] [-s <n>] [-v <n>] [-X] [-Y|-Z] <size>" << std::endl;
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
  Algorithm::ldfs_driver<state_t>,
  Algorithm::ldfs_plus_driver<state_t>,
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
  float p = 0.0;
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
      case 'X':
      case 'Y':
      case 'Z':
        version += (1<<(int)((*argv)[1]-'X'));
        ++argv;
        --argc;
        if( ZVER && YVER ) {
          std::cout << "Y and Z cannot be used simultaneously." << std::endl;
          exit( -1 );
        }
        break;
    }
  }
  if( argc == 1 ) {
    size = strtoul( *argv, 0, 0 );
  }
  else {
    usage( std::cout );
    exit( -1 );
  }

  // build problem instances and heuristic
  srand48( seed );
  state_t init( lrand48()%size, lrand48()%size );
  state_t goal( lrand48()%size, lrand48()%size );
  problem_t problem( size, p, init, goal );
  if( Utils::verbosity >= 50 ) problem.print( std::cout );

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

