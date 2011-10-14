#include <iostream>
#include <iomanip>
#include <ext/hash_map>
#include <ext/hash_set>
#include <list>
#include <map>
#include <set>
#include <vector>
#include <math.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <float.h>
#include <assert.h>

#define MILLION       1000000
#define MAXOUTCOMES   10

template<typename T> inline T min( const T a, const T b ) { return( a <= b ? a : b ); }
template<typename T> inline T max( const T a, const T b ) { return( a >= b ? a : b ); }
inline float time( float secs, float usecs ) { return( secs + usecs / MILLION ); }

class problem_t;
class state_t;
class hash_t;

int verbosity = 0;

typedef size_t action_t;
const action_t noop = 0;
const action_t onelfwd = 1;
const action_t onerfwd = 2;
const action_t first_action = 1;
const action_t last_action = 3;

char *aname[] = { "vi", "slrtdp", "ulrtdp", "blrtdp", "ilao", "elrtdp", "check", "hdp-i" };
size_t value_iteration( const problem_t&, hash_t&, float, size_t, float );
size_t standard_lrtdp( const problem_t&, hash_t&, float, size_t, float );
size_t uniform_lrtdp( const problem_t&, hash_t&, float, size_t, float );
size_t bounded_lrtdp( const problem_t&, hash_t&, float, size_t, float );
size_t improved_lao( const problem_t&, hash_t&, float, size_t, float );
size_t plain_check( const problem_t&, hash_t&, float, size_t, float );
size_t hdp_i( const problem_t&, hash_t&, float, size_t, float );
size_t (*algorithm[])( const problem_t&, hash_t&, float, size_t, float ) = {
  value_iteration,
  standard_lrtdp,
  uniform_lrtdp,
  bounded_lrtdp,
  improved_lao,
  standard_lrtdp,
  plain_check,
  //hdp_i,
  0
};

typedef std::pair<state_t,bool> (*sample_func_t)( const problem_t&, const hash_t&, const state_t&, const action_t& );

class bits_t {
  size_t bits_;
public:
  bits_t( size_t bits ) : bits_(bits) { }
  ~bits_t() { }
  void print( std::ostream &os ) const
  {
    for( size_t i = 8*sizeof(size_t); i > 0; --i ) os << ((bits_>>(i-1))%2);
  }
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
  bool operator<( const state_t &s ) const { return( (data1_ < s.data1_) || ((data1_ == s.data1_) && (data2_ < s.data2_)) ); }
  void print( std::ostream &os ) const { bits_t b1(branch1()), b2(data2_); os << "( " << depth() << " , [" << branch1() << "|" << data2_ << "]:" << b1 << "|" << b2 << " )"; }
  friend class problem_t;
};

inline std::ostream& operator<<( std::ostream &os, const state_t &s ) { s.print( os ); return( os ); }

class data_t {
  float value_;
  bool solved_;
  bool marked_;
  size_t visits_;
  action_t action_;
public:
  data_t( float value = 0, bool solved = false, bool marked = false, size_t visits = 0 ) : value_(value), solved_(solved), marked_(marked), visits_(visits), action_(noop) { }
  data_t* ptr() { return( this ); }
  float value() const { return( value_ ); }
  void update( float value ) { assert( value_ <= value ); value_ = value; }
  bool solved() const { return( solved_ ); }
  void solve() { solved_ = true; }
  bool marked() const { return( marked_ ); }
  void mark() { marked_ = true; }
  void unmark() { marked_ = false; }
  size_t visits() const { return( visits_ ); }
  void visit() { ++visits_; }
  void clear_visits() { visits_ = 0; }
  action_t action() const { return( action_ ); }
  void set_action( action_t action ) { action_ = action; }
  void print( std::ostream &os ) const { os << "( " << value_ << " , " << (solved_?1:0) << " , " << (marked_?1:0) << " , " << visits_ << " , " << action_ << " )"; }
};

inline std::ostream& operator<<( std::ostream &os, const data_t &d ) { d.print( os ); return( os ); }

namespace hashing = ::__gnu_cxx;
namespace __gnu_cxx {

  template<> class hash<state_t>
  {
  public:
    size_t operator()( const state_t &s ) const { return( s.hash() ); }
  };

  class hash_set_t : public hash_set<state_t> { };

  class hash_map_t : public hash_map<state_t,data_t>
  {
    const hash_map_t *h_;
    data_t* push( const state_t &s, const data_t &d ) { std::pair<iterator,bool> p = insert( std::make_pair( s, d ) ); return( (*p.first).second.ptr() ); }
    iterator lookup( const state_t &s ) { return( find( s ) ); }
    const_iterator lookup( const state_t &s ) const { return( find( s ) ); }
  public:
    hash_map_t( const hash_map_t *h = 0 ) : h_(h) { }
    virtual ~hash_map_t() { }
    data_t* data_ptr( const state_t &s ) { iterator di = lookup( s ); if( di == end() ) return( push( s, data_t(h_==0?0:h_->value(s)) ) ); else return( (*di).second.ptr() ); }
    float hvalue( const state_t &s ) const { return( h_->value( s ) ); }
    float value( const state_t &s ) const { const_iterator di = lookup( s ); return( di == end() ? (h_==0?0:h_->value(s)) : (*di).second.value() ); }
    void update( const state_t &s, float value ) { iterator di = lookup( s ); if( di == end() ) push( s, data_t(value,false,false) ); else (*di).second.update( value ); }
    bool solved( const state_t &s ) const { const_iterator di = lookup( s ); return( di == end() ? false : (*di).second.solved() ); }
    void solve( const state_t &s ) { iterator di = lookup( s ); if( di == end() ) push( s, data_t((h_==0?0:h_->value(s)),true,false) ); else (*di).second.solve(); }
    bool marked( const state_t &s ) const { const_iterator di = lookup( s ); return( di == end() ? false : (*di).second.marked() ); }
    void mark( const state_t &s ) { iterator di = lookup( s ); if( di == end() ) push( s, data_t((h_==0?0:h_->value(s)),false,true) ); else (*di).second.mark(); }
    void unmark( const state_t &s ) { iterator di = lookup( s ); if( di != end() ) (*di).second.unmark(); }
    void unmark_all() { for( iterator di = begin(); di != end(); ++di ) (*di).second.unmark(); }
    size_t visits( const state_t &s ) const { const_iterator di = lookup( s ); return( di == end() ? 0 : (*di).second.visits() ); }
    void visit( const state_t &s ) { iterator di = lookup( s ); if( di == end() ) push( s, data_t((h_==0?0:h_->value(s)),false,false,1) ); else (*di).second.visit(); }
    void clear_visits( const state_t &s ) { iterator di = lookup( s ); if( di != end() ) (*di).second.clear_visits(); }
    action_t action( const state_t &s ) const { const_iterator di = lookup( s ); if( di == end() ) return( noop ); else return( (*di).second.action() ); }
    void set_action( const state_t &s, const action_t &action ) { iterator di = lookup( s ); if( di != end() ) (*di).second.set_action( action ); }
    void dump( std::ostream &os ) const { for( const_iterator di = begin(); di != end(); ++di ) os << (*di).first << " : " << (*di).second << std::endl; }
  };

};

class problem_t {
  size_t n_;
  float p_;
  float q_;
  float r_;
  state_t init_;
  hashing::hash_set_t noisy_;
public:
  problem_t( size_t n, float p, float q = 0.0, float r = 0.0 ) : n_(n), p_(p), q_(q), r_(r) { fill_noisy_states(); }
  ~problem_t() { }
  const state_t& init() const { return( init_ ); }
  bool terminal( const state_t &s ) const { return( s.depth() == n_-1 ); }
  void fill_noisy_states();
  bool noisy( const state_t &s ) const { return(r_>0.0?(noisy_.find(s)!=noisy_.end()):false); }
  const std::vector<std::pair<state_t,float> >& next( const state_t &s, const action_t &a ) const
  {
    static std::vector<std::pair<state_t,float> > outcomes( MAXOUTCOMES );
    outcomes.clear();
    if( a == noop ) {
      outcomes.push_back( std::make_pair( s, 1.0 ) );
    }
    else {
      size_t i = 0;
      float p = pow( p_, 1+s.onebits() );
      float q = (noisy(s)?p*q_:0.0);
      if( p-q > 0 ) outcomes.push_back( std::make_pair( s, p-q ) );
      if( q > 0 ) outcomes.push_back( std::make_pair( s, q ) );
      if( 1-p > 0 ) outcomes.push_back( std::make_pair( s, 1-p ) );
      if( a == onelfwd ) {
        if( p-q > 0 ) outcomes[i++].first.onelfwd( n_ );
        if( q > 0 ) outcomes[i++].first.onerfwd( n_ );
      }
      else if( a == onerfwd ) {
        if( p-q > 0 ) outcomes[i++].first.onerfwd( n_ );
        if( q > 0 ) outcomes[i++].first.onelfwd( n_ );
      }
      if( 1-p > 0 ) outcomes[i++].first.onebwd();
      assert( i == outcomes.size() );
    }
    return( outcomes );
  }
  std::pair<state_t,bool> sample( const hash_t &hash, const state_t &s, const action_t &a ) const
  {
    const std::vector<std::pair<state_t,float> > &outcomes = next( s, a );
    float d = drand48();
    for( size_t i = 0; i < outcomes.size(); ++i ) {
      if( d < outcomes[i].second ) return( std::make_pair( outcomes[i].first, true ) );
      d -= outcomes[i].second;
    }
    return( std::make_pair( outcomes[0].first, true ) );
  }
  std::pair<state_t,bool> usample( const hash_t &hash, const state_t &s, const action_t &a ) const
  {
    const std::vector<std::pair<state_t,float> > &outcomes = next( s, a );
    return( std::make_pair( outcomes[lrand48()%outcomes.size()].first, true ) );
  }
  std::pair<state_t,bool> nsample( const hash_t &hash, const state_t &s, const action_t &a ) const;
  void print( std::ostream &os ) const;
  size_t policy_size( hash_t &hash, const state_t &s ) const;
  size_t policy_size_aux( hash_t &hash, const state_t &s ) const;
};

inline std::ostream& operator<<( std::ostream &os, const problem_t &p ) { p.print( os ); return( os ); }

class hash_t : public hashing::hash_map_t
{
  unsigned updates_;
public:
  hash_t( const hash_t *h = 0 ) : hashing::hash_map_t(h), updates_(0) { }
  virtual ~hash_t() { }
  unsigned updates() const { return( updates_ ); }
  void update( const state_t &s, float value ) { hashing::hash_map_t::update( s, value ); inc_updates(); }
  void inc_updates() { ++updates_; }
  virtual float QValue( const problem_t &problem, const state_t &s, const action_t &a ) const
  {
    float qv = 0;
    if( problem.terminal( s ) ) return( 0 );
    const std::vector<std::pair<state_t,float> > &outcomes = problem.next( s, a );
    for( size_t i = 0; i < outcomes.size(); ++i )
      qv += outcomes[i].second * value( outcomes[i].first );
    return( 1+qv );
  }
  std::pair<action_t,float> bestQValue( const problem_t &problem, const state_t &s ) const
  {
    action_t ba = noop;
    float bqv = FLT_MAX;
    for( action_t a = first_action; a < last_action; ++a ) {
      float qv = QValue( problem, s, a );
      if( qv < bqv ) {
        bqv = qv;
        ba = a;
      }
    }
    return( std::make_pair(ba,bqv) );
  }
};

class min_hash_t : public hash_t
{
public:
  min_hash_t( const hash_t *h = 0 ) : hash_t(h) { }
  virtual ~min_hash_t() { }
  virtual float QValue( const problem_t &problem, const state_t &s, const action_t &a ) const
  {
    float qv = FLT_MAX;
    if( problem.terminal( s ) ) return( 0 );
    const std::vector<std::pair<state_t,float> > &outcomes = problem.next( s, a );
    for( size_t i = 0; i < outcomes.size(); ++i ) {
      float v = value( outcomes[i].first );
      qv = min( qv, v );
    }
    return( 1+qv );
  }
};

class policy_graph_t {
  const problem_t &problem_;
  hash_t &hash_;
  size_t size_;
  std::vector<state_t> roots_;
  std::list<state_t> tips_;
  std::list<state_t> nodes_;
public:
  policy_graph_t( const problem_t &problem, hash_t &hash ) : problem_(problem), hash_(hash), size_(0) { }
  ~policy_graph_t() { }
  size_t size() const { return( size_ ); }
  void add_root( const state_t &s ) { roots_.push_back( s ); }
  const std::list<state_t>& tips() const { return( tips_ ); }
  const std::list<state_t>& nodes() const { return( nodes_ ); }
  void recompute()
  {
    size_ = 0;
    tips_.clear();
    nodes_.clear();

    std::list<state_t> open;
    for( std::vector<state_t>::iterator si = roots_.begin(); si != roots_.end(); ++si ) {
      data_t *dptr = hash_.data_ptr( *si );
      open.push_back( *si );
      dptr->solve();
      dptr->mark();
    }

    while( !open.empty() ) {
      state_t t = open.front();
      open.pop_front();
      data_t *dptr = hash_.data_ptr( t );

      if( problem_.terminal( t ) ) {
        dptr->set_action( noop );
        nodes_.push_front( t );
        ++size_;
      }
      else {
        bool unsolved = false;
        std::pair<action_t,float> p = hash_.bestQValue( problem_, t );
        assert( p.first != noop );
        dptr->set_action( p.first );
        const std::vector<std::pair<state_t,float> > &outcomes = problem_.next( t, p.first );
        for( size_t i = 0; i < outcomes.size(); ++i ) {
          if( !hash_.solved( outcomes[i].first ) ) {
            unsolved = true;
            break;
          }
        }

        ++size_;
        if( !unsolved ) {
          for( size_t i = 0; i < outcomes.size(); ++i ) {
            data_t *dptr = hash_.data_ptr( outcomes[i].first );
            if( !dptr->marked() ) {
              open.push_back( outcomes[i].first );
              dptr->solve();
              dptr->mark();
            }
          }
          nodes_.push_front( t );
        }
        else {
          tips_.push_back( t );
          dptr->set_action( noop );
        }
      }
    }
    for( std::list<state_t>::iterator si = nodes_.begin(); si != nodes_.end(); ++si ) hash_.unmark( *si );
    for( std::list<state_t>::iterator si = tips_.begin(); si != tips_.end(); ++si ) hash_.unmark( *si );
  }
  void postorder_dfs( const state_t &s, std::list<state_t> &visited )
  {
    std::list<state_t> open;
    open.push_back( s );
    hash_.mark( s );

    while( !open.empty() ) {
      state_t t = open.back();
      visited.push_front( t );
      open.pop_back();
      data_t *dptr = hash_.data_ptr( t );
      assert( dptr->solved() );

      if( dptr->action() != noop ) {
        const std::vector<std::pair<state_t,float> > &outcomes = problem_.next( t, dptr->action() );
        for( size_t i = 0; i < outcomes.size(); ++i ) {
          data_t *dptr = hash_.data_ptr( outcomes[i].first );
          if( !dptr->marked() ) {
            open.push_back( outcomes[i].first );
            dptr->mark();
          }
        }
      }
      else {
        std::pair<action_t,float> p = hash_.bestQValue( problem_, t );
        const std::vector<std::pair<state_t,float> > &outcomes = problem_.next( t, p.first );
        for( size_t i = 0; i < outcomes.size(); ++i )
          hash_.solve( outcomes[i].first );
      }
    }
    for( std::list<state_t>::iterator si = visited.begin(); si != visited.end(); ++si ) hash_.unmark( *si );
  }
  void update( std::list<state_t> &visited )
  {
    for( std::list<state_t>::iterator si = visited.begin(); si != visited.end(); ++si ) {
      std::pair<action_t,float> p = hash_.bestQValue( problem_, *si );
      hash_.update( *si, p.second );
    }
  }
};

void
problem_t::fill_noisy_states()
{
  void generate_space( const problem_t&, hash_t& );
  if( r_ > 0.0 ) {
    hash_t hash;
    generate_space( *this, hash );
    if( verbosity >= 20 ) std::cout << "state space = " << hash.size() << std::endl;
    for( hash_t::iterator hi = hash.begin(); hi != hash.end(); ++hi ) {
      if( drand48() < r_ ) noisy_.insert( (*hi).first );
    }
    if( verbosity >= 20 ) std::cout << "noisy size = " << noisy_.size() << std::endl;
  }
}

std::pair<state_t,bool>
problem_t::nsample( const hash_t &hash, const state_t &s, const action_t &a ) const
{
  bool label[MAXOUTCOMES];
  float m = 0;
  size_t n = 0;
  const std::vector<std::pair<state_t,float> > &outcomes = next( s, a );
  for( size_t i = 0; i < outcomes.size(); ++i ) {
    if( (label[i] = hash.solved( outcomes[i].first )) ) {
      m += outcomes[i].second;
      ++n;
    }
  }
  m = 1.0 - m;
  n = outcomes.size() - n;
  if( n == 0 ) return( std::make_pair( s, false ) );

  float d = drand48();
  for( size_t i = 0; i < outcomes.size(); ++i ) {
    if( !label[i] && ((n == 1) || (d <= outcomes[i].second/m)) ) {
      return( std::make_pair( outcomes[i].first, true ) );
    }
    else if( !label[i] ) {
      --n;
      d -= outcomes[i].second/m;
    }
  }
  assert( 0 );
  return( std::make_pair( s, false ) );
}

size_t
problem_t::policy_size( hash_t &hash, const state_t &s ) const
{
  hash.unmark_all();
  size_t size = policy_size_aux( hash, s );
  return( size );
}

size_t
problem_t::policy_size_aux( hash_t &hash, const state_t &s ) const
{
  std::vector<state_t> succ;

  size_t size = 0;
  if( !terminal(s) && !hash.marked(s) ) {
    hash.mark( s );
    std::pair<action_t,float> p = hash.bestQValue( *this, s );
    const std::vector<std::pair<state_t,float> > &outcomes = next( s, p.first );
    for( size_t i = 0; i < outcomes.size(); ++i )
      succ.push_back( outcomes[i].first );
    for( size_t i = 0; i < succ.size(); ++i )
      size += policy_size_aux( hash, succ[i] );
    ++size;
  }
  return( size );
}

void
generate_space( const problem_t &problem, hash_t &hash )
{
  std::list<state_t> open;
  open.push_back( problem.init() );
  hash.mark( problem.init() );
  if( verbosity >= 100 ) std::cout << "marking " << problem.init() << std::endl;
  while( !open.empty() ) {
    state_t t = open.front();
    open.pop_front();
    for( action_t a = first_action; a < last_action; ++a ) {
      const std::vector<std::pair<state_t,float> > &outcomes = problem.next( t, a );
      for( size_t i = 0; i < outcomes.size(); ++i ) {
        if( !hash.marked( outcomes[i].first ) ) {
          open.push_back( outcomes[i].first );
          hash.mark( outcomes[i].first );
          if( verbosity >= 100 ) std::cout << "marking " << outcomes[i].first << std::endl;
        }
      }
    }
  }
  hash.unmark_all();
}

size_t
value_iteration( const problem_t &problem, hash_t &hash, float epsilon, size_t bound, float )
{
  generate_space( problem, hash );
  if( verbosity >= 20 ) std::cout << "state space = " << hash.size() << std::endl;

  size_t iters = 0;
  float residual = epsilon + 1;
  while( residual > epsilon ) {
    if( iters >= bound ) break;
    residual = 0;
    for( hash_t::iterator hi = hash.begin(); hi != hash.end(); ++hi ) {
      float hv = (*hi).second.value();
      std::pair<action_t,float> p = hash.bestQValue( problem, (*hi).first );
      residual = max( residual, (float)fabs( p.second - hv ) );
      (*hi).second.update( p.second );
      hash.inc_updates();
      if( verbosity >= 100 ) std::cout << (*hi).first << " = " << (*hi).second << std::endl;
    }
    ++iters;
    if( verbosity >= 50 ) std::cout << "residual=" << residual << std::endl;
  }
  return( iters );
}

std::pair<state_t,bool>
standard_lrtdp_sample( const problem_t &problem, const hash_t &hash, const state_t &s, const action_t &a )
{
  return( problem.sample( hash, s, a ) );
}

std::pair<state_t,bool>
uniform_lrtdp_sample( const problem_t &problem, const hash_t &hash, const state_t &s, const action_t &a )
{
  return( problem.usample( hash, s, a ) );
}

std::pair<state_t,bool>
normalized_lrtdp_sample( const problem_t &problem, const hash_t &hash, const state_t &s, const action_t &a )
{
  return( problem.nsample( hash, s, a ) );
}

bool
check_solved( const problem_t &problem, hash_t &hash, const state_t &s, float epsilon )
{
  std::list<state_t> open, closed;

  data_t *dptr = hash.data_ptr( s );
  if( !dptr->solved() ) {
    open.push_back( s );
    dptr->mark();
  }

  bool rv = true;
  while( !open.empty() ) {
    state_t t = open.back();
    open.pop_back();
    closed.push_back( t );
    if( problem.terminal( t ) ) continue;

    std::pair<action_t,float> p = hash.bestQValue( problem, t );
    if( fabs( p.second - hash.value( t ) ) > epsilon ) {
      rv = false;
      continue;
    }

    const std::vector<std::pair<state_t,float> > &outcomes = problem.next( t, p.first );
    for( size_t i = 0; i < outcomes.size(); ++ i ) {
      data_t *dptr = hash.data_ptr( outcomes[i].first );
      if( !dptr->solved() && !dptr->marked() ) {
        open.push_back( outcomes[i].first );
        dptr->mark();
      }
    }
  }

  if( rv ) {
    while( !closed.empty() ) {
      if( verbosity >= 30 ) std::cout << "solving " << closed.back() << std::endl;
      hash.solve( closed.back() );
      closed.pop_back();
    }
  }
  else {
    while( !closed.empty() ) {
      std::pair<action_t,float> p = hash.bestQValue( problem, closed.back() );
      data_t *dptr = hash.data_ptr( closed.back() );
      dptr->update( p.second );
      dptr->unmark();
      closed.pop_back();
    }
  }
  return( rv );
}

size_t
lrtdp_trial( const problem_t &problem, hash_t &hash, const state_t &s, float epsilon, size_t bound, float greedy, sample_func_t sample )
{
  std::list<state_t> states;
  std::pair<state_t,bool> n;

  data_t *dptr = hash.data_ptr( s );
  states.push_back( s );
  dptr->visit();
  state_t t = s;

  if( verbosity >= 50 ) std::cout << "trial: begin" << std::endl;
  size_t steps = 1;
  while( !problem.terminal(t) && !dptr->solved() && (dptr->visits() <= bound) ) {
    if( verbosity >= 50 ) std::cout << "  " << t << " = " << dptr->value() << std::endl;
    std::pair<action_t,float> p = hash.bestQValue( problem, t );
    dptr->update( p.second );
    hash.inc_updates();
    if( drand48() < greedy )
      n = uniform_lrtdp_sample( problem, hash, t, p.first );
    else
      n = (*sample)( problem, hash, t, p.first );
    if( !n.second ) break;
    t = n.first;
    dptr = hash.data_ptr( t );
    states.push_back( t );
    dptr->visit();
    ++steps;
  }
  if( verbosity >= 50 ) std::cout << "trial: end" << std::endl;

  while( !states.empty() ) {
    hash.clear_visits( states.back() );
    bool solved = check_solved( problem, hash, states.back(), epsilon );
    states.pop_back();
    if( !solved ) break;
  }

  while( !states.empty() ) {
    hash.clear_visits( states.back() );
    states.pop_back();
  }

  return( steps );
}

size_t
lrtdp( const problem_t &problem, hash_t &hash, float epsilon, size_t bound, float greedy, sample_func_t sample )
{
  size_t steps = 0, trials = 0;
  const state_t &init = problem.init();
  while( !hash.solved( init ) ) {
    size_t s = lrtdp_trial( problem, hash, init, epsilon, bound, greedy, sample );
    steps = max( steps, s );
    ++trials;
  }
  //std::cout << "max-steps=" << steps << std::endl;
  //std::cout << "avg-steps=" << (float)steps/(float)trials << std::endl;
  return( trials );
}

size_t
standard_lrtdp( const problem_t &problem, hash_t &hash, float epsilon, size_t bound, float greedy )
{
  return( lrtdp( problem, hash, epsilon, bound, greedy, &standard_lrtdp_sample ) );
}

size_t
uniform_lrtdp( const problem_t &problem, hash_t &hash, float epsilon, size_t bound, float greedy )
{
  return( lrtdp( problem, hash, epsilon, bound, greedy, &uniform_lrtdp_sample ) );
}

size_t
bounded_lrtdp( const problem_t &problem, hash_t &hash, float epsilon, size_t bound, float greedy )
{
  return( lrtdp( problem, hash, epsilon, bound, greedy, &normalized_lrtdp_sample ) );
}

size_t
improved_lao( const problem_t &problem, hash_t &hash, float epsilon, size_t, float )
{
  std::list<state_t> visited;
  policy_graph_t graph( problem, hash );
  graph.add_root( problem.init() );
  graph.recompute();

  size_t iterations = 0;
loop:
  ++iterations;
  while( !graph.tips().empty() ) {
    visited.clear();
    graph.postorder_dfs( problem.init(), visited );
    graph.update( visited );
    graph.recompute();
  }

  // convergence test
  float residual = 1.0 + epsilon;
  while( residual > epsilon ) {
    residual = 0.0;
    for( std::list<state_t>::const_iterator si = graph.nodes().begin(); si != graph.nodes().end(); ++si ) {
      float hv = hash.value( *si );
      std::pair<action_t,float> p = hash.bestQValue( problem, *si );
      residual = max( residual, (float)fabs( p.second - hv ) );
      hash.update( *si, p.second );
    }
    graph.recompute();
    if( !graph.tips().empty() ) goto loop;
  }
  return( iterations );
}

size_t
plain_check( const problem_t &problem, hash_t &hash, float epsilon, size_t, float )
{
  size_t trials = 0;
  const state_t &init = problem.init();
  while( !hash.solved( init ) ) {
    check_solved( problem, hash, init, epsilon );
    ++trials;
  }
  return( trials );
}

void
diff_time( unsigned long &secs, unsigned long &usecs, struct timeval &t1, struct timeval &t2 )
{
  if( t1.tv_sec == t2.tv_sec ) {
    secs = 0;
    usecs = t2.tv_usec - t1.tv_usec;
  }
  else {
    secs = (t2.tv_sec - t1.tv_sec) - 1;
    usecs = (MILLION - t1.tv_usec) + t2.tv_usec;
    if( usecs > MILLION ) {
      ++secs;
      usecs = usecs % MILLION;
    }
  }
}

void
usage( std::ostream &os )
{
  os << "usage: tree [-a <n>] [-b <n>] [-e <f>] [-g <f>] [-h <n>] [-p <f>] [-q <f>] [-r <f>] [-s <n>] [-v <n>] <size>"
     << std::endl << std::endl
     << "  -a <n>    Algorithm bitmask: 1=vi, 2=slrtdp, 4=ulrtdp, 8=blrtdp, 16=ilao, 32=plain check."
     << std::endl
     << "  -b <n>    Visits bound for blrtdp. Default: 0."
     << std::endl
     << "  -e <f>    Epsilon. Default: 0."
     << std::endl
     << "  -g <f>    Parameter for epsilon-greedy. Default: 0."
     << std::endl
     << "  -h <n>    Heuristics: 0=zero, 1=minmin. Default: 0."
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
        verbosity = strtoul( argv[1], 0, 0 );
        argv += 2;
        argc -= 2;
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

  // build problem instances
  srand48( seed );
  problem_t prob( size, p, q, r );

  min_hash_t heuristics;
  if( h == 1 ) {
    value_iteration( prob, heuristics, 0, UINT_MAX, 0 );
  }
  if( verbosity >= 100 ) heuristics.dump( std::cout );

  // solve problem with algorithms
  for( size_t i = 0; (i < 10) && (algorithm[i] != 0); ++i ) {
    if( (alg>>i) % 2 ) {
      unsigned long secs, usecs;
      struct rusage start, stop;

      srand48( seed );
      getrusage( RUSAGE_SELF, &start );
      hash_t hash( &heuristics );
      size_t t = (*algorithm[i])( prob, hash, eps, (i==3?bound:UINT_MAX), (i==5?g:0.0) );
      getrusage( RUSAGE_SELF, &stop );
      diff_time( secs, usecs, start.ru_utime, stop.ru_utime );

      std::cout << std::setw(2) << (1<<i) << " "
                << std::setw(6) << aname[i] << " "
                << std::setprecision(9) << hash.value( prob.init() ) << " "
                << seed << " "
                << t << " "
                << hash.updates() << " "
                << heuristics.value( prob.init() ) << " "
                << hash.size() << " "
                << prob.policy_size( hash, prob.init() ) << " "
                << time( secs, usecs ) << std::endl;
    }
  }
  exit( 0 );
}

