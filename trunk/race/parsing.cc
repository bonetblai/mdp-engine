#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "parsing.h"

#define BOXW    10

namespace ps {

void
header( std::ostream &os, size_t rows, size_t cols )
{
  os << "%!PS-Adobe-2.0 EPSF-1.2" << std::endl
     << "%%Creator: unknown" << std::endl
     << "%%DocumentFonts:" << std::endl
     << "%%Pages: 1" << std::endl
     << "%%BoundingBox: 0 0 " << (cols+2)*BOXW << " " << (rows+2)*BOXW << std::endl
     << "%%EndComments" << std::endl
     << "%%EndProlog" << std::endl
     << "%%Page: 1 1" << std::endl
     << "0 setlinewidth " << BOXW << " " << BOXW << " translate" << std::endl;
}

void
trailer( std::ostream &os )
{
  os << "%%EOF" << std::endl;
}

void
draw_filled_rectangle( std::ostream &os, int x, int y, int dx, int dy, int r, int g, int b )
{
  os << r << " " << g << " " << b << " setrgbcolor" << std::endl
     << "newpath " << x << " " << y << " moveto "
     << dx << " 0 rlineto 0 " << dy << " rlineto "
     << -dx << " 0 rlineto closepath fill" << std::endl
     << "0 0 0 setrgbcolor" << std::endl
     << "newpath " << x << " " << y << " moveto "
     << dx << " 0 rlineto 0 " << dy << " rlineto " << -dx
     << " 0 rlineto closepath stroke" << std::endl;
}

void
draw_thick_line( std::ostream &os, float x, float y, int dx, int dy )
{
  os << "gsave 0 0 0 setrgbcolor 2 setlinewidth" << std::endl
     << "newpath " << x << " " << y << " moveto "
     << dx << " " << dy << " rlineto closepath stroke grestore" << std::endl;
}

void
draw_text( std::ostream &os, float x, float y, float angle, int ptsz, char *text )
{
  os << "0 0 0 setrgbcolor" << std::endl
     << "/Helvetica " << ptsz << " selectfont" << std::endl
     << x << " " << y << " moveto " << angle << " rotate" << std::endl
     << "(" << text << ") show" << std::endl;
}

}; // namespace ps

void
grid_t::parse( std::ostream &os, FILE *is )
{
  char buff[1024];
  if( fgets( buff, sizeof(buff), is ) == NULL ) {
    os << "error reading file header" << std::endl;
    throw( 0 );
  }
  else {
    sscanf( buff, "dim: %u %u", &rows_, &cols_ );
    if( (rows_ == 0) || (cols_ == 0) ) throw( 0 );
    assert( (size_t)cols_ <= sizeof(buff) );
    map_ = new char[rows_*cols_];
  }

  // parse file
  for( size_t r = 0; r < (size_t)rows_; ++r ) {
    if( fgets( buff, sizeof(buff), is ) == NULL ) {
      os << " error while reading row " << r << std::endl;
      throw( 0 );
    }
    for( size_t c = 0; c < (size_t)cols_; ++c ) {
      switch( buff[c] ) {
        case 's':
          map_[r*cols_ + c] = Valid | Start;
          starts_.push_back( r*cols_ + c );
          break;
        case 'g':
          map_[r*cols_ + c] = Valid | Goal;
          goals_.push_back( r*cols_ + c );
          break;
        case '.':
          map_[r*cols_ + c] = Valid;
          break;
        case 'x':
          map_[r*cols_ + c] = Invalid;
          break;
        default:
          os << " error reading col " << c << " of row " << r << std::endl;
          throw( 0 );
      }
    }
  }
}

void
grid_t::dump( std::ostream &os, bool postscript ) const
{
  if( postscript ) ps::header( os, rows_, cols_ );

  for( size_t y = 0; y < (size_t)rows_; ++y ) {
    for( size_t x = 0; x < (size_t)cols_; ++x ) {
      switch( map_[y*cols_ + x] ) {
        case Valid:
          if( postscript )
            ps::draw_filled_rectangle( os, x*BOXW, (rows_-1-y)*BOXW, BOXW, BOXW, 255, 255, 255 );
          else
            os << '.';
          break;
        case (Valid|Start):
          if( postscript )
            ps::draw_filled_rectangle( os, x*BOXW, (rows_-1-y)*BOXW, BOXW, BOXW, 0, 255, 255 );
          else
            os << 's';
          break;
        case (Valid|Goal):
          if( postscript )
            ps::draw_filled_rectangle( os, x*BOXW, (rows_-1-y)*BOXW, BOXW, BOXW, 255, 0, 0 );
          else
            os << 'g';
          break;
        default:
          if( !postscript ) os << 'x';
          break;
      }

#if 0
      if( postscript ) {
        // draw outer thick frame
        if( map_[y*cols_+x] & Valid ) {
          if( (c == 0) || !(gridMap[r*cols + c - 1] & VALID) ) psDrawThickLine(x,(rows-1)*BOXW-y,0,BOXW);
          if( c == cols - 1 ) psDrawThickLine(x+BOXW,(rows-1)*BOXW-y,0,BOXW);
          if( (r == 0) || !(gridMap[(r-1)*cols + c] & VALID) ) psDrawThickLine(x-0.5,(rows-1)*BOXW-y+BOXW,BOXW+1,0);
          if( r == rows - 1 ) psDrawThickLine(x-0.5,(rows-1)*BOXW-y,BOXW+1,0);
        }
        else {
          if( (c != 0) && (gridMap[r*cols + c - 1] & VALID) ) psDrawThickLine(x,(rows-1)*BOXW-y,0,BOXW);
          if( (r != 0) && (gridMap[(r-1)*cols + c] & VALID) ) psDrawThickLine(x-0.5,(rows-1)*BOXW-y+BOXW,BOXW+1,0);
        }
      }
#endif
    }
    if( !postscript ) os << std::endl;
  }

  if( postscript )
    ps::trailer( os );
  else {
    os << "Valid positions = { ";
    for( size_t x = 0; x < (size_t)cols_; ++x ) {
      for( size_t y = 0; y < (size_t)rows_; ++y )
        if( valid_pos( x, y ) ) os << "(" << x << "," << y << ") ";
    }
    os << "}" << std::endl;
  }
}

