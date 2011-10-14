/*
 *  Copyright (C) 2006 Universidad Simon Bolivar
 * 
 *  Permission is hereby granted to distribute this software for
 *  non-commercial research purposes, provided that this copyright
 *  notice is included with any such distribution.
 *  
 *  THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 *  EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE
 *  SOFTWARE IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU
 *  ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 *  
 *  Blai Bonet, bonet@ldc.usb.ve
 *
 */

#ifndef RANDOM_H
#define RANDOM_H

#include <iostream>
#include <cassert>
#include <stdlib.h>

//#define DEBUG

namespace Random {

inline void set_random_seeds() {
}

inline void seed_for_reals(int seed) {
}

inline void seed_for_integers(int seed) {
}

inline float real_zero_one() {
    return drand48();
}

inline unsigned uniform(unsigned max) {
    return lrand48() % max;
}

};

#undef DEBUG

#endif

