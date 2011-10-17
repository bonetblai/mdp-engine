/*
 *  Copyright (C) 2011 Universidad Simon Bolivar
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

#ifndef UCT_H
#define UCT_H

#include "policy.h"

#include <iostream>
#include <cassert>
#include <limits>
#include <vector>

//#define DEBUG

namespace Policy {

template<typename T> class uct_t : public improvement_t<T> {
  protected:

  public:
    uct_t(const Problem::problem_t<T> &problem, const policy_t<T> &base_policy)
      : improvement_t<T>(problem, base_policy) {
    }
    virtual ~uct_t() { }
    virtual Problem::action_t operator()(const T &s) const {
        return 0;
    }
};

}; // namespace Policy

#undef DEBUG

#endif

