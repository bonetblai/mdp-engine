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

#ifndef PARAMETERS_H
#define PARAMETERS_H

namespace Algorithm {

struct parameters_t {
    float epsilon_;

    struct vi_parameters_t {
        unsigned max_number_iterations_;
        vi_parameters_t() : max_number_iterations_(0) { }
    } vi;

    struct rtdp_parameters_t {
        unsigned bound_;
        unsigned max_number_steps_;
        float epsilon_greedy_;
        rtdp_parameters_t() : bound_(0), max_number_steps_(0), epsilon_greedy_(0) { }
    } rtdp;

    struct eval_parameters_t {
        unsigned max_number_steps_;
    } eval;

    parameters_t() : epsilon_(0) { }
};

}; // end of namespace Algorithm

#endif

