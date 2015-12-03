/*
 *  Copyright (C) 2015 Universidad Simon Bolivar
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

#ifndef UTILS_H
#define UTILS_H

#include <cassert>
#include <iostream>
#include <map>

#include <sys/resource.h>
#include <sys/time.h>

//#define DEBUG

namespace Utils {

#if 0
extern float kappa_log;

inline size_t kappa_value(float p, float kl = kappa_log) {
    return (size_t)floor(-log(p) / kl);
}
#endif

inline float read_time_in_seconds() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (float)r_usage.ru_utime.tv_sec +
           (float)r_usage.ru_utime.tv_usec / (float)1000000;
}

template<typename T> inline T min(const T a, const T b) {
    return a <= b ? a : b;
}

template<typename T> inline T max(const T a, const T b) {
    return a >= b ? a : b;
}

template<typename T> inline T abs(const T a) {
    return a < 0 ? -a : a;
}

void split_request(const std::string &request, std::string &name, std::string &parameter_str) {
    size_t first = request.find_first_of("(");
    size_t last = request.find_last_of(")");
    assert(first != std::string::npos);
    assert(last != std::string::npos);

    name = request.substr(0, first);
    parameter_str = request.substr(1 + first, last - first - 1);
    //std::cout << "name=" << name << ", parameters=|" << parameter_str << "|" << std::endl;
}

void tokenize(const std::string &parameter_str, std::multimap<std::string, std::string> &parameters, char separator = ',') {
    for( size_t i = 0; i < parameter_str.size(); ++i ) {
        size_t first = i;
        int nesting_level = 0;
        while( i < parameter_str.size() ) {
            if( (parameter_str[i] == separator) && (nesting_level == 0) ) {
                break;
            } else if( parameter_str[i] == ')' ) {
                assert(nesting_level > 0);
                --nesting_level;
            } else if( parameter_str[i] == '(' ) {
                ++nesting_level;
            }
            ++i;
        }
        assert(nesting_level == 0);
        std::string par = parameter_str.substr(first, i - first);
        //std::cout << "parameter=|" << par << "|" << std::endl;
        size_t equal = par.find_first_of("=");
        assert(equal != std::string::npos);
        std::string key = par.substr(0, equal);
        std::string value = par.substr(equal + 1);
        //std::cout << "key=|" << key << "|, value=|" << value << "|" << std::endl;
        parameters.insert(std::make_pair(key, value));
    }
}

}; // end of namespace

#undef DEBUG

#endif

