/*
 *  Copyright (c) 2011-2016 Universidad Simon Bolivar
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
#include <math.h>

#include <sys/resource.h>
#include <sys/time.h>

//#define DEBUG

namespace Utils {

#if 0 // kappa stuff
extern float kappa_log;

inline size_t kappa_value(float p, float kl = kappa_log) {
    return (size_t)floor(-log(p) / kl);
}
#endif

inline std::string normal() { return "\x1B[0m"; }
inline std::string red() { return "\x1B[31;1m"; }
inline std::string green() { return "\x1B[32;1m"; }
inline std::string yellow() { return "\x1B[33;1m"; }
inline std::string blue() { return "\x1B[34;1m"; }
inline std::string magenta() { return "\x1B[35;1m"; }
inline std::string cyan() { return "\x1B[36;1m"; }
inline std::string error() { return "\x1B[31;1merror: \x1B[0m"; }
inline std::string warning() { return "\x1B[35;1mwarning: \x1B[0m"; }
inline std::string internal_error() { return "\x1B[31;1minternal error: \x1B[0m"; }

inline float read_time_in_seconds() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    float time = (float)r_usage.ru_utime.tv_sec + (float)r_usage.ru_utime.tv_usec / (float)1000000;
    getrusage(RUSAGE_CHILDREN, &r_usage);
    time += (float)r_usage.ru_utime.tv_sec + (float)r_usage.ru_utime.tv_usec / (float)1000000;
    return time;
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

inline void split_request(const std::string &request, std::string &name, std::string &parameter_str) {
    size_t first = request.find_first_of("(");
    size_t last = request.find_last_of(")");
    if( (first != std::string::npos) && (last != std::string::npos) ) {
        name = request.substr(0, first);
        parameter_str = request.substr(1 + first, last - first - 1);
    } else {
        name = request;
    }
    //std::cout << "name=" << name << ", parameters=|" << parameter_str << "|" << std::endl;
}

inline void tokenize(const std::string &parameter_str, std::multimap<std::string, std::string> &parameters, char separator = ',') {
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

inline int sample_from_distribution(int n, const float *cdf) {
#if 1 // do sampling in log(n) time
    assert(n > 0);
    if( n == 1 ) return 0;

    float p = drand48();
    if( p == 1 ) { // border condition (it can only happens when converting drand48() to float)
        for( int i = n - 1; i > 0; --i ) {
            if( cdf[i - 1] < cdf[i] )
                return i;
        }
        assert(cdf[0] > 0);
        return 0;
    }
    assert(p < 1);

    int lower = 0;
    int upper = n;
    int mid = (lower + upper) >> 1;
    float lcdf = mid == 0 ? 0 : cdf[mid - 1];

    while( !(lcdf <= p) || !(p < cdf[mid]) ) {
        assert(lower < upper);
        assert((lower <= mid) && (mid < upper));
        if( lcdf > p ) {
            upper = mid;
        } else {
            assert(p >= cdf[mid]);
            lower = 1 + mid;
        }
        mid = (lower + upper) >> 1;
        lcdf = mid == 0 ? 0 : cdf[mid - 1];
    }
    assert((lcdf <= p) && (p < cdf[mid]));
    //std::cout << "    return: mid=" << mid << std::endl;
    return mid;
#else
    float p = drand48();
    for( int i = 0; i < n; ++i )
        if( p < cdf[i] ) return i;
    return n - 1;
#endif
}

inline void stochastic_sampling(int n, const float *cdf, int k, std::vector<int> &indices) {
    indices.clear();
    indices.reserve(k);
    for( int i = 0; i < k; ++i ) {
        int index = Utils::sample_from_distribution(n, cdf);
        indices.push_back(index);
    }
    assert(k == int(indices.size()));
}

inline void stochastic_universal_sampling(int n, const float *cdf, int k, std::vector<int> &indices) {
    indices.clear();
    indices.reserve(k);
    float u = drand48() / float(k);
    for( int i = 0, j = 0; j < k; ++j ) {
        while( (i < n) && (u > cdf[i]) ) ++i;
        indices.push_back(i == n ? n - 1 : i);
        u += 1.0 / float(k);
    }
    assert(k == int(indices.size()));
}

}; // end of namespace

#undef DEBUG

#endif

