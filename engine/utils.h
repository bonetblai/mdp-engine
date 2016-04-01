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

#include "random.h"

//#define DEBUG

namespace Utils {

extern bool g_use_colors;

#if 0 // kappa stuff
extern float kappa_log;

inline size_t kappa_value(float p, float kl = kappa_log) {
    return (size_t)floor(-log(p) / kl);
}
#endif

inline std::string normal() { return !g_use_colors ? "" : "\x1B[0m"; }
inline std::string red() { return !g_use_colors ? "" : "\x1B[31;1m"; }
inline std::string green() { return !g_use_colors ? "" : "\x1B[32;1m"; }
inline std::string yellow() { return !g_use_colors ? "" : "\x1B[33;1m"; }
inline std::string blue() { return !g_use_colors ? "" : "\x1B[34;1m"; }
inline std::string magenta() { return !g_use_colors ? "" : "\x1B[35;1m"; }
inline std::string cyan() { return !g_use_colors ? "" : "\x1B[36;1m"; }
inline std::string error() { return !g_use_colors ? "" : "\x1B[31;1merror: \x1B[0m"; }
inline std::string warning() { return !g_use_colors ? "" : "\x1B[35;1mwarning: \x1B[0m"; }
inline std::string internal_error() { return !g_use_colors ? "" : "\x1B[31;1minternal error: \x1B[0m"; }

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
#ifdef DEBUG
    std::cout << "split_request: name=" << name << ", parameters=|" << parameter_str << "|" << std::endl;
#endif
}

inline bool tokenize(const std::string &parameter_str, std::multimap<std::string, std::string> &parameters, char separator = ',') {
    for( size_t i = 0; i < parameter_str.size(); ++i ) {
        size_t first = i;
        int nesting_level = 0;
        while( i < parameter_str.size() ) {
            if( (parameter_str[i] == separator) && (nesting_level == 0) ) {
                break;
            } else if( parameter_str[i] == ')' ) {
                if( nesting_level <= 0 ) {
                    std::cout << Utils::error() << "tokenize: invalid input '" << parameter_str << "'" << std::endl;
                    return false;
                }
                --nesting_level;
            } else if( parameter_str[i] == '(' ) {
                ++nesting_level;
            }
            ++i;
        }
        if( nesting_level != 0 ) {
            std::cout << Utils::error() << "tokenize: invalid input '" << parameter_str << "'" << std::endl;
            return false;
        }
        std::string par = parameter_str.substr(first, i - first);
#ifdef DEBUG
        std::cout << "tokenize: parameter=|" << par << "|" << std::endl;
#endif
        size_t equal = par.find_first_of("=");
        if( equal == std::string::npos ) {
            std::cout << Utils::error() << "tokenize: invalid input '" << parameter_str << "'" << std::endl;
            return false;
        }
        std::string key = par.substr(0, equal);
        std::string value = par.substr(equal + 1);
#ifdef DEBUG
        std::cout << "tokenize: key=|" << key << "|, value=|" << value << "|" << std::endl;
#endif
        parameters.insert(std::make_pair(key, value));
    }
    return true;
}

inline void print_bits(std::ostream &os, unsigned bitmap, int n) {
    for( int i = n - 1; i >= 0; --i )
        os << ((bitmap >> i) & 1);
    os << std::flush;
}

template<typename T> inline uint32_t jenkins_one_at_a_time_hash(const std::vector<T> &key) {
    uint32_t hash = 0;
    for( size_t i = 0; i < key.size(); ++i ) {
        hash += key[i].hash();
        hash += (hash << 10);
        hash ^= (hash >> 6);
    }
    hash += (hash << 3);
    hash ^= (hash >> 11);
    hash += (hash << 15);
    return hash;
}

}; // end of namespace

#undef DEBUG

#endif

