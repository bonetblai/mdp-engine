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

#ifndef RANDOM_H
#define RANDOM_H

#include <iostream>
#include <cassert>
#include <stdlib.h>

//#define DEBUG

namespace Random {

inline void set_seed(int seed) {
    unsigned short useed[3];
    useed[0] = useed[1] = useed[2] = seed;
    srand48((long int)seed);
    seed48(useed);
}

inline float _random_float() {
    float d = drand48();
#ifdef DEBUG
    std::cerr << "_random_float: " << d << std::endl;
#endif
    return d;
}

inline unsigned _random_unsigned() {
    int r = lrand48();
#ifdef DEBUG
    std::cerr << "_random_unsigned: " << r << std::endl;
#endif
    return r;
}

inline float real() {
    return _random_float();
}

inline unsigned random(unsigned max) {
    assert(max > 0);
    return max == 1 ? 0 : _random_unsigned() % max;
}

inline unsigned random(unsigned min, unsigned max) {
    assert(max - min > 0);
    return min + random(max - min);
}

inline float uniform() {
    return drand48();
}

inline float uniform(float a, float b) {
    assert(a <= b);
    return a + (b - a) * uniform();
}

inline int sample_from_distribution(int n, const float *cdf) {
#if 1 // do sampling in log(n) time
    assert(n > 0);
    if( n == 1 ) return 0;

    float p = uniform();
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
    float p = uniform();
    for( int i = 0; i < n; ++i )
        if( p < cdf[i] ) return i;
    return n - 1;
#endif
}

inline void stochastic_sampling(int n, const float *cdf, int k, std::vector<int> &indices) {
    indices.clear();
    indices.reserve(k);
    for( int i = 0; i < k; ++i ) {
        int index = sample_from_distribution(n, cdf);
        indices.push_back(index);
    }
    assert(k == int(indices.size()));
}

inline void stochastic_universal_sampling(int n, const float *cdf, int k, std::vector<int> &indices) {
    indices.clear();
    indices.reserve(k);
    float u = uniform() / float(k);
    for( int i = 0, j = 0; j < k; ++j ) {
        while( (i < n) && (u > cdf[i]) ) ++i;
        indices.push_back(i == n ? n - 1 : i);
        u += 1.0 / float(k);
    }
    assert(k == int(indices.size()));
}

// total divergence
inline float total_divergence(const std::vector<float> &p, const std::vector<float> &q) {
    assert(p.size() == q.size());
    float d = 0;
    for( int i = 0, isz = int(p.size()); i < isz; ++i )
        d += fabs(p[i] - q[i]);
    return d / 2;
}

// Kullback-Leibler divergence
inline float kl_divergence(const std::vector<float> &p, const std::vector<float> &q, bool symmetric) {
    assert(p.size() == q.size());
    float h_p = 0, h_pq = 0, h_q = 0, h_qp = 0;
    for( int i = 0, isz = int(p.size()); i < isz; ++i ) {
        if( p[i] > 0 ) h_p += p[i] * log2f(p[i]);
        if( p[i] > 0 ) h_pq += p[i] * log2f(q[i]);
        if( symmetric ) {
            if( q[i] > 0 ) h_q += q[i] * log2f(q[i]);
            if( q[i] > 0 ) h_qp += q[i] * log2f(p[i]);
        }
    }
    return (h_p - h_pq) + (h_q - h_qp);
}

// lambda divergence
inline float lambda_divergence(const std::vector<float> &p, const std::vector<float> &q, float lambda) {
    assert((lambda >= 0) && (lambda <= 1));
    assert(p.size() == q.size());
    std::vector<float> m(p);
    for( int i = 0, isz = int(p.size()); i < isz; ++i )
        m[i] = lambda * p[i] + (1 - lambda) * q[i];
    return lambda * kl_divergence(p, m, false) + (1 - lambda) * kl_divergence(q, m, false);
}

// Jensen-Shannon divergence
inline float js_divergence(const std::vector<float> &p, const std::vector<float> &q) {
    return lambda_divergence(p, q, 0.5);
}

};

#undef DEBUG

#endif

