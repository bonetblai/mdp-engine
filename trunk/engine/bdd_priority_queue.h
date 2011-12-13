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

#ifndef BDD_PRIORITY_QUEUE_H
#define BDD_PRIORITY_QUEUE_H

#include <iostream>
#include <cassert>
#include <vector>

//#define DEBUG

namespace std {

template<typename T, typename CMP> class bdd_priority_queue {
  protected:
    unsigned capacity_;
    unsigned size_;
    CMP cmpfn_;

    struct container_t {
        T element_;
        container_t *next_;
        container_t(const T &element) : element_(element), next_(0) { }
    };
    typedef bdd_priority_queue<T, CMP> base;

    mutable container_t *pool_;
    vector<container_t*> array_;
    unsigned min_;

    void heapify(unsigned index) {
        assert((index > 0) && (index <= size_));
        std::cout << "heapify(" << index << ")" << std::endl;
        unsigned left = index << 1, right = 1 + left, largest = 0;
        std::cout << "  size=" << size_ << ", left=" << left << ", right=" << right << std::endl;
        std::cout << "  A[index]=" << array_[index]->element_ << std::endl;
        if( left <= size_ ) std::cout << "  A[left]=" << array_[left]->element_ << std::endl;
        if( right <= size_ ) std::cout << "  A[right]=" << array_[right]->element_ << std::endl;
        if( (left <= size_) && cmpfn_(array_[index]->element_, array_[left]->element_) ) {
            largest = left;
        } else {
            largest = index;
        }
        if( (right <= size_) && cmpfn_(array_[index]->element_, array_[right]->element_) ) {
            largest = right;
        }
        std::cout << "  largest=" << largest << std::endl;
        if( largest != index ) {
            container_t *tmp = array_[largest];
            array_[largest] = array_[index];
            array_[index] = tmp;
            heapify(largest);
        }
    }

  public:
    bdd_priority_queue(unsigned capacity) : capacity_(capacity), pool_(0), min_(0) {
        std::cout << "hola: capacity=" << capacity_ << std::endl;
        array_ = vector<container_t*>(1+capacity_, 0);
        clear();
    }
    ~bdd_priority_queue() { }

    unsigned capacity() const { return capacity_; }
    unsigned size() const { return size_; }
    bool empty() const { return size_ == 0; }

    void clear() {
        for( unsigned index = 1; index <= size_; ++index ) {
            array_[index]->next_ = pool_;
            pool_ = array_[index];
            array_[index] = 0;
        }
        size_ = 0;
        min_ = 0;
    }

    const T top() const { return array_[1]->element_; }

    bool push(T &element) {
        if( size_ < capacity_ ) {
            std::cout << "size=" << size_ << std::endl;
            container_t *container = 0;
            if( pool_ == 0 ) {
                container = new container_t(element);
            } else {
                container = pool_;
                pool_ = pool_->next_;
                container->element_ = element;
                container->next_ = 0;
            }
            //std::cout << "element=" << element << ", container=" << container << std::endl;

            unsigned index = 1+size_, parent = index >> 1;
            //std::cout << "index=" << index << ", A[parent]=" << array_[parent] << std::endl;
            while( (index > 1) && cmpfn_(array_[parent]->element_, element) ) {
                //std::cout << "index=" << index << ", A[parent]=" << array_[parent] << std::endl;
                array_[index] = array_[parent];
                index = parent;
                parent = parent >> 1;
            }
            assert((index > 0) && (index <= 1+size_));
            array_[index] = container;
            ++size_;
            std::cout << "A[" << index << "]=" << array_[index] << std::endl;

            // update min
            std::cout << "min=" << min_ << std::endl;
            if( (min_ == 0) || cmpfn_(element, array_[min_]->element_) )
                min_ = index;
                
            return true;
        } else {
            return false;
        }
    }

    void pop() {
        assert((size_ > 0) && (size_ <= capacity_));
        std::cout << "pop: size=" << size_ << std::flush << ", array[size]=" << array_[size_] << std::endl;
        array_[1]->next_ = pool_;
        pool_ = array_[1];
        array_[1] = array_[size_];
        --size_;
        if( size_ > 0 ) heapify(1);
    }
};

}; // end of namespace

#undef DEBUG

#endif

