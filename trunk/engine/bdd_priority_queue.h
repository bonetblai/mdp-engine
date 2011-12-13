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

template<typename T, typename MAX_CMP_FN, typename MIN_CMP_FN> class bdd_priority_queue {
  protected:
    mutable MAX_CMP_FN max_cmpfn_;
    mutable MIN_CMP_FN min_cmpfn_;

    struct container_t {
        T element_;
        unsigned xref_;
        container_t *next_;
        container_t() : xref_(0), next_(0) { }
        container_t(const T &element) : element_(element), xref_(0), next_(0) { }
    };
    typedef bdd_priority_queue<T, MAX_CMP_FN, MIN_CMP_FN> base;

    const unsigned capacity_;
    mutable container_t *pool_;

    unsigned max_size_;
    vector<container_t*> max_array_;

    unsigned min_size_;
    vector<unsigned> min_array_;

    container_t* allocate_container(const T &element) const {
        if( pool_ == 0 ) {
            return new container_t(element);
        } else {
            container_t *p = pool_;
            pool_ = pool_->next_;
            p->element_ = element;
            p->next_ = 0;
            p->xref_ = 0;
            return p;
        }
    }

    bool is_leaf_in_max(unsigned index) const { return (index << 1) > max_size_; }
    bool is_leaf_in_min(unsigned index) const { return (index << 1) > min_size_; }

    bool check_max() const {
        // check heap property
        for( unsigned index = 1; index <= max_size_; ++index ) {
            unsigned left = index << 1, right = 1 + left;
            if( (left <= max_size_) && max_cmpfn_(max_array_[index]->element_, max_array_[left]->element_) ) {
                std::cout << "heap property violated at max_array[index=" << index << "]" << std::endl;
                return false;
            }
            if( (right <= max_size_) && max_cmpfn_(max_array_[index]->element_, max_array_[right]->element_) ) {
                std::cout << "heap property violated at max_array[index=" << index << "]" << std::endl;
                return false;
            }
        }

        // check invariants for xrefs
        for( unsigned index = 1; index <= max_size_; ++index ) {
            if( index != min_array_[max_array_[index]->xref_] ) {
                std::cout << "invariant violated for max_array[index=" << index << "]" << std::endl;
                return false;
            }
        }

        // all tests passed, declare it sound
        return true;
    }

    bool check_min() const {
        // check heap property in min_array
        for( unsigned index = 1; index <= min_size_; ++index ) {
            unsigned left = index << 1, right = 1 + left;
            //std::cout << min_size_ << " " << index << " " << left << " " << right << std::endl;
            //std::cout << min_size_ << " " << min_array_[index] << " " << min_array_[left] << " " << min_array_[right] << std::endl;
            if( (left <= min_size_) &&
                min_cmpfn_(max_array_[min_array_[index]]->element_, max_array_[min_array_[left]]->element_) ) {
                std::cout << "heap property violated at min_array[index=" << index << "]" << std::endl;
                return false;
            }
            if( (right <= min_size_) &&
                min_cmpfn_(max_array_[min_array_[index]]->element_, max_array_[min_array_[right]]->element_) ) {
                std::cout << "heap property violated at min_array[index=" << index << "]" << std::endl;
                return false;
            }
        }

        // check invariants for xrefs in min_array
        for( unsigned index = 1; index <= min_size_; ++index ) {
            if( index != max_array_[min_array_[index]]->xref_ ) {
                std::cout << "invariant violated for min_array[index=" << index << "]" << std::endl;
                return false;
            }
        }

        // all tests passed, declare it sound
        return true;
    }

    bool check() const { return check_max() && check_min(); }

    void push_max(unsigned index, container_t *container) {
        unsigned parent = index >> 1;
        while( (index > 1) && max_cmpfn_(max_array_[parent]->element_, container->element_) ) {
            max_array_[index] = max_array_[parent];
            min_array_[max_array_[index]->xref_] = index;
            index = parent;
            parent = parent >> 1;
        }
        max_array_[index] = container;
        push_min(1 + min_size_, index);
        ++min_size_;
    }

    void heapify_max(unsigned index) {
        assert((index > 0) && (index <= max_size_));
        unsigned left = index << 1, right = 1 + left, largest = 0;

        if( (left <= max_size_) && max_cmpfn_(max_array_[index]->element_, max_array_[left]->element_) ) {
            largest = left;
        } else {
            largest = index;
        }
        if( (right <= max_size_) && max_cmpfn_(max_array_[largest]->element_, max_array_[right]->element_) ) {
            largest = right;
        }

        if( largest != index ) {
            container_t *tmp = max_array_[largest];
            max_array_[largest] = max_array_[index];
            max_array_[index] = tmp;
            min_array_[max_array_[largest]->xref_] = largest;
            min_array_[max_array_[index]->xref_] = index;
            heapify_max(largest);
        }
    }

    void push_min(unsigned index, unsigned element) {
        unsigned parent = index >> 1;
        while( (index > 1) && min_cmpfn_(max_array_[min_array_[parent]]->element_, max_array_[element]->element_) ) {
            min_array_[index] = min_array_[parent];
            max_array_[min_array_[index]]->xref_ = index;
            index = parent;
            parent = parent >> 1;
        }
        min_array_[index] = element;
        max_array_[element]->xref_ = index;
    }

    void remove_from_min(unsigned index) {
        std::cout << "index=" << index << std::endl;
        assert(is_leaf_in_min(index)); // TOOD: not always as there can be many ties 
        if( index != min_size_ ) {
            push_min(index, min_array_[min_size_]);
        }
        --min_size_;
        assert(check_min());
    }

    void min_heapify(unsigned index) {
        assert((index > 0) && (index <= min_size_));
        unsigned left = index << 1, right = 1 + left, largest = 0;

        if( (left <= min_size_) &&
            min_cmpfn_(max_array_[min_array_[index]]->element_, max_array_[min_array_[left]]->element_) ) {
            largest = left;
        } else {
            largest = index;
        }
        if( (right <= min_size_) &&
            min_cmpfn_(max_array_[min_array_[largest]]->element_, max_array_[min_array_[right]]->element_) ) {
            largest = right;
        }

        if( largest != index ) {
            container_t *tmp = min_array_[largest];
            min_array_[largest] = min_array_[index];
            min_array_[index] = tmp;
            min_heapify(largest);
        }
    }

  public:
    bdd_priority_queue(unsigned capacity)
      : capacity_(capacity), pool_(0), max_size_(0), min_size_(0) {
        std::cout << "building..." << std::flush;
        max_array_ = vector<container_t*>(1+capacity_, 0);
        min_array_ = vector<unsigned>(1+capacity_, 0);
        clear();
        std::cout << "done" << std::endl;
    }
    ~bdd_priority_queue() {
        clear();
        while( pool_ != 0 ) {
            container_t *p = pool_->next_;
            delete pool_;
            pool_ = p;
        }
    }

    unsigned capacity() const { return capacity_; }
    unsigned size() const { return max_size_; }
    bool empty() const { return max_size_ == 0; }

    void clear() {
        std::cout << "clearing: size=" << max_size_ << "..." << std::flush;
        for( unsigned index = 1; index <= max_size_; ++index ) {
            max_array_[index]->next_ = pool_;
            pool_ = max_array_[index];
            max_array_[index] = 0;
        }
        max_size_ = 0;

        bzero(&min_array_[0], (1 + capacity_) * sizeof(unsigned));
        min_size_ = 0;

        assert(check_max());
        assert(check_min());
        std::cout << "done" << std::endl;
    }

    const T top() const {
        return max_array_[1]->element_;
    }

    bool push(T &element) {
        if( max_size_ < capacity_ ) {
            std::cout << "push:" << std::flush;
            std::cout << " <get-container>" << std::flush;
            container_t *container = allocate_container(element);
            std::cout << " <push-max>" << std::flush;
            push_max(1 + max_size_, container);
            ++max_size_;
            std::cout << " <check-max>" << std::flush;
            assert(check_max());
            std::cout << " <check-min>" << std::flush;
            assert(check_min());
            std::cout << " done" << std::endl;
            return true;
        } else {
            assert(0);
            return false;
        }
    }

    void pop() {
        assert((max_size_ > 0) && (max_size_ <= capacity_));
        std::cout << "pop:" << std::flush;
        std::cout << " <remove-from-min>" << std::flush;
        remove_from_min(max_array_[1]->xref_);
        max_array_[1]->next_ = pool_;
        pool_ = max_array_[1];
        max_array_[1] = max_array_[max_size_];
        --max_size_;
        std::cout << " <heapify-max>" << std::flush;
        if( max_size_ > 0 ) heapify_max(1);
        std::cout << " <check-max>" << std::flush;
        assert(check_max());
        std::cout << " <check-max>" << std::flush;
        assert(check_min());
        std::cout << " done" << std::endl;
    }
};

}; // end of namespace

#undef DEBUG

#endif

