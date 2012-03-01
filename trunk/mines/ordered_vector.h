#ifndef ORDERED_VECTOR_H
#define ORDERED_VECTOR_H

#include <cassert>
#include <iostream>
#include <vector>
#include <string.h>

class ordered_vector_t {
    int *vector_;
    int capacity_;
    int size_;

    int binary_search(int e) const {
        int lb = 0, ub = size_ - 1, mid = (lb + ub) / 2;
        if( e < vector_[lb] ) {
            mid = lb - 1;
        } else if( vector_[ub] <= e ) {
            mid = ub;
        } else {
            assert(vector_[lb] <= e);
            assert(e < vector_[ub]);
            while( lb != mid ) {
                if( vector_[mid] <= e ) {
                    lb = mid;
                } else {
                    ub = mid;
                }
                mid = (lb + ub) / 2;
            }
            assert(vector_[mid] <= e);
            assert(e < vector_[mid+1]);
        }
        assert((mid == -1) || (vector_[mid] <= e));
        return mid;
    }

  public:
    ordered_vector_t() : vector_(0), capacity_(0), size_(0) { }
    explicit ordered_vector_t(const ordered_vector_t &vec)
      : vector_(0), capacity_(0), size_(0) {
        *this = vec;
    }
    ordered_vector_t(ordered_vector_t &&vec)
      : vector_(vec.vector_), capacity_(vec.capacity_), size_(vec.size_) {
        vec.vector_ = 0;
        vec.capacity_ = 0;
        vec.size_ = 0;
    }
    ~ordered_vector_t() { delete[] vector_; }

    void reserve(int new_capacity) {
        if( capacity_ < new_capacity ) {
            capacity_ = new_capacity;
            int *nvector = new int[capacity_];
            memcpy(nvector, vector_, size_ * sizeof(int));
            delete[] vector_;
            vector_ = nvector;
        }
    }

    void clear() { size_ = 0; }
    bool empty() const { return size_ == 0; }
    int size() const { return size_; }

    void insert(int e) {
        if( size_ == 0 ) {
            reserve(1);
            vector_[0] = e;
            ++size_;
        } else {
            int mid = binary_search(e);
            if( (mid == -1) || (vector_[mid] != e) ) {
                reserve(1 + capacity_);
                for( int i = size_ - 1; i > mid; --i ) {
                    vector_[1+i] = vector_[i];
                }
                vector_[mid+1] = e;
                ++size_;
            }
        }
    }

    // TODO: make this more efficient
    void insert(const ordered_vector_t &vec) {
        for( const_iterator it = vec.begin(); it != vec.end(); ++it ) {
            if( (size_ == 0) || (*it > vector_[size_-1]) ) {
                // copy the whole vector at the end
                int copysz = vec.size() - it.index();
                reserve(size_ + copysz);
                memcpy(&vector_[size_], &vec.vector_[it.index()], copysz * sizeof(int));
                return;
            } else {
                insert(*it);
            }
        }
    }

    void erase(int e) {
        int mid = binary_search(e);
        if( (mid != -1) && (vector_[mid] == e) ) {
            for( int i = mid; i < size_ - 1; ++i ) {
                vector_[i] = vector_[i+1];
            }
            --size_;
        }
    }

    void erase_ordered_indices(const std::vector<int> &indices) {
        int indices_sz = indices.size();
        if( indices_sz != 0 ) {
            int k = indices[0];
            for( int i = 0, j = k; j < size_; ) {
                while( (i < indices_sz) && (j == indices[i]) ) { ++j; ++i; }
                while( (j < size_) && ((i >= indices_sz) || (j < indices[i])) ) {
                    vector_[k++] = vector_[j++];
                }
                assert((j == size_) || (i < indices_sz));
                assert((j == size_) || (j == indices[i]));
            }
            assert(k == size_ - indices_sz);
            size_ = k;
        }
    }

    int operator[](int i) const { return vector_[i]; }
    const ordered_vector_t& operator=(const ordered_vector_t &vec) {
        reserve(vec.size_);
        memcpy(vector_, vec.vector_, vec.size_ * sizeof(int));
        size_ = vec.size_;
        return *this;
    }
    bool operator==(const ordered_vector_t &vec) const {
        if( size_ != vec.size_ )
            return false;
        else
            return memcmp(vector_, vec.vector_, size_ * sizeof(int)) == 0;
    }
    bool operator!=(const ordered_vector_t &vec) const {
        return *this == vec ? false : true;
    }

    struct const_iterator {
        const int *base_;
        const int *ptr_;
        const_iterator(const int *ptr) : base_(ptr), ptr_(ptr) { }
        ~const_iterator() { }
        const_iterator& operator++() {
            ++ptr_;
            return *this;
        }
        int operator*() const { return *ptr_; }
        bool operator==(const const_iterator &it) const {
            return ptr_ == it.ptr_;
        }
        bool operator!=(const const_iterator &it) const {
            return ptr_ != it.ptr_;
        }
        int index() const { return ptr_ - base_; }
    };
    const_iterator begin() const {
        return const_iterator(vector_);
    }
    const_iterator end() const {
        return const_iterator(&vector_[size_]);
    }

    void print(std::ostream &os) const {
        os << '{';
        for( int i = 0; i < size_; ++i ) {
            os << vector_[i] << ',';
        }
        os << '}';
    }
};

inline std::ostream& operator<<(std::ostream &os, const ordered_vector_t &vec) {
    vec.print(os);
    return os;
}

#endif

