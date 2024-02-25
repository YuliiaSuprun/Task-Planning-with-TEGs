#ifndef PRODUCT_STATE_H
#define PRODUCT_STATE_H

#include <iostream>
#include <ostream>
#include "GridState.h"

class ProductState {
public:
    ProductState(const GridState& domain_state, size_t dfa_state)
    : domain_state_(domain_state), dfa_state_(dfa_state) {}

    // Accessor methods
    GridState get_domain_state() const {
        return domain_state_;
    }

    size_t get_dfa_state() const {
        return dfa_state_;
    }

    bool isCached() const { 
        return domain_state_.isCached();
    }

    void cache() { 
        domain_state_.cache();  
    }

    bool operator==(const ProductState& other) const {
        return domain_state_ == other.domain_state_ && dfa_state_ == other.dfa_state_;
    }

    // Need it for set.
    bool operator<(const ProductState& other) const {
        if (domain_state_ < other.domain_state_) {
            return true;
        } else if (domain_state_ == other.domain_state_ && dfa_state_ < other.dfa_state_) {
            return true;
        }
        return false;
    }

    friend std::ostream& operator<<(std::ostream& os, const ProductState& ps);

private:
    GridState domain_state_;
    size_t dfa_state_;
};

inline std::ostream& operator<<(std::ostream& os, const ProductState& ps) {
    os << "(" << ps.get_domain_state() << ", DFA: " << ps.get_dfa_state() << ")";
    return os;
}

#endif // PRODUCT_STATE_H
