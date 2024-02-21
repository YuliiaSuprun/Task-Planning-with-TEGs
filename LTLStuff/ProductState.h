#ifndef PRODUCT_STATE_H
#define PRODUCT_STATE_H

#include <iostream>
#include <ostream>
#include "GridState.h"

class ProductState {
public:
    ProductState(const GridState& grid_state, size_t dfa_state)
    : grid_state_(grid_state), dfa_state_(dfa_state) {}

    // Accessor methods
    GridState get_grid_state() const {
        return grid_state_;
    }

    size_t get_dfa_state() const {
        return dfa_state_;
    }

    bool isCached() const { 
        return grid_state_.isCached();
    }

    void cache() { 
        grid_state_.cache();  
    }

    bool operator==(const ProductState& other) const {
        return grid_state_ == other.grid_state_ && dfa_state_ == other.dfa_state_;
    }

    // Need it for set.
    bool operator<(const ProductState& other) const {
        if (grid_state_ < other.grid_state_) {
            return true;
        } else if (grid_state_ == other.grid_state_ && dfa_state_ < other.dfa_state_) {
            return true;
        }
        return false;
    }

    friend std::ostream& operator<<(std::ostream& os, const ProductState& ps);

private:
    GridState grid_state_;
    size_t dfa_state_;
};

inline std::ostream& operator<<(std::ostream& os, const ProductState& ps) {
    os << "(" << ps.get_grid_state() << ", DFA: " << ps.get_dfa_state() << ")";
    return os;
}

#endif // PRODUCT_STATE_H
