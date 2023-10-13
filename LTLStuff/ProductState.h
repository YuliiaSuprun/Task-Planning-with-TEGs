#ifndef PRODUCT_STATE_H
#define PRODUCT_STATE_H

#include "GridState.h"

class ProductState {
public:
    ProductState(const GridState& grid_state, int dfa_state)
    : grid_state_(grid_state), dfa_state_(dfa_state) {}

    // Accessor methods
    GridState get_grid_state() const {
        return grid_state_;
    }

    int get_dfa_state() const {
        return dfa_state_;
    }

private:
    GridState grid_state_;
    int dfa_state_;
};

#endif // PRODUCT_STATE_H
