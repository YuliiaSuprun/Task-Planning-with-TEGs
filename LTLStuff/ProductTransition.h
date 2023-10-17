#ifndef PRODUCT_TRANSITION_H
#define PRODUCT_TRANSITION_H

#include <iostream>
#include <ostream>

#include <spot/tl/formula.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twa/bddprint.hh>
#include <bddx.h>
#include "ProductState.h"

using namespace std;

class ProductTransition {
public:
    ProductTransition(bdd condition, const ProductState& in_state, const ProductState& out_state)
    : condition_(condition), in_state_(in_state), out_state_(out_state) {}

    // Accessor methods
    bdd condition() const {
        return condition_;
    }

    ProductState in_state() const {
        return in_state_;
    }

    ProductState out_state() const {
        return out_state_;
    }

    bool operator==(const ProductTransition& other) const {
        return in_state_ == other.in_state_ && out_state_ == other.out_state_;
    }

    // Need it for set.
    bool operator<(const ProductTransition& other) const {
        if (in_state_ < other.in_state_) {
            return true;
        } else if (in_state_ == other.in_state_ && out_state_ < other.out_state_) {
            return true;
        }
        return false;
    }

    friend ostream& operator<<(ostream& os, const ProductTransition& ps);

private:
    bdd condition_; 
    ProductState in_state_;
    ProductState out_state_;
};

inline std::ostream& operator<<(std::ostream& os, const ProductTransition& ps) {
    os << "(" << ps.in_state() << " X [" << ps.condition() << "]) -> " << ps.out_state() << endl;
    return os;
}

#endif // PRODUCT_TRANSITION_H
