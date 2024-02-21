#ifndef PRODUCT_TRANSITION_H
#define PRODUCT_TRANSITION_H

#include <iostream>
#include <ostream>
#include <memory>
#include <type_traits>

#include <spot/tl/formula.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twa/bddprint.hh>
#include <bddx.h>
#include "ProductState.h"
#include "Action.h"
#include "CompoundAction.h"

using namespace std;

class ProductTransition {
public:

    ProductTransition(const ProductState& in_state, const ProductState& out_state, bdd edge_cond, shared_ptr<Action> action, bool isCached=false)
    : in_state_(in_state), out_state_(out_state), dfa_edge_cond_(edge_cond), action_(action), isCached_(isCached) {}

    // Accessor methods
    bdd dfa_edge_condition() const {
        return dfa_edge_cond_;
    }

    shared_ptr<Action> action() const {
        return action_;
    }

    ProductState in_state() const {
        return in_state_;
    }

    ProductState out_state() const {
        return out_state_;
    }

    bool isCached() const { 
        return isCached_;
    }

    vector<ProductState> path(bool with_end = false) const { 
        if (isCached_) {
            // Use std::dynamic_pointer_cast for shared_ptr
            shared_ptr<const CompoundAction> compoundActionPtr = dynamic_pointer_cast<const CompoundAction>(action_);

            if (!compoundActionPtr) {
                cerr << "ERROR: compoundActionPtr is null!!!" << endl;
                return {};
            }

            return compoundActionPtr->cached_path(with_end);
        } else {
            // This is a primitive action.
            if (with_end) {
                return vector<ProductState> {in_state_, out_state_};
            } else {
                return vector<ProductState> {in_state_};
            }
        }
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

    ProductState in_state_;
    ProductState out_state_;
    bdd dfa_edge_cond_; 
    shared_ptr<Action> action_;
    bool isCached_;
};

inline std::ostream& operator<<(std::ostream& os, const ProductTransition& ps) {
    os << "(" << ps.in_state() << " X [" << ps.dfa_edge_condition() << "]) -> " << ps.out_state() << endl;
    return os;
}

#endif // PRODUCT_TRANSITION_H
