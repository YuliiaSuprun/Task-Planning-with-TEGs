#ifndef GRID_STATE_H
#define GRID_STATE_H

#include <iostream>
#include <cmath>
#include "DomainState.h"
#include "GridAction.h"

class GridState : public DomainState {
public:
    GridState(size_t x = 0, size_t y = 0, bool isCached=false) : x_(x), y_(y), isCached_(isCached) {}

    // Overriding virtual functions from DomainState
    void cache() override {
        isCached_ = true;  
    }

    bool isCached() const override { 
        return isCached_;
    }

    // Apply a GridAction to the current state and return the resulting state.
    std::shared_ptr<DomainState> apply(const PrimitiveAction& action) const override {
        auto grid_action_ptr = dynamic_cast<const GridAction*>(&action);
        if (!grid_action_ptr) {
            std::cerr << "ERROR: invalid action. GridAction was expected." << std::endl;
            return nullptr;
        }
        return std::make_shared<GridState>(x_ + grid_action_ptr->deltaX(), y_ + grid_action_ptr->deltaY());
    }

    double distance(const DomainState& other) const override {
        auto other_grid_state_ptr = dynamic_cast<const GridState*>(&other);
        if (!other_grid_state_ptr) {
            std::cerr << "ERROR: invalid domain state. GridState was expected." << std::endl;
            return -1;
        }
        // We don't really have to take a square root.
        return euclideanDistanceSquared(*other_grid_state_ptr);
    }


    bool operator==(const DomainState& other) const override {
        auto otherState = dynamic_cast<const GridState*>(&other);
        if (!otherState) {
            return false;
        }
        return x_ == otherState->x_ && y_ == otherState->y_;
    }

    bool operator<(const DomainState& other) const override {
        auto otherState = dynamic_cast<const GridState*>(&other);
        if (!otherState) {
            return false;
        }
        if (x_ != otherState->x_) return x_ < otherState->x_;
        return y_ < otherState->y_;
    }

    size_t x() const { return x_; }
    size_t y() const { return y_; }

    void set_x(size_t x) { x_ = x; }
    void set_y(size_t y) { y_ = y; }


    // Calculate Euclidean distance to another GridState

    double euclideanDistanceSquared(const GridState& other) const {
        // We don't really have to take a square root.
        return std::pow(static_cast<double>(x_) - other.x_, 2) +
                         std::pow(static_cast<double>(y_) - other.y_, 2);
    }

    double euclideanDistance(const GridState& other) const {
        return std::sqrt(std::pow(static_cast<double>(x_) - other.x_, 2) +
                         std::pow(static_cast<double>(y_) - other.y_, 2));
    }

    size_t manhattanDistance(const GridState& other) const {
        return std::abs(static_cast<int>(x_) - static_cast<int>(other.x_)) +
               std::abs(static_cast<int>(y_) - static_cast<int>(other.y_));
    }

protected:
    std::ostream& print(std::ostream& os) const override {
        os << "[" << x_ << ", " << y_ << "]";
        return os;
    }


private:
    size_t x_;
    size_t y_;
    bool isCached_;
};


#endif // GRID_STATE_H
