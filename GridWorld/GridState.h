#ifndef GRID_STATE_H
#define GRID_STATE_H

#include <iostream>
#include <cmath>
#include "GridAction.h"

class GridState {
public:
    GridState(size_t x = 0, size_t y = 0, bool isCached=false) : x_(x), y_(y), isCached_(isCached) {}

    size_t x() const { return x_; }
    size_t y() const { return y_; }

    bool isCached() const { 
        return isCached_;
    }

    void cache() {
        isCached_ = true;  
    }

    void set_x(size_t x) { x_ = x; }
    void set_y(size_t y) { y_ = y; }

    bool operator==(const GridState& other) const {
        return x_ == other.x_ && y_ == other.y_;
    }

    bool operator<(const GridState& other) const {
        if (x_ != other.x_) return x_ < other.x_;
        return y_ < other.y_;
    }

    friend std::ostream& operator<<(std::ostream& os, const GridState& gs);

    // Apply a GridAction to the current state and return the resulting state.
    GridState apply(const GridAction& action) const {
        return GridState(x_ + action.deltaX(), y_ + action.deltaY());
    }

    // Calculate Euclidean distance to another GridState
    double distance(const GridState& other) const {
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


private:
    size_t x_;
    size_t y_;
    bool isCached_;
};

inline std::ostream& operator<<(std::ostream& os, const GridState& gs) {
    os << "[" << gs.x() << ", " << gs.y() << "]";
    return os;
}


#endif // GRID_STATE_H
