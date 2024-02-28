#ifndef GRID_ACTION_H
#define GRID_ACTION_H

#include "Action.h"

class GridAction : public Action {
public:
    GridAction(int deltaX = 0, int deltaY = 0) : deltaX_(deltaX), deltaY_(deltaY) {}

    int deltaX() const { return deltaX_; }
    int deltaY() const { return deltaY_; }

    void set_x(int x) { deltaX_ = x; }
    void set_y(int y) { deltaY_ = y; }

    bool operator==(const GridAction& other) const {
        return deltaX_ == other.deltaX_ && deltaY_ == other.deltaY_;
    }

    bool operator<(const GridAction& other) const {
        if (deltaX_ != other.deltaX_) return deltaX_ < other.deltaX_;
        return deltaY_ < other.deltaY_;
    }

private:
    int deltaX_;
    int deltaY_;
};

#endif // GRID_ACTION_H
