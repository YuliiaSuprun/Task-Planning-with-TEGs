#ifndef GRID_ACTION_H
#define GRID_ACTION_H

#include "PrimitiveAction.h"

class GridAction : public PrimitiveAction {
public:
    GridAction(int deltaX = 0, int deltaY = 0) : deltaX_(deltaX), deltaY_(deltaY) {}

    int deltaX() const { return deltaX_; }
    int deltaY() const { return deltaY_; }

    void set_x(int x) { deltaX_ = x; }
    void set_y(int y) { deltaY_ = y; }

    bool operator==(const PrimitiveAction& other) const override {
        auto other_ptr = dynamic_cast<const GridAction*>(&other);
        if (!other_ptr) {
            return false;
        }
        return deltaX_ == other_ptr->deltaX_ && deltaY_ == other_ptr->deltaY_;
    }

    bool operator<(const PrimitiveAction& other) const override {
        auto other_ptr = dynamic_cast<const GridAction*>(&other);
        if (!other_ptr) {
            return false;
        }
        if (deltaX_ != other_ptr->deltaX_) return deltaX_ < other_ptr->deltaX_;
        return deltaY_ < other_ptr->deltaY_;
    }

private:
    int deltaX_;
    int deltaY_;
};

#endif // GRID_ACTION_H
