#ifndef GRID_STATE_H
#define GRID_STATE_H

class GridState {
public:
    GridState(size_t x = 0, size_t y = 0) : x_(x), y_(y) {}

    size_t x() const { return x_; }
    size_t y() const { return y_; }

    void set_x(size_t x) { x_ = x; }
    void set_y(size_t y) { y_ = y; }

    bool operator==(const GridState& other) const {
        return x_ == other.x_ && y_ == other.y_;
    }

    bool operator<(const GridState& other) const {
        if (x_ != other.x_) return x_ < other.x_;
        return y_ < other.y_;
    }

private:
    size_t x_;
    size_t y_;
};

#endif // GRID_STATE_H
