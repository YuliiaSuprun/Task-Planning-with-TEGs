#ifndef GRID_STATE_H
#define GRID_STATE_H

class GridState {
public:
    GridState(int x = 0, int y = 0) : x_(x), y_(y) {}

    int x() const { return x_; }
    int y() const { return y_; }

    void set_x(int x) { x_ = x; }
    void set_y(int y) { y_ = y; }

    bool operator==(const GridState& other) const {
        return x_ == other.x_ && y_ == other.y_;
    }

    bool operator<(const GridState& other) const {
        if (x_ != other.x_) return x_ < other.x_;
        return y_ < other.y_;
    }

private:
    int x_;
    int y_;
};

#endif // GRID_STATE_H
