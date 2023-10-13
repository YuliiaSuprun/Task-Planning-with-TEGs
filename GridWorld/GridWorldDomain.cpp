#include <iostream>
#include <stdexcept>
#include "GridWorldDomain.h"

GridWorldDomain::GridWorldDomain(size_t R, size_t C, 
                                 const vector<vector<bool>>& obstacle_matrix,
                                 const vector<GridAction>& actions) 
    : R_(R), C_(C) {
    if (actions.empty()) {
        initializeDefaultActions();
    } else {
        actions_ = actions;
    }
    
    if (obstacle_matrix.empty()) {
        initializeDefaultObstacleMatrix();
    } else {
        if (obstacle_matrix.size() != R_ || (R_ > 0 && obstacle_matrix[0].size() != C_)) {
            cerr << "ERROR: Obstacle matrix dimensions mismatch" << endl;
        }
        obstacle_matrix_ = obstacle_matrix;
    }
}

bool GridWorldDomain::is_valid_state(const GridState& state) const {
    size_t x = state.x();
    size_t y = state.y();
    return x >= 0 && y >= 0 && x < R_ && y < C_ && (!obstacle_matrix_[x][y]);
}

bool GridWorldDomain::is_valid_transition(const GridState& s1, const GridState& action, const GridState& s2) const {
    // always returns false for now
    (void)s1;
    (void)s2;
    (void)action;
    return false;
}

void GridWorldDomain::initializeDefaultActions() {
    actions_.push_back(GridAction(0, 1));
    actions_.push_back(GridAction(1, 1));
    actions_.push_back(GridAction(1, 0));
    actions_.push_back(GridAction(1, -1));
    actions_.push_back(GridAction(0, -1));
    actions_.push_back(GridAction(-1, -1));
    actions_.push_back(GridAction(-1, 0));
    actions_.push_back(GridAction(-1, 1));
}

void GridWorldDomain::initializeDefaultObstacleMatrix() {
    // Create a RxC matrix with all zeros (false).
    obstacle_matrix_.resize(R_, vector<bool>(C_, false));
}
