#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <random>
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
        create_empty_obstacle_matrix();
    } else {
        if (obstacle_matrix.size() != R_ || (R_ > 0 && obstacle_matrix[0].size() != C_)) {
            cerr << "ERROR: Obstacle matrix dimensions mismatch" << endl;
        }
        obstacle_matrix_ = obstacle_matrix;
    }
}

bool GridWorldDomain::is_obstacle(const GridState& state) const {
    size_t x = state.x();
    size_t y = state.y();
    return obstacle_matrix_[x][y];
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

void GridWorldDomain::create_empty_obstacle_matrix() {
    // Create an empty obstacle matrix.
    obstacle_matrix_.clear();
    obstacle_matrix_.resize(R_, vector<bool>(C_, false));
}

void GridWorldDomain::add_rectangle_obstacle(double rmin, double rmax, double cmin, double cmax) {
    size_t rstart = max(static_cast<size_t>(0), static_cast<size_t>(ceil(rmin * R_) - 1));
    size_t rend = ceil(rmax * R_);
    size_t cstart = max(static_cast<size_t>(0), static_cast<size_t>(ceil(cmin * C_) - 1));
    size_t cend = ceil(cmax * C_);
    
    for (size_t i = rstart; i < rend; ++i) {
        for (size_t j = cstart; j < cend; ++j) {
            obstacle_matrix_[i][j] = true;
        }
    }
}

void GridWorldDomain::create_border_obstacle_matrix() {
    add_rectangle_obstacle(0.1, 0.2, 0.1, 0.45);
    add_rectangle_obstacle(0.1, 0.2, 0.55, 0.9);
    add_rectangle_obstacle(0.8, 0.9, 0.1, 0.45);
    add_rectangle_obstacle(0.8, 0.9, 0.55, 0.9);
    add_rectangle_obstacle(0.1, 0.45, 0.1, 0.2);
    add_rectangle_obstacle(0.55, 0.9, 0.1, 0.2);
    add_rectangle_obstacle(0.1, 0.45, 0.8, 0.9);
    add_rectangle_obstacle(0.55, 0.9, 0.8, 0.9);

    // Close right entrance.
    add_rectangle_obstacle(0.4, 0.6, 0.1, 0.2);
    add_rectangle_obstacle(0.4, 0.6, 0.8, 0.9);

    // Corners.
    add_rectangle_obstacle(0, 0.11, 0.89, 1);
    add_rectangle_obstacle(0.89, 1, 0, 0.11);
}

void GridWorldDomain::create_corner_obstacle_matrix() {
    add_rectangle_obstacle(0.8, 0.9, 0.4, 0.9);
    add_rectangle_obstacle(0.4, 0.9, 0.8, 0.9);
}

void GridWorldDomain::create_blocks_obstacle_matrix() {
    add_rectangle_obstacle(0.6, 0.85, 0.6, 0.85);
    add_rectangle_obstacle(0.15, 0.4, 0.15, 0.4);
}

void GridWorldDomain::create_corridor_obstacle_matrix() {
    add_rectangle_obstacle(0.35, 0.45, 0.1, 0.45);
    add_rectangle_obstacle(0.35, 0.45, 0.55, 0.9);
    add_rectangle_obstacle(0.55, 0.65, 0.1, 0.45);
    add_rectangle_obstacle(0.55, 0.65, 0.55, 0.9);
    add_rectangle_obstacle(0.1, 0.45, 0.35, 0.45);
    add_rectangle_obstacle(0.55, 0.9, 0.35, 0.45);
    add_rectangle_obstacle(0.1, 0.45, 0.55, 0.65);
    add_rectangle_obstacle(0.55, 0.9, 0.55, 0.65);
}

void GridWorldDomain::create_random_obstacle_matrix(double density) {
    default_random_engine generator;
    uniform_real_distribution<double> distribution(0.0, 1.0);

    for (size_t i = 0; i < R_; ++i) {
        for (size_t j = 0; j < C_; ++j) {
            if (distribution(generator) < density) {
                obstacle_matrix_[i][j] = true;
            }
        }
    }
}

set<GridState> GridWorldDomain::get_obstacle_set() const {
    set<GridState> obstacles_set;
    for (size_t r = 0; r < obstacle_matrix_.size(); ++r) {
        for (size_t c = 0; c < obstacle_matrix_[r].size(); ++c) {
            if (obstacle_matrix_[r][c]) {  
                obstacles_set.insert(GridState(r, c));
            }
        }
    }
    return obstacles_set;
}
