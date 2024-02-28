#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <random>
#include "GridWorldDomain.h"

GridWorldDomain::GridWorldDomain(size_t R, size_t C, 
                                 const vector<vector<bool>>& obstacle_matrix,
                                 const vector<shared_ptr<PrimitiveAction>>& actions) 
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

    mark_all_states_as_unexplored();
}

// Implementing virtual methods from Domain
const vector<shared_ptr<PrimitiveAction>>& GridWorldDomain::get_actions(const DomainState& state) const {
    (void)state; // To avoid compiler's warnings.
    return actions_;
}

const vector<shared_ptr<DomainState>>& GridWorldDomain::get_all_states() {
    if (all_domain_states_.empty()) {
        all_domain_states_.reserve(R_ * C_);
        for (size_t r = 0; r < R_; ++r) {
            for (size_t c = 0; c < C_; ++c) {
                all_domain_states_.emplace_back(make_shared<GridState>(r, c));
            }
        }
    }
    return all_domain_states_;
}

const vector<shared_ptr<DomainState>> GridWorldDomain::get_successor_states(const DomainState& curr_state) {
    vector<shared_ptr<DomainState>> successor_states;

    auto grid_state_ptr = dynamic_cast<const GridState*>(&curr_state);
    if (!grid_state_ptr) {
        cerr << "ERROR: Invalid DomainState. GridState was expected." << endl;
        return successor_states; 
    }

    for (const auto& action : get_actions(*grid_state_ptr)) {
        auto next_state_ptr = grid_state_ptr->apply(*action);
        if (is_valid_state(*next_state_ptr)) {
            successor_states.push_back(next_state_ptr);
        }
    }
    return successor_states;
}

const vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> GridWorldDomain::get_successor_state_action_pairs(const DomainState& curr_state) {
    vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> successor_state_action_pairs;

    auto grid_state_ptr = dynamic_cast<const GridState*>(&curr_state);
    if (!grid_state_ptr) {
        cerr << "ERROR: Invalid DomainState. GridState was expected." << endl;
        return successor_state_action_pairs; 
    }

    for (const auto& action : get_actions(*grid_state_ptr)) {
        auto next_state_ptr = grid_state_ptr->apply(*action);
        if (is_valid_state(*next_state_ptr)) {
            successor_state_action_pairs.emplace_back(next_state_ptr, action);
        }
    }
    return successor_state_action_pairs;
}

bool GridWorldDomain::is_valid_state(const DomainState& state) const {
    auto grid_state_ptr = dynamic_cast<const GridState*>(&state);
    if (!grid_state_ptr) {
        cerr << "ERROR: Invalid DomainState. GridState was expected." << endl;
    }
    size_t x = grid_state_ptr->x();
    size_t y = grid_state_ptr->y();
    return x >= 0 && y >= 0 && x < R_ && y < C_ && (!obstacle_matrix_[x][y]);
}

void GridWorldDomain::mark_as_explored(const DomainState& state) {
    auto grid_state_ptr = dynamic_cast<const GridState*>(&state);
    if (!grid_state_ptr) {
        cerr << "ERROR: Invalid DomainState. GridState was expected." << endl;
    }
    size_t x = grid_state_ptr->x();
    size_t y = grid_state_ptr->y();
    exploration_matrix_[x][y] = true;
}

void GridWorldDomain::mark_all_states_as_unexplored() {
    exploration_matrix_.clear();
    exploration_matrix_.resize(R_, vector<bool>(C_, false));
}


// Methods that are unique to GridWorldDomain


bool GridWorldDomain::is_obstacle(const GridState& state) const {
    size_t x = state.x();
    size_t y = state.y();
    return obstacle_matrix_[x][y];
}

bool GridWorldDomain::was_explored(const GridState& state) const {
    size_t x = state.x();
    size_t y = state.y();
    return exploration_matrix_[x][y];
}

bool GridWorldDomain::is_valid_transition(const GridState& s1, const GridState& action, const GridState& s2) const {
    // always returns false for now
    (void)s1;
    (void)s2;
    (void)action;
    return false;
}

void GridWorldDomain::initializeDefaultActions() {
    actions_.push_back(make_shared<GridAction>(0, 1));
    actions_.push_back(make_shared<GridAction>(1, 1));
    actions_.push_back(make_shared<GridAction>(1, 0));
    actions_.push_back(make_shared<GridAction>(1, -1));
    actions_.push_back(make_shared<GridAction>(0, -1));
    actions_.push_back(make_shared<GridAction>(-1, -1));
    actions_.push_back(make_shared<GridAction>(-1, 0));
    actions_.push_back(make_shared<GridAction>(-1, 1));
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