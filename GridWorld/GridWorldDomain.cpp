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

// Define a static map.
map<GridState, vector<SkillAction>> GridWorldDomain::skill_actions_ = {};

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

void GridWorldDomain::add_skill_action(GridState state, const SkillAction& action) {
    // operator[] will automatically insert a new pair with an empty vector
    // if the key doesn't exist.
    vector<SkillAction>& actions = skill_actions_[state];
    // SkillAction must have an equality operator defined.
    if (find(actions.begin(), actions.end(), action) == actions.end()) {
        GridWorldDomain::skill_actions_[state].push_back(action);
    } else {
        cout << "Warning: Attempted to add a duplicate SkillAction under the same label." << endl;
    }
}

void GridWorldDomain::print_all_skill_actions() {
    cout << "Printing all Skill-Actions" << endl;
    for (const auto& state_actions_pair : skill_actions_) {
        const auto& state = state_actions_pair.first;
        const auto& actions = state_actions_pair.second;

        if (!actions.empty()) {
            // Print out the labels
            cout << "Grid State: " << state << endl;
            cout << "\nActions: \n";
            
            // Print out the actions
            for (const auto& action : actions) {
                action.print_path();
                // You might want to print more details about each action here
            }
            cout << "-----------\n";
        }
    }
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

const vector<GridState>& GridWorldDomain::get_all_states() {
    if (all_domain_states_.empty()) {
        // Pre-allocate enough memory for the vector.
        all_domain_states_.reserve(R_ * C_);
        for (size_t r = 0; r < R_; ++r) {
            for (size_t c = 0; c < C_; ++c) {
                all_domain_states_.emplace_back(r, c);
            }
        }
    }
    return all_domain_states_;
}

const vector<GridAction>& GridWorldDomain::get_actions(const GridState& state) const {
    (void)state; // To avoid compiler's warnings.
    return actions_;
}

const vector<GridState> GridWorldDomain::get_successor_states(GridState& curr_state) {
    vector<GridState> successor_states;
    for (const auto& action : get_actions(curr_state)) {
        GridState next_state = curr_state.apply(action);
        if (is_valid_state(next_state)) {
            successor_states.push_back(next_state);
        }
    }
    return successor_states;
}

const vector<pair<GridState, GridAction>> GridWorldDomain::get_successor_state_action_pairs(GridState& curr_state) {

    vector<pair<GridState, GridAction>> successor_state_action_pairs;
    for (const auto& action : get_actions(curr_state)) {
        GridState next_state = curr_state.apply(action);
        if (is_valid_state(next_state)) {
            successor_state_action_pairs.emplace_back(next_state, action);
        }
    }
    return successor_state_action_pairs;
}