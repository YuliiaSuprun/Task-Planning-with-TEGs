#ifndef GRID_WORLD_DOMAIN_H
#define GRID_WORLD_DOMAIN_H

#include <vector>
#include <set>
#include <algorithm> 
#include <map>
#include "GridState.h"
#include "GridAction.h"
#include "SkillAction.h"

using namespace std;

class GridWorldDomain {
public:
    GridWorldDomain(size_t R, size_t C, 
                    const vector<vector<bool>>& obstacle_matrix = {},
                    const vector<GridAction>& actions = {});
    
    // Getters.
    size_t R() const { return R_; }
    size_t C() const { return C_; }
    const vector<vector<bool>>& get_obstacle_matrix() const { return obstacle_matrix_; }
    const vector<GridAction>& get_actions(const GridState& state) const;
    const vector<SkillAction>& get_skill_actions(GridState state) { return skill_actions_[state]; }
    set<GridState> get_obstacle_set() const;

    const vector<GridState>& get_all_states();
    const vector<GridState> get_successor_states(GridState& curr_state);
    const vector<pair<GridState, GridAction>> get_successor_state_action_pairs(GridState& curr_state);


    bool is_obstacle(const GridState& state) const;
    bool is_valid_state(const GridState& state) const;
    bool is_valid_transition(const GridState& s1, const GridState& action, const GridState& s2) const;

    void add_skill_action(GridState state, const SkillAction& action);

    void print_all_skill_actions();

    // Functions for creating obstacle matrices.
    void add_rectangle_obstacle(double rmin, double rmax, double cmin, double cmax);
    void create_empty_obstacle_matrix();
    void create_border_obstacle_matrix();
    void create_corner_obstacle_matrix();
    void create_blocks_obstacle_matrix();
    void create_corridor_obstacle_matrix();
    void create_random_obstacle_matrix(double density);

    bool was_explored(const GridState& state) const;
    void mark_as_explored(const GridState& state);

    void mark_all_states_as_unexplored();

private:
    void initializeDefaultActions();

    size_t R_;
    size_t C_;
    vector<GridAction> actions_;
    static map<GridState, vector<SkillAction>> skill_actions_;

    vector<vector<bool>> obstacle_matrix_;
    vector<vector<bool>> exploration_matrix_;
    vector<GridState> all_domain_states_;
};

#endif // GRID_WORLD_DOMAIN_H
