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
    const vector<GridAction>& get_actions() const { return actions_; }
    const vector<SkillAction>& get_skill_actions(GridState state) { return skill_actions_[state]; }
    set<GridState> get_obstacle_set() const;


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

private:
    void initializeDefaultActions();

    size_t R_;
    size_t C_;
    vector<GridAction> actions_;
    static map<GridState, vector<SkillAction>> skill_actions_;

    vector<vector<bool>> obstacle_matrix_;
};

#endif // GRID_WORLD_DOMAIN_H
