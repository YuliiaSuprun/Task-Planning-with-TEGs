#ifndef GRID_WORLD_DOMAIN_H
#define GRID_WORLD_DOMAIN_H

#include <vector>
#include "GridState.h"

using namespace std;

class GridWorldDomain {
public:
    GridWorldDomain(size_t R, size_t C, 
                    const vector<vector<bool>>& obstacle_matrix = {},
                    const vector<GridState>& actions = {});
    
    size_t R() const { return R_; }
    size_t C() const { return C_; }
    
    bool is_valid_state(const GridState& state) const;
    bool is_valid_transition(const GridState& s1, const GridState& action, const GridState& s2) const;
    const vector<vector<bool>>& getObstacleMatrix() const { return obstacle_matrix_; }

private:
    void initializeDefaultActions();
    void initializeDefaultObstacleMatrix();

    size_t R_;
    size_t C_;
    vector<GridState> actions_;
    vector<vector<bool>> obstacle_matrix_;
};

#endif // GRID_WORLD_DOMAIN_H
