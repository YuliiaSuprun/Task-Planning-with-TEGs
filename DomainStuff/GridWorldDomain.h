#ifndef GRID_WORLD_DOMAIN_H
#define GRID_WORLD_DOMAIN_H

#include <vector>
#include <set>
#include <algorithm> 
#include <map>
#include "Domain.h"
#include "DomainState.h"
#include "GridState.h"
#include "GridAction.h"

using namespace std;

class GridWorldDomain : public Domain{
public:
    GridWorldDomain(size_t R, size_t C, 
                    const vector<vector<bool>>& obstacle_matrix = {},
                    const vector<shared_ptr<GridAction>>& actions = {});

    // Overriding virtual methods from Domain
    // const vector<shared_ptr<PrimitiveAction>>& get_actions(const DomainState& state) const override;
    const vector<shared_ptr<DomainState>>& get_all_states() override;
    const vector<shared_ptr<DomainState>> get_successor_states(const DomainState& curr_state) override;
    const vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> get_successor_state_action_pairs(const DomainState& curr_state) override;
    void mark_all_states_as_unexplored() override;
    bool is_valid_state(const DomainState& state) const override;
    void mark_as_explored(const DomainState& state) override;

    // Getters.
    size_t R() const { return R_; }
    size_t C() const { return C_; }
    const vector<vector<bool>>& get_obstacle_matrix() const { return obstacle_matrix_; }

    set<GridState> get_obstacle_set() const;


    bool is_obstacle(const GridState& state) const;
    bool is_valid_transition(const GridState& s1, const GridState& action, const GridState& s2) const;

    // Functions for creating obstacle matrices.
    void add_rectangle_obstacle(double rmin, double rmax, double cmin, double cmax);
    void create_empty_obstacle_matrix();
    void create_border_obstacle_matrix();
    void create_corner_obstacle_matrix();
    void create_blocks_obstacle_matrix();
    void create_corridor_obstacle_matrix();
    void create_random_obstacle_matrix(double density);

    bool was_explored(const GridState& state) const;

private:
    void initializeActions();

    size_t R_;
    size_t C_;
    vector<shared_ptr<GridAction>> actions_;

    vector<vector<bool>> obstacle_matrix_;
    vector<vector<bool>> exploration_matrix_;
    vector<shared_ptr<DomainState>> all_domain_states_;
};

#endif // GRID_WORLD_DOMAIN_H
