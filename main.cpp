#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <utility>
#include "GridState.h"
#include "GridWorldDomain.h"
#include "GridWorldPlotter.h"
#include "LTLFormula.h"

#include "TEGTask.h"

using namespace std;

int main() {
    // GridWorldDomain dimensions.
    double scale = 1;
    size_t R = 20 * scale + 1;
    size_t C = 20 * scale + 1;

    GridWorldDomain domain(R, C);
    // Can create domain obstacles.
    // domain.create_random_obstacle_matrix(0.2);

    // Define default locations for atomic propositions
    GridState goal(1, 19);
    GridState checkpoint1(19, 19);
    GridState checkpoint2(19, 1);
    GridState checkpoint3(2, 2);
    // Creat a set of hazardous locations.
    set<GridState> hazards;
    for (size_t i = 5; i < 21; ++i) {
        hazards.emplace(5, i);
    }

    // Create a list of LTLf formulas that we want to plan for.
    vector<LTLFormula> ltl_formula_list = {
    // Eventually reach a goal.
    {"F g", {{"g", set<GridState>{goal}}}},
    // Reach a goal, while avoiding hazards.
    {"(F g) & G(!h)", {{"g", set<GridState>{goal}}, {"h", hazards}}},
    // Reach a checkpoint and a goal (in any order), while avoiding hazards.
    {"F c & F g & G(!h)", {{"g", set<GridState>{goal}}, {"h", hazards}, {"c", set<GridState>{checkpoint1}}}},
    // Reach a checkpoint first, then reach a goal (in this order), while avoiding hazards.
    {"(F c) & G(c -> Fg) & G(!h)", {{"g", set<GridState>{goal}}, {"h", hazards}, {"c", set<GridState>{checkpoint1}}}},
    // Reach multiple checkpoints (in specific order), while avoiding hazards.
    {"(F c1) & G(c1 -> ((F c2) & G(c2 -> Fg))) & G(!h)", {{"g", set<GridState>{goal}}, {"h", hazards}, {"c1", set<GridState>{checkpoint1}}, {"c2", set<GridState>{checkpoint2}}, {"c3", set<GridState>{checkpoint3}}}}
    };

    // Set the starting grid state.
    GridState start_grid_state(0, 0);

    // Iterate over all formulas and solve the TEGTask for each of them.
    for (size_t task_id = 0; task_id < ltl_formula_list.size(); ++task_id) {
        LTLFormula formula = ltl_formula_list.at(task_id);
        cout << "Solving for task_id=" << task_id << " and formula=" << formula << endl;

        TEGTask task(formula, domain, start_grid_state, task_id);

        // Solve the task and get the solution path
        vector<ProductState> solution_path = task.solve();
    
        if (!solution_path.empty()) {
            cout << "Solution for task_id=" << task_id << " is:" << endl;
            task.print_product_path();
            task.print_grid_path();
            task.print_dfa_path();
            GridWorldPlotter plotter(domain);
            plotter.visualize_path(task);
        } else {
            cout << "No solution found for task_id=" << task_id << endl;
        }
    }

    return 0;
}
