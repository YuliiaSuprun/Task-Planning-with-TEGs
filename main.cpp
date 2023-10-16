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
    domain.create_random_obstacle_matrix(0.2);

    // Define default locations for atomic propositions
    GridState goal(19, 19);
    GridState checkpoint(9, 10);
    // Creat a set of hazardous locations.
    set<GridState> hazards;
    for (size_t i = 5; i < 15; ++i) {
        hazards.emplace(i, 11);
    }

    vector<LTLFormula> ltl_formula_list = {
    {"F g", {{"g", set<GridState>{goal}}}}, // Eventually reach a goal.
    {"(F g) & G(!h)", {{"g", set<GridState>{goal}}, {"h", hazards}}}};
    // Eventually reach a goal, while avoiding hazards.

    GridState start_grid_state(0, 0);


    cout << ltl_formula_list.at(0).get_formula() << endl;

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
