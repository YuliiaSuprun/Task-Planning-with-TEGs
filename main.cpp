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
// #include "utility_funcs.h"
#include "TEGTask.h"

using namespace std;

int main() {
    // GridWorldDomain dimensions.
    double scale = 1;
    size_t R = 20 * scale + 1;
    size_t C = 20 * scale + 1;

    // Create the obstacle matrix.
    // vector<vector<bool>> obstacle_matrix = create_border_obstacle_matrix(R, C); 

    GridWorldDomain domain(R, C);

    // Define default locations for atomic propositions
    GridState goal(19, 19);
    GridState checkpoint(10, 10);
    GridState hazard(9, 9);
    GridState start_grid_state(0, 0);

    vector<LTLFormula> ltl_formula_list = {
    {"F g", {{"g", set<GridState>{goal}}}},
    {"(F g) & (F c)", {{"g", set<GridState>{goal}}, {"c", set<GridState>{checkpoint}}}},
    {"(G (!h)) & (F d)", {{"g", set<GridState>{goal}}, {"h", set<GridState>{hazard}}}},
    {"G (!h) & F c & (G !g U c) & (c -> F g)", {{"g", set<GridState>{goal}}, {"h", set<GridState>{hazard}}, {"c", set<GridState>{checkpoint}}}}};

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
