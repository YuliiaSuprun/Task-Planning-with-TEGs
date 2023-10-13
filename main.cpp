#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <utility>
#include "GridState.h"
#include "LTLFormula.h"
// #include "utility_funcs.h"
// #include "GridWorldDomain.h"
// #include "TEG_Task.h"

using namespace std;

int main() {
    // Settings
    // int scale = 1;
    // GridWorldDomain domain(20 * scale + 1, 20 * scale + 1);

    // Define default locations for atomic propositions
    GridState goal(19, 19);
    GridState checkpoint(10, 10);
    GridState hazard(9, 9);
    GridState start_grid_state(0, 0);

    // vector<vector<bool>> obstacle_matrix = create_border_obstacle_matrix(domain.R(), domain.C());

    vector<LTLFormula> ltl_formula_list = {
    {"F g", {{"g", set<GridState>{goal}}}},
    {"(F g) & (F c)", {{"g", set<GridState>{goal}}, {"c", set<GridState>{checkpoint}}}},
    {"(G (!h)) & (F d)", {{"g", set<GridState>{goal}}, {"h", set<GridState>{hazard}}}},
    {"G (!h) & F c & (G !g U c) & (c -> F g)", {{"g", set<GridState>{goal}}, {"h", set<GridState>{hazard}}, {"c", set<GridState>{checkpoint}}}}};

    cout << ltl_formula_list.at(0).get_formula() << endl;

    // for (size_t SCENE_ID = 0; SCENE_ID < ltl_formula_list.size(); ++SCENE_ID) {
    //     LTLFormula entry = ltl_formula_list[SCENE_ID];
    //     std::cout << "Solving for SCENE_ID=" << SCENE_ID << " and ltl_formula=" << entry.formula << std::endl;

    //     // Create the TEG_Task instance with the chosen LTLf formula
    //     TEG_Task task(entry.formula, domain, entry.ap_mapping, start_grid_state, obstacle_matrix, SCENE_ID);

    //     // Compute the product automaton
    //     task.compute_product();

    //     // Solve the task and get the solution path
    //     std::vector<std::array<int, 2>> solution_path = task.solve();
    //     if (!solution_path.empty()) {
    //         std::cout << "Solution for SCENE_ID " << SCENE_ID << " is:";
    //         for (const auto& state : solution_path) {
    //             std::cout << " [" << state[0] << "," << state[1] << "]";
    //         }
    //         std::cout << std::endl;
    //     } else {
    //         std::cout << "No solution found for SCENE_ID " << SCENE_ID << std::endl;
    //     }
    // }

    return 0;
}
