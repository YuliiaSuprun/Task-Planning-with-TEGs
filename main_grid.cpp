#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <utility>
#include <chrono>
#include "GridState.h"
#include "GridWorldDomain.h"
#include "GridWorldPlotter.h"

#include "TEGProblem.h"

using namespace std;

int main() {

    // On-the-fly Product Graph Construction Flag.
    bool on_the_fly = true;

    // Caching flag.
    bool cache = true;

    // Real-time planning feedback: success and failure scores used.
    bool feedback = true;

    // Support for equivalence regions guidance: informed search in task domain.
    bool use_landmarks = true;

    // GridWorldDomain dimensions.
    double scale = 1; 
    size_t R = 20 * scale + 1;
    size_t C = 20 * scale + 1;

    auto domain = make_shared<GridWorldDomain>(R, C);
    // Can create domain obstacles.
    domain->create_random_obstacle_matrix(0.1);

    // Define default locations for atomic propositions
    auto goal = make_shared<GridState>(1, 19);
    auto checkpoint1 = make_shared<GridState>(19, 19);
    auto checkpoint2 = make_shared<GridState>(15, 10);
    auto checkpoint3 = make_shared<GridState>(1, 5);
    // Creat a set of hazardous locations.
    DomainStateSet hazards;
    for (size_t i = 5; i < 21; ++i) {
        hazards.insert(make_shared<GridState>(5, i));
    }

    // Use a shared mapping of atomic propositions to grid domain states.
    map<string, DomainStateSet> ap_mapping {
        {"g", DomainStateSet{goal}}, 
        {"h", hazards}, 
        {"c1", DomainStateSet{checkpoint1}}, 
        {"c2", DomainStateSet{checkpoint2}}, 
        {"c3", DomainStateSet{checkpoint3}}};

    // Create a list of LTLf formulas that we want to plan for.
    vector<pair<string, map<string, DomainStateSet>>> ltl_formula_list = {
    // Eventually reach a goal.
    {"F g", ap_mapping},
    // Reach a goal, while avoiding hazards.
    {"(F g) & G(!h)", ap_mapping},
    // Reach a checkpoint and a goal (in any order), while avoiding hazards.
    {"F g & F c1 & G(!h)", ap_mapping},
    // Reach a checkpoint first, then reach a goal (in this order), while avoiding hazards.
    {"(F c1) & G(c1 -> Fg) & G(!h)", ap_mapping},
    // Reach multiple 2 checkpoints and goal (in specific order), while avoiding hazards.
    {"(F c1) & G(c1 -> ((F c2) & G(c2 -> Fg))) & G(!h)", ap_mapping},
    // Reach multiple 3 checkpoints and goal (in specific order), while avoiding hazards.
    {"(F c1) & G(c1 -> ((F c2) & G(c2 -> ((F c3) & G(c3 -> Fg))))) & G(!h)", ap_mapping}};

    // Set the starting grid state.
    auto start_grid_state = make_shared<GridState>(1, 0);

    // Iterate over all formulas and solve the TEGProblem for each of them.
    for (size_t problem_id = 0; problem_id < ltl_formula_list.size(); ++problem_id) {
        auto formula_pair = ltl_formula_list.at(problem_id);
        string formula = formula_pair.first;
        auto ap_mapping = formula_pair.second;

        cout << "Solving for problem_id=" << problem_id << " and formula=" << formula << endl;

        // Initialize the TEG Problem.
        TEGProblem problem(formula, ap_mapping, domain, start_grid_state, problem_id, on_the_fly, cache, feedback, use_landmarks);

        auto start = chrono::high_resolution_clock::now();

        // Solve the problem and get the solution path
        vector<ProductState> solution_path = problem.solve();

        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = end - start;
        std::cout << "Time of searching for a solution: " << elapsed.count() << " seconds" << std::endl;
    
        if (!solution_path.empty()) {
            cout << "Solution for problem_id=" << problem_id << " is:" << endl;
            problem.print_product_path();
            problem.print_domain_path();
            problem.print_dfa_path();
            GridWorldPlotter plotter(domain);
            plotter.visualize_path(problem);
        } else {
            cout << "No solution found for problem_id=" << problem_id << endl;
        }
    }

    return 0;
}
