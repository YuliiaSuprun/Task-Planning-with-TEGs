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
#include "PDDLDomain.h"
#include "PDDLProblem.h"
#include <pddlboat/utility.hpp>
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>
#include <pddlboat/parser/ast.hpp>
#include <pddlboat/solver/expressions.hpp>
#include <pddlboat/solver/domain.hpp>
#include <pddlboat/solver/planner.hpp>
#include <pddlboat/solver/fdplanner.hpp>
#include <pddlboat/solver/z3planner.hpp>
#include <pddlboat/solver/astarplanner.hpp>

using namespace std;

// Example of the command
// ./run.sh /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/domain.pddl /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/e03.pddl -f -c -h


int main(int argc, char** argv) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <PDDL Domain File> <PDDL Problem File> [-c] [-f] [-l] [-h]" << endl;
        exit(EXIT_FAILURE);
    }

    string domainFilePath = argv[1];
    string problemFilePath = argv[2];
    bool cache = false;
    bool feedback = false;
    bool use_landmarks = false;
    bool hamming_dist = false;

    for (int i = 3; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "-c") {
            cache = true;
        } else if (arg == "-f") {
            feedback = true;
        } else if (arg == "-l") {
            use_landmarks = true;
        } else if (arg == "-h") {
            hamming_dist = true;
        } else {
            cerr << "Unknown argument: " << arg << endl;
            exit(EXIT_FAILURE);
        }
    }

    // Instantiate PDDLDomain and PDDLProblem.
    shared_ptr<PDDLDomain> pddlDomain;
    shared_ptr<PDDLProblem> pddlProblem;
    try {
        pddlDomain = make_shared<PDDLDomain>(domainFilePath);
        pddlProblem = make_shared<PDDLProblem>(problemFilePath, pddlDomain, cache, feedback, use_landmarks, hamming_dist);
    } catch (const std::exception& e) {
        cerr << "Error: " << e.what() << endl;
        exit(EXIT_FAILURE);
    }

    auto pddlboatProblem = pddlProblem->getPddlboatProblemPtr();
    // Do not forget to set a problem!!
    pddlDomain->setProblem(pddlboatProblem);

    auto start = chrono::high_resolution_clock::now();

    // Solve the problem and get the solution path
    vector<ProductState> solution_path = pddlProblem->solve();

    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = end - start;
    std::cout << "Time of searching for a solution: " << elapsed.count() << " seconds" << std::endl;

    if (!solution_path.empty()) {
        cout << "Solution for problem is:" << endl;
        pddlProblem->print_product_path();
        // pddlProblem->print_domain_path();
        // pddlProblem->print_dfa_path();
    } else {
        cout << "No solution found for problem" << endl;
    }

    // // Step 1: Convert problem goal into LTLf formula as a string.
    // pddlboat::ExpressionPtr goalExpression = pddlboatProblem->goal;
    // cout << "The goal is defined as follows: " << endl;
    // goalExpression->toPDDL(std::cout) << std::endl;

    // auto ltlStr = goalExpression->toLTL();
    // cout << "The goal in the LTL format: " << ltlStr << endl;

    // pddlboat::VariableSet vars;
    // goalExpression->getVariables(vars);
    // cout << "Printing variables: ";
    // for (const auto& var : vars.getNames()) {
    //     cout << var << ", ";
    // }
    // cout << endl;

    // auto startState = pddlboatProblem->start; 
    // pddlboat::Assignment ass;


    // set<string> predicates;
    // cout << "Printing the used predicates: ";
    // goalExpression->getUsedPredicates(predicates, startState, ass);
    // for (const auto& pred : predicates) {
    //     cout << pred << ", ";
    // }
    // cout << endl;

    // cout << endl << "Printing the predicate mapping: ";

    // // The value contains the name of the predicate and its grounding (a vector of objects).
    // map<string, pair<string, vector<string>>> pred_mapping;

    // goalExpression->getAtomicPropsMap(pred_mapping);
    // for (const auto& pred_pair : pred_mapping) {
    //     cout << pred_pair.first << ": predicate is " << pred_pair.second.first << " with grounding: ";
    //     for (const auto& value : pred_pair.second.second) {
    //         cout << value << "; ";
    //     }
    //     cout << "=====" << endl;
    // }
    
    // std::vector<pddlboat::State::Key> allTruePreds = startState->getTruePredicates();
    // for (const auto& key : allTruePreds) {
    //     cout << "Predicate is " << key.first << " with assignment: " << endl;
    //     for (const auto& value : key.second) {
    //         cout << value << "; ";
    //     }
    //     cout << "=====" << endl;
    // }

    // Solve the problem
    // pddlboat::Z3Planner::Options options;
    // // options.dump_clauses = true;
    // // options.horizon.max = 4;
    // // auto task_planner = make_shared<pddlboat::Z3Planner>(pddlboatProblem, options);
    // auto task_planner = make_shared<pddlboat::AStarPlanner>(pddlboatProblem);

    // cout << "Plan:" << endl;
    // auto plan = make_shared<pddlboat::Plan>(pddlboatProblem);
    // if (!task_planner->solve(*plan))
    // {
    //     cerr << "Failed to solve!" << endl;
    //     return 1;
    // }
    // cout << *plan << endl;
    return 0;
}